#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <unordered_map>

#include "candle_ros2/md_node.hpp"

namespace
{
using Bytes = std::vector<u8>;

// Emulates only the synchronous MD register protocol used by these callbacks.
// No USB transport, ROS executor, robot services, or hardware are involved.
class RegisterBus final : public mab::I_CommunicationInterface
{
  public:
    size_t writes = 0;
    size_t reads = 0, largestReadFrame = 0;
    std::vector<size_t> readRequestSizes;
    size_t stateWrites = 0, modeWrites = 0, zeroWrites = 0;
    std::vector<float> torqueCommands;
    bool rejectZero = false;
    bool rejectStateRead = false;
    bool rejectAllReads = false;
    std::unordered_map<u16, Bytes> registers;
    mab::MDRegisters_S schema;

    RegisterBus()
    {
        std::apply([&](auto&... reg)
            { ((registers[reg.m_regAddress] = Bytes(reg.getSize(), 0)), ...); }, schema.getAllRegisters());
        set(schema.state, static_cast<u16>(64));
        set(schema.canID, static_cast<u32>(113));
        set(schema.motionModeStatus, static_cast<u8>(0));
        set(schema.mainEncoderPosition, 0.25f);
    }
    template<class T>
    void set(const mab::MDRegisterEntry_S<T>& reg, T value)
    {
        auto& bytes = registers.at(reg.m_regAddress);
        std::memcpy(bytes.data(), &value, sizeof(value));
    }
    template<class T>
    T get(const mab::MDRegisterEntry_S<T>& reg) const
    {
        T value{};
        std::memcpy(&value, registers.at(reg.m_regAddress).data(), sizeof(value));
        return value;
    }
    u16 state() const
    {
        u16 value = 0;
        std::memcpy(&value, registers.at(schema.state.m_regAddress).data(), sizeof(value));
        return value;
    }
    Error_t connect() override { return OK; }
    Error_t disconnect() override { return OK; }
    Error_t transfer(Bytes, u32) override { return OK; }
    std::pair<Bytes, Error_t> transfer(Bytes request, u32, size_t) override
    {
        if (request[0] == 2)
            return {{2, 1, 'r', 4, 5, 2}, OK};
        if (request.size() < 7 || request[0] != 4)
            return {{}, TRANSMITTER_ERROR};
        const bool writing = request[5] == 0x40;
        if (writing) ++writes;
        else
        {
            ++reads;
            largestReadFrame = std::max(largestReadFrame, request.size() - 5);
            readRequestSizes.push_back(request.size()); // Full USB request, including five-byte envelope.
            if (rejectAllReads) return {{}, TIMEOUT};
        }
        Bytes result{4, 1, writing ? u8{0xA0} : u8{0x41}, 0};
        for (size_t offset = 7; offset < request.size();)
        {
            if (request.size() - offset < 2)
                return {{}, TRANSMITTER_ERROR};
            const u16 address = request[offset] | (static_cast<u16>(request[offset + 1]) << 8);
            auto found = registers.find(address);
            if (found == registers.end() || request.size() - offset - 2 < found->second.size())
                return {{}, TRANSMITTER_ERROR};
            auto& bytes = found->second;
            if (writing)
            {
                if (rejectZero && address == schema.runZero.m_regAddress)
                    return {{4, 1, 0xA0, 0}, TIMEOUT};
                std::copy_n(request.begin() + offset + 2, bytes.size(), bytes.begin());
                if (address == schema.state.m_regAddress) ++stateWrites;
                if (address == schema.motionModeCommand.m_regAddress) ++modeWrites;
                if (address == schema.runZero.m_regAddress) ++zeroWrites;
                if (address == schema.targetTorque.m_regAddress)
                    torqueCommands.push_back(get(schema.targetTorque));
            }
            else
            {
                if (rejectStateRead && address == schema.state.m_regAddress)
                    return {{}, TIMEOUT};
                result.push_back(request[offset]);
                result.push_back(request[offset + 1]);
                result.insert(result.end(), bytes.begin(), bytes.end());
            }
            offset += 2 + bytes.size();
        }
        return {result, OK};
    }
};

candleParams_S testParams()
{
    candleParams_S params{};
    params.default_qos = "BestEffort";
    params.joint_name_prefix = "test_drive_";
    params.gripper_open_position_rad = 0.0;
    params.gripper_closed_position_rad = 0.75;
    params.gripper_impedance_kp = 30.0;
    params.gripper_impedance_kd = 0.2;
    params.gripper_velocity_limit_rad_s = 2.0;
    params.gripper_open_velocity_limit_rad_s = 10.0;
    params.close.openLead = .14;
    params.gripper_torque_limit_nm = 2.0;
    params.soft_close_slow_torque_limit_nm = 2.0;
    params.homing_required = true;
    params.joint_state_publish_period_ms = 10;
    params.health_publish_period_ms = 200;
    params.md_can_response_timeout_100us = 10;
    params.md_host_timeout_ms = 3;
    params.encoder_wrap_period_rad = 6.283185307;
    params.position_recovery_max_delta_rad = 0.1;
    params.position_recovery_samples = 3;
    params.position_recovery_retry_ms = 100;
    params.init_devices_zero = false;
    return params;
}
}

class MdNodeTestAccess
{
  public:
    static void seedDrive(MdNode& node)
    {
        node.m_mds.emplace_back(113, node.m_candle.get());
        node.m_positionTrackers.emplace(113, PositionTracker(6.283185307, 0.1, 3));
        node.m_positionTrackers.at(113).initialize(0.25);
        node.m_gripperCalibrations.emplace(113, GripperHoming::Calibration{0.75, node.closeConfig, 0.0});
        node.m_autoHoming.activate(113);
    }
    static void hold(MdNode& node)
    {
        ImpedanceClose controller(0.25, 0.75, 2.0, node.closeConfig);
        controller.state = ImpedanceClose::State::TorqueHold;
        controller.holdingTorque = 0.25;
        node.m_softCloseJobs.emplace(113, MdNode::SoftCloseJob{controller, std::chrono::steady_clock::now(), false});
    }
    static bool holding(const MdNode& node)
    { return node.m_softCloseJobs.contains(113) && !node.m_softCloseJobs.at(113).opening; }
    static bool opening(const MdNode& node)
    { return node.m_softCloseJobs.contains(113) && node.m_softCloseJobs.at(113).opening; }
    static bool homing(const MdNode& node) { return node.isHoming(113); }
    static bool inhibited(const MdNode& node) { return node.m_motionInhibited.contains(113); }
    static bool autoHomePending(const MdNode& node) { return node.m_autoHoming.pending(113); }
    static bool needsHome(const MdNode& node) { return node.m_homingRequired.contains(113); }
    static PositionTracker::State reference(const MdNode& node) { return node.m_positionTrackers.at(113).state(); }
    static bool acceptsMotion(const MdNode& node) { return node.canAcceptPositionCommand(113); }
    static void feedback(MdNode& node) { node.publishJointStates(); }
    static void motion(MdNode& node, const candle_ros2::msg::MotionCmd& message) { node.cbMotionCmd(message); }
    static void pid(MdNode& node, const candle_ros2::msg::PositionPidCmd& message) { node.cbPositionCmd(message); }
    static void impedance(MdNode& node, const candle_ros2::msg::ImpedanceCmd& message) { node.cbImpedanceCmd(message); }
    static auto target(MdNode& node, double position)
    {
        auto request = std::make_shared<candle_ros2::srv::SetGripperTargets::Request>();
        auto response = std::make_shared<candle_ros2::srv::SetGripperTargets::Response>();
        request->device_ids = {113};
        request->target_position_rad = {position};
        node.cbSetGripperTargets(request, response);
        return response;
    }
    static auto limits(MdNode& node, std::vector<float> velocity, std::vector<float> torque)
    {
        auto request = std::make_shared<candle_ros2::srv::SetLimits::Request>();
        auto response = std::make_shared<candle_ros2::srv::SetLimits::Response>();
        request->device_ids = {113};
        request->velocity_limit = std::move(velocity);
        request->torque_limit = std::move(torque);
        node.cbSetLimits(request, response);
        return response;
    }
    static auto zero(MdNode& node)
    {
        auto request = std::make_shared<candle_ros2::srv::Generic::Request>();
        auto response = std::make_shared<candle_ros2::srv::Generic::Response>();
        request->device_ids = {113};
        node.cbZero(request, response);
        return response;
    }
    static auto open(MdNode& node)
    {
        auto request = std::make_shared<candle_ros2::srv::Generic::Request>();
        auto response = std::make_shared<candle_ros2::srv::Generic::Response>();
        request->device_ids = {113};
        node.cbOpenGripper(request, response);
        return response;
    }
    static const auto& motionControl(const MdNode& node)
    { return node.m_softCloseJobs.at(113).control; }
    static const auto& calibration(const MdNode& node)
    { return node.m_gripperCalibrations.at(113); }
    static bool calibrated(const MdNode& node)
    { return node.m_gripperCalibrations.contains(113); }
    static void tickMotion(MdNode& node, double position, double velocity = 0.0,
                           double torque = 0.0, int elapsedMs = 10)
    {
        node.m_softCloseJobs.at(113).lastUpdate =
            std::chrono::steady_clock::now() - std::chrono::milliseconds(elapsedMs);
        node.tickSoftClose(node.m_mds.front(), position, velocity, torque);
    }
    static void setFeedbackMode(MdNode& node, mab::MdMode_E mode)
    {
        node.m_mds.front().m_mdRegisters.motionModeStatus = mode;
    }
    static void setFeedbackState(MdNode& node, u16 state)
    {
        node.m_mds.front().m_mdRegisters.state = state;
    }
    static void expireMotionVerification(MdNode& node)
    {
        node.m_softCloseJobs.at(113).lastVerification =
            std::chrono::steady_clock::now() - std::chrono::milliseconds(101);
    }
    static void disableThenLoseCommunication(MdNode& node)
    {
        auto request = std::make_shared<candle_ros2::srv::Generic::Request>();
        request->device_ids = {113};
        node.cbDisable(std::make_shared<rmw_request_id_t>(), request);
        node.beginRecovery(113);
    }
    static void beginHomingReleaseDisable(MdNode& node)
    {
        // Stage the same asynchronous disable used after a powered stop. This
        // deliberately does not call /disable or stopDrive: neither has latched
        // automatic recovery when an ordinary release starts.
        GripperHoming sequence(.25, -1, node.homeConfig, node.closeConfig, .5);
        sequence.control.state = ImpedanceHome::State::Releasing;
        node.m_homingJob.emplace(MdNode::HomingJob{
            113, sequence, std::chrono::steady_clock::now(), nullptr});
        node.m_homingRequired.insert(113);
        node.m_motionInhibited.insert(113);
        node.m_pendingDisables.try_emplace(113, node.m_mds.front());
    }
    static void expireDisable(MdNode& node)
    {
        node.m_pendingDisables.at(113).started -=
            candle_ros2::DisableTransition::timeout + std::chrono::milliseconds(1);
    }
    static void pollDisable(MdNode& node) { node.tickDisableJobs(); }
    static void loseCommunication(MdNode& node) { node.beginRecovery(113); }
    static std::string diagnosticRegisters(MdNode& node, int group)
    { return node.readDiagnosticRegisters(node.m_mds.front(), group, 0); }
    static void diagnostics(MdNode& node) { node.publishDiagnostics(); }
    static void expireDiagnosticPeriod(MdNode& node)
    { node.m_lastDiagnosticRead -= std::chrono::milliseconds(201); }
    static const auto& diagnosticHistory(const MdNode& node) { return node.m_diagnostics.at(113); }
    static void flushDiagnostics(MdNode& node) { node.flushDiagnosticFaults(); }
    static bool diagnosticReadsDisabled(const MdNode& node) { return node.m_diagnosticPollingDisabled; }
    static bool diagnosticStartupReady(const MdNode& node) { return node.m_diagnosticStartupReady; }
    static void clearDiagnosticCalibration(MdNode& node)
    {
        node.m_gripperCalibrations.clear();
        node.m_homingRequired.insert(113);
    }
    static void seedDiagnosticStartup(MdNode& node)
    {
        clearDiagnosticCalibration(node);
        for (const u16 id : {447, 111, 345, 343, 342})
        {
            node.m_mds.emplace_back(id, node.m_candle.get());
            node.m_homingRequired.insert(id);
        }
    }
    static void completeDiagnosticCalibration(MdNode& node, u16 id)
    {
        node.m_gripperCalibrations.insert_or_assign(id,
            GripperHoming::Calibration{0.75, node.closeConfig, 0.0});
        node.m_homingRequired.erase(id);
    }
    static void diagnosticHoming(MdNode& node, bool active)
    {
        if (!active) { node.m_homingJob.reset(); return; }
        GripperHoming sequence(.25, -1, node.homeConfig, node.closeConfig, .5);
        node.m_homingJob.emplace(MdNode::HomingJob{
            113, sequence, std::chrono::steady_clock::now(), nullptr});
    }
    static void pendingDiagnosticDisable(MdNode& node)
    { node.m_pendingDisables.try_emplace(113, node.m_mds.front()); }
    static void clearDiagnosticDisables(MdNode& node) { node.m_pendingDisables.clear(); }
    static void bypassDiagnosticHoming(MdNode& node) { node.homingRequired = false; }
    static void explicitDisable(MdNode& node)
    {
        auto request = std::make_shared<candle_ros2::srv::Generic::Request>();
        request->device_ids = {113};
        node.cbDisable(std::make_shared<rmw_request_id_t>(), request);
    }
    static bool hasDiagnosticFault(const MdNode& node)
    {
        const auto found = node.m_diagnostics.find(113);
        return found != node.m_diagnostics.end() &&
            (found->second.pending || found->second.lastFaultNs || found->second.suppressed);
    }
    static void recordAndStop(MdNode& node)
    {
        auto& md = node.m_mds.front();
        md.m_mdRegisters.mainEncoderPosition = .02f;
        md.m_mdRegisters.mainEncoderVelocity = .001f;
        md.m_mdRegisters.motorTorque = -.002f;
        md.m_mdRegisters.state = 39;
        md.m_mdRegisters.motionModeStatus = mab::MdMode_E::IMPEDANCE;
        node.recordDiagnosticFeedback(md, true, .02);
        node.stopDrive(md, "test fault before disable");
    }
};

namespace
{
class MdNodeCallbacks : public ::testing::Test
{
  protected:
    RegisterBus* bus = nullptr;
    std::shared_ptr<mab::Candle> candle;
    std::shared_ptr<MdNode> node;
    static void SetUpTestSuite() { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override
    {
        Logger::g_m_verbosity = Logger::Verbosity_E::SILENT;
        auto connection = std::make_unique<RegisterBus>();
        bus = connection.get();
        candle = std::make_shared<mab::Candle>(mab::CAN_DATARATE_1M, std::move(connection));
        ASSERT_EQ(candle->init(), mab::candleTypes::Error_t::OK);
        rclcpp::NodeOptions options;
        options.start_parameter_services(false).start_parameter_event_publisher(false);
        node = std::make_shared<MdNode>(options, candle, testParams());
        MdNodeTestAccess::seedDrive(*node);
        ASSERT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
    }
    void TearDown() override { node.reset(); candle.reset(); }
    void recreateWithTravelLimits(bool enabled)
    {
        node.reset();
        auto params = testParams();
        params.close.travelLimitsEnabled = enabled;
        rclcpp::NodeOptions options;
        options.start_parameter_services(false).start_parameter_event_publisher(false);
        node = std::make_shared<MdNode>(options, candle, params);
        MdNodeTestAccess::seedDrive(*node);
    }
    candle_ros2::msg::MotionCmd motionCommand()
    {
        candle_ros2::msg::MotionCmd command;
        command.device_ids = {113};
        command.target_position = {0.25f};
        command.target_velocity = {0.0f};
        command.target_torque = {0.0f};
        return command;
    }
    void expectFailedHomingReleaseStaysStopped(bool communicationFailure)
    {
        MdNodeTestAccess::beginHomingReleaseDisable(*node);
        ASSERT_TRUE(MdNodeTestAccess::homing(*node));
        if (communicationFailure)
            bus->rejectStateRead = true;
        else
        {
            // The write was acknowledged but enabled readback never changes.
            bus->set(bus->schema.state, static_cast<u16>(39));
            MdNodeTestAccess::expireDisable(*node);
        }
        MdNodeTestAccess::pollDisable(*node);
        EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
        EXPECT_FALSE(MdNodeTestAccess::opening(*node));
        EXPECT_FALSE(MdNodeTestAccess::homing(*node));
        EXPECT_FALSE(MdNodeTestAccess::acceptsMotion(*node));

        // Restored communication followed by further losses must not turn a
        // failed release into permission to run another mechanical homing.
        bus->rejectStateRead = false;
        bus->set(bus->schema.state, static_cast<u16>(64));
        for (int i = 0; i < 3; ++i)
        {
            MdNodeTestAccess::loseCommunication(*node);
            EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
        }
    }
};

TEST_F(MdNodeCallbacks, DiagnosticsReadWatchdogWarningEvenWhenQuickStatusIsClean)
{
    // Unsupported on the deployed firmware; no diagnostic group may query it.
    bus->registers.erase(bus->schema.miscStatus.m_regAddress);
    bus->set(bus->schema.quickStatus, static_cast<u16>(0));
    bus->set(bus->schema.communicationStatus, uint32_t{1} << 30);
    bus->set(bus->schema.canWatchdog, static_cast<u16>(100));
    const auto status = MdNodeTestAccess::diagnosticRegisters(*node, 0);
    EXPECT_NE(status.find("\"quick_status\":0"), std::string::npos);
    const auto protection = MdNodeTestAccess::diagnosticRegisters(*node, 1);
    EXPECT_NE(protection.find("\"communication_status\":1073741824"), std::string::npos);
    EXPECT_NE(protection.find("\"communication_can_watchdog_warning\":true"), std::string::npos);
    EXPECT_NE(protection.find("\"misc_status\":null"), std::string::npos);
    const auto command = MdNodeTestAccess::diagnosticRegisters(*node, 2);
    EXPECT_NE(command.find("\"can_watchdog_ms\":100"), std::string::npos);
    for (int group = 3; group < 6; ++group) MdNodeTestAccess::diagnosticRegisters(*node, group);
    EXPECT_EQ(bus->reads, 6u);
    EXPECT_EQ(bus->readRequestSizes, (std::vector<size_t>{35, 31, 36, 37, 25, 22}));
    for (const auto fullRequestSize : bus->readRequestSizes) EXPECT_LE(fullRequestSize, 37u);
    EXPECT_EQ(bus->writes, 0u);
    EXPECT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
}

TEST_F(MdNodeCallbacks, DiagnosticReadFailureIsExplicitAndDoesNotChangeControl)
{
    bus->rejectStateRead = true;
    const auto record = MdNodeTestAccess::diagnosticRegisters(*node, 2);
    EXPECT_NE(record.find("\"read_ok\":false"), std::string::npos);
    EXPECT_NE(record.find("\"values\":null"), std::string::npos);
    EXPECT_EQ(bus->writes, 0u);
    EXPECT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
}

TEST_F(MdNodeCallbacks, DiagnosticSchedulerAddsAtMostOneReadPerPeriod)
{
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 1u);
    for (int i = 0; i < 10; ++i) MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 1u);
    MdNodeTestAccess::expireDiagnosticPeriod(*node);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 2u);
    EXPECT_EQ(bus->writes, 0u);
}

TEST_F(MdNodeCallbacks, DiagnosticReadsWaitForAllSixInitialCalibrations)
{
    MdNodeTestAccess::seedDiagnosticStartup(*node);
    for (const u16 id : {113, 447, 111, 345, 343, 342})
    {
        MdNodeTestAccess::expireDiagnosticPeriod(*node);
        MdNodeTestAccess::diagnostics(*node);
        EXPECT_EQ(bus->reads, 0u); // Includes every quiet inter-drive startup gap.
        EXPECT_FALSE(MdNodeTestAccess::diagnosticStartupReady(*node));
        MdNodeTestAccess::completeDiagnosticCalibration(*node, id);
    }
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 1u);
    EXPECT_TRUE(MdNodeTestAccess::diagnosticStartupReady(*node));

    // A later physical fault invalidates calibration but must not erase the
    // startup latch: disabled post-fault evidence remains available.
    MdNodeTestAccess::clearDiagnosticCalibration(*node);
    MdNodeTestAccess::expireDiagnosticPeriod(*node);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 2u);
}

TEST_F(MdNodeCallbacks, DiagnosticReadsSkipAnyHomingAndPendingDisableIncludingFaultRefresh)
{
    MdNodeTestAccess::diagnostics(*node); // Establish the startup latch.
    const auto reads = bus->reads;
    MdNodeTestAccess::diagnosticHoming(*node, true);
    MdNodeTestAccess::expireDiagnosticPeriod(*node);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, reads);
    MdNodeTestAccess::diagnosticHoming(*node, false);
    MdNodeTestAccess::recordAndStop(*node);
    MdNodeTestAccess::diagnostics(*node); // Flushes memory but must not poll during disable.
    EXPECT_EQ(bus->reads, reads);
    EXPECT_EQ(MdNodeTestAccess::diagnosticHistory(*node).refreshGroup, 0);
    MdNodeTestAccess::clearDiagnosticDisables(*node);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, reads + 1);
    EXPECT_EQ(MdNodeTestAccess::diagnosticHistory(*node).refreshGroup, 1);
}

TEST_F(MdNodeCallbacks, DiagnosticStartupCanBypassOptionalHomingButNeverActiveHoming)
{
    MdNodeTestAccess::clearDiagnosticCalibration(*node);
    MdNodeTestAccess::bypassDiagnosticHoming(*node);
    MdNodeTestAccess::diagnosticHoming(*node, true);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 0u);
    MdNodeTestAccess::diagnosticHoming(*node, false);
    MdNodeTestAccess::diagnostics(*node);
    EXPECT_EQ(bus->reads, 1u);
    EXPECT_TRUE(MdNodeTestAccess::diagnosticStartupReady(*node));
}

TEST_F(MdNodeCallbacks, FirstDiagnosticFailurePermanentlyDisablesOptionalPolling)
{
    bus->rejectAllReads = true;
    MdNodeTestAccess::diagnostics(*node);
    ASSERT_EQ(bus->reads, 1u);
    EXPECT_TRUE(MdNodeTestAccess::diagnosticReadsDisabled(*node));
    bus->rejectAllReads = false;
    for (int tick = 0; tick < 12; ++tick)
    {
        MdNodeTestAccess::expireDiagnosticPeriod(*node);
        MdNodeTestAccess::diagnostics(*node);
    }
    EXPECT_EQ(bus->reads, 1u);
    EXPECT_EQ(bus->writes, 0u);
    EXPECT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
    MdNodeTestAccess::recordAndStop(*node);
    MdNodeTestAccess::clearDiagnosticDisables(*node);
    MdNodeTestAccess::expireDiagnosticPeriod(*node);
    MdNodeTestAccess::diagnostics(*node); // Fault history still flushes with the circuit open.
    EXPECT_FALSE(MdNodeTestAccess::diagnosticHistory(*node).pending.has_value());
    EXPECT_EQ(bus->reads, 1u);
    EXPECT_NE(MdNodeTestAccess::diagnosticRegisters(*node, 0).find("\"read_skipped\":true"),
              std::string::npos);
    EXPECT_EQ(bus->reads, 1u);
}

TEST_F(MdNodeCallbacks, ExplicitDisableDoesNotCaptureOrSuppressFollowingRealFault)
{
    MdNodeTestAccess::explicitDisable(*node);
    EXPECT_FALSE(MdNodeTestAccess::hasDiagnosticFault(*node));
    MdNodeTestAccess::recordAndStop(*node); // Same tick, well within the one-second limiter.
    const auto& history = MdNodeTestAccess::diagnosticHistory(*node);
    ASSERT_TRUE(history.pending.has_value());
    EXPECT_EQ(history.pending->sequence, 1u);
    EXPECT_EQ(history.pending->reason, "test fault before disable");
    EXPECT_EQ(history.suppressed, 0u);
}

TEST_F(MdNodeCallbacks, FaultFlushPerformsNoReadsOrWritesAndDestructorRetainsPendingFault)
{
    MdNodeTestAccess::recordAndStop(*node);
    const auto reads = bus->reads, writes = bus->writes;
    MdNodeTestAccess::flushDiagnostics(*node);
    EXPECT_FALSE(MdNodeTestAccess::diagnosticHistory(*node).pending.has_value());
    EXPECT_EQ(bus->reads, reads);
    EXPECT_EQ(bus->writes, writes);
}

TEST_F(MdNodeCallbacks, DestructorLogsPendingFaultBeforeFinishedWithoutRegisterSnapshots)
{
    MdNodeTestAccess::recordAndStop(*node);
    testing::internal::CaptureStderr();
    node.reset();
    const auto log = testing::internal::GetCapturedStderr();
    const auto fault = log.find("\"event\":\"drive_fault\"");
    const auto finished = log.find("Candle ROS2 MD node finished.");
    EXPECT_NE(fault, std::string::npos);
    EXPECT_NE(finished, std::string::npos);
    EXPECT_LT(fault, finished);
    EXPECT_EQ(log.find("\"event\":\"register_snapshot\""), std::string::npos);
}

TEST_F(MdNodeCallbacks, StopCapturesEnabledFeedbackBeforeDisableWithoutDiagnosticReads)
{
    MdNodeTestAccess::hold(*node);
    const auto before = bus->reads;
    MdNodeTestAccess::recordAndStop(*node);
    const auto& history = MdNodeTestAccess::diagnosticHistory(*node);
    ASSERT_TRUE(history.pending.has_value());
    EXPECT_TRUE(history.pending->atStop.valid);
    EXPECT_EQ(history.pending->atStop.state, 39u);
    EXPECT_EQ(history.pending->atStop.mode, 4u);
    EXPECT_EQ(history.pending->atStop.operation, 3);
    EXPECT_DOUBLE_EQ(history.pending->atStop.position, .02);
    EXPECT_EQ(bus->reads, before); // Existing disable writes only; snapshot never polls.
    EXPECT_FALSE(MdNodeTestAccess::holding(*node));
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
}

TEST(DriveDiagnostics, BoundedHistoryKeepsExactFaultTickBetweenDecimatedSamples)
{
    candle_ros2::DriveDiagnosticHistory history;
    candle_ros2::DriveDiagnosticSample sample;
    for (int i = 0; i < 80; ++i)
    {
        sample.steadyNs = 1'000'000'000 + i * history.samplePeriodNs;
        sample.rosNs = sample.steadyNs;
        sample.position = i;
        sample.valid = true;
        history.observe(sample);
    }
    EXPECT_EQ(history.count, 64u);
    EXPECT_DOUBLE_EQ(history.samples[history.next].position, 16);
    sample.steadyNs += 1'000'000;
    sample.valid = false;
    sample.position = NAN;
    history.observe(sample);
    EXPECT_EQ(history.count, 64u);
    ASSERT_TRUE(history.capture(1, sample.rosNs, sample.steadyNs, "failed read"));
    EXPECT_FALSE(history.pending->atStop.valid);
    EXPECT_TRUE(std::isnan(history.pending->atStop.position));
    EXPECT_DOUBLE_EQ(history.pending->atStop.feedbackIntervalMs, 1.0);
    EXPECT_FALSE(history.capture(2, sample.rosNs, sample.steadyNs, "duplicate"));
    EXPECT_EQ(history.suppressed, 1u);
}

TEST(DriveDiagnostics, JsonEscapesReasonsAndMarksUnavailableFeedbackNull)
{
    EXPECT_EQ(candle_ros2::diagnosticQuote("bad\n\"read\"\\"), "\"bad\\u000a\\\"read\\\"\\\\\"");
    const auto record = candle_ros2::DriveDiagnosticSample{}.json();
    EXPECT_NE(record.find("\"feedback_valid\":false"), std::string::npos);
    EXPECT_NE(record.find("\"position_rad\":null"), std::string::npos);
    EXPECT_NE(record.find("\"state\":null"), std::string::npos);
    EXPECT_EQ(record.find(":nan"), std::string::npos);
}

TEST_F(MdNodeCallbacks, RejectsUint32IdWithoutAliasingExistingDrive)
{
    auto command = motionCommand();
    command.device_ids = {65649};
    MdNodeTestAccess::motion(*node, command);
    EXPECT_EQ(bus->writes, 0u);
    EXPECT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
}

TEST_F(MdNodeCallbacks, RejectsNonfiniteAndMalformedMotionWithoutCancellingHold)
{
    MdNodeTestAccess::hold(*node);
    for (size_t field = 0; field < 3; ++field)
    {
        auto command = motionCommand();
        auto* values = field == 0 ? &command.target_position : field == 1 ? &command.target_velocity : &command.target_torque;
        (*values)[0] = std::numeric_limits<float>::quiet_NaN();
        MdNodeTestAccess::motion(*node, command);
        EXPECT_EQ(bus->writes, 0u);
        EXPECT_TRUE(MdNodeTestAccess::holding(*node));
    }
    auto malformed = motionCommand();
    malformed.target_torque.clear();
    MdNodeTestAccess::motion(*node, malformed);
    EXPECT_EQ(bus->writes, 0u);
    EXPECT_TRUE(MdNodeTestAccess::holding(*node));
}

TEST_F(MdNodeCallbacks, OutOfTravelTargetPreservesActiveHold)
{
    MdNodeTestAccess::hold(*node);
    auto response = MdNodeTestAccess::target(*node, 1000.0);
    ASSERT_EQ(response->success.size(), 1u);
    EXPECT_FALSE(response->success[0]);
    EXPECT_TRUE(MdNodeTestAccess::holding(*node));
    EXPECT_EQ(bus->writes, 0u);
}

TEST_F(MdNodeCallbacks, OptionalTravelGuardAppliesToDirectTargetsAndMotionCommands)
{
    for (bool enabled : {true, false})
    {
        recreateWithTravelLimits(enabled);
        for (double position : {-.105768, 1.2})
        {
            const auto response = MdNodeTestAccess::target(*node, position);
            ASSERT_EQ(response->success.size(), 1u);
            EXPECT_EQ(response->success[0], !enabled);
            if (!enabled) EXPECT_FLOAT_EQ(bus->get(bus->schema.targetPosition), position);
            auto command = motionCommand();
            command.target_position = {static_cast<float>(position)};
            const auto writes = bus->writes;
            MdNodeTestAccess::motion(*node, command);
            EXPECT_EQ(bus->writes > writes, !enabled);
            EXPECT_TRUE(MdNodeTestAccess::acceptsMotion(*node));
        }
    }
}

TEST_F(MdNodeCallbacks, DisablingTravelGuardStillRejectsNonfiniteAndUnrepresentableTargets)
{
    recreateWithTravelLimits(false);
    MdNodeTestAccess::hold(*node);
    const auto writes = bus->writes;
    for (double position : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::max()})
    {
        const auto response = MdNodeTestAccess::target(*node, position);
        ASSERT_EQ(response->success.size(), 1u);
        EXPECT_FALSE(response->success[0]);
        auto command = motionCommand();
        command.target_position = {static_cast<float>(position)};
        MdNodeTestAccess::motion(*node, command);
        EXPECT_TRUE(MdNodeTestAccess::holding(*node));
        EXPECT_EQ(bus->writes, writes);
    }
}

TEST_F(MdNodeCallbacks, OptionalTravelGuardAppliesToMeasuredFeedback)
{
    for (bool enabled : {true, false})
    {
        recreateWithTravelLimits(enabled);
        bus->set(bus->schema.mainEncoderPosition, -.105768f);
        MdNodeTestAccess::feedback(*node);
        EXPECT_EQ(MdNodeTestAccess::inhibited(*node), enabled);
        EXPECT_EQ(MdNodeTestAccess::diagnosticHistory(*node).latest.travelLimitsEnabled, enabled);
    }
    bus->set(bus->schema.mainEncoderPosition, std::numeric_limits<float>::quiet_NaN());
    MdNodeTestAccess::feedback(*node);
    EXPECT_FALSE(MdNodeTestAccess::acceptsMotion(*node));
}

TEST_F(MdNodeCallbacks, InvalidPidImpedanceAndLimitsNeverWriteRegisters)
{
    candle_ros2::msg::PositionPidCmd pid;
    pid.device_ids = {113};
    pid.position_pid.resize(1);
    pid.position_pid[0].kp = std::numeric_limits<float>::quiet_NaN();
    pid.position_pid[0].max_output = 2.0f;
    MdNodeTestAccess::pid(*node, pid);
    candle_ros2::msg::ImpedanceCmd impedance;
    impedance.device_ids = {113};
    impedance.kp = {-1.0f};
    impedance.kd = {0.2f};
    impedance.max_output = {2.0f};
    MdNodeTestAccess::impedance(*node, impedance);
    for (auto response : {MdNodeTestAccess::limits(*node, {}, {2.0f}),
                          MdNodeTestAccess::limits(*node, {2.0f}, {-1.0f}),
                          MdNodeTestAccess::limits(*node, {std::numeric_limits<float>::infinity()}, {2.0f})})
    {
        ASSERT_EQ(response->success.size(), 1u);
        EXPECT_FALSE(response->success[0]);
    }
    EXPECT_EQ(bus->writes, 0u);
}

TEST_F(MdNodeCallbacks, ExplicitDisableSurvivesLaterCommunicationLoss)
{
    MdNodeTestAccess::hold(*node);
    MdNodeTestAccess::disableThenLoseCommunication(*node);
    EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
    EXPECT_TRUE(MdNodeTestAccess::needsHome(*node));
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::holding(*node));
    EXPECT_FALSE(MdNodeTestAccess::acceptsMotion(*node));
    EXPECT_EQ(bus->state(), 64u);
}

TEST_F(MdNodeCallbacks, FailedZeroAcknowledgementFaultsReferenceAndKeepsDriveDisabled)
{
    bus->rejectZero = true;
    auto response = MdNodeTestAccess::zero(*node);
    ASSERT_EQ(response->success.size(), 1u);
    EXPECT_FALSE(response->success[0]);
    EXPECT_EQ(MdNodeTestAccess::reference(*node), PositionTracker::State::Faulted);
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_TRUE(MdNodeTestAccess::needsHome(*node));
    EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
    EXPECT_FALSE(MdNodeTestAccess::acceptsMotion(*node));
    EXPECT_EQ(bus->state(), 64u);
}

TEST_F(MdNodeCallbacks, HomingReleaseDisableReadFailureLatchesAgainstLaterRecovery)
{
    expectFailedHomingReleaseStaysStopped(true);
}

TEST_F(MdNodeCallbacks, HomingReleaseDisableTimeoutLatchesAgainstLaterRecovery)
{
    expectFailedHomingReleaseStaysStopped(false);
}

TEST_F(MdNodeCallbacks, NormalOpeningUsesFastPositionRampWithoutHomingOrZeroing)
{
    MdNodeTestAccess::hold(*node);
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    bus->set(bus->schema.motorImpPidKp, 30.0f);
    bus->set(bus->schema.motorImpPidKd, .2f);
    bus->set(bus->schema.targetTorque, .25f);
    const auto response = MdNodeTestAccess::open(*node);
    ASSERT_EQ(response->success.size(), 1u);
    ASSERT_TRUE(response->success[0]);
    EXPECT_TRUE(MdNodeTestAccess::opening(*node));
    EXPECT_FALSE(MdNodeTestAccess::holding(*node));
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_FALSE(MdNodeTestAccess::needsHome(*node));
    EXPECT_EQ(bus->stateWrites, 0u);
    EXPECT_EQ(bus->modeWrites, 0u);
    EXPECT_EQ(bus->zeroWrites, 0u);
    EXPECT_EQ(bus->state(), 39u);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKp), 30.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKd), .2f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.targetPosition), .25f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.targetVelocity), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.targetTorque), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.maxTorque), 2.0f);
    const auto& control = MdNodeTestAccess::motionControl(*node);
    EXPECT_DOUBLE_EQ(control.speed, 10.0);
    EXPECT_DOUBLE_EQ(control.config.openLead, .14);
    EXPECT_DOUBLE_EQ(control.closed, 0.0);
    EXPECT_DOUBLE_EQ(control.integralTorque, 0.0);
    EXPECT_DOUBLE_EQ(MdNodeTestAccess::calibration(*node).closedPosition, .75);
    EXPECT_DOUBLE_EQ(MdNodeTestAccess::calibration(*node).openPosition, 0.0);

    MdNodeTestAccess::tickMotion(*node, .25, 0.0, 0.0, 1);
    ASSERT_EQ(control.state, ImpedanceClose::State::Closing);
    EXPECT_NEAR(bus->get(bus->schema.targetPosition),
                .25 - std::min(10.0 * control.elapsed, .14), 1e-7);
    EXPECT_DOUBLE_EQ(control.integralTorque, 0.0);
    EXPECT_EQ(bus->stateWrites, 0u);
    EXPECT_EQ(bus->modeWrites, 0u);
    EXPECT_EQ(bus->zeroWrites, 0u);
    ASSERT_FALSE(bus->torqueCommands.empty());
    for (float torque : bus->torqueCommands) EXPECT_FLOAT_EQ(torque, 0.0f);
}

TEST_F(MdNodeCallbacks, PeriodicOpeningVerificationStopsOverwrittenGainBeforeReassertingIt)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    const auto commandCount = bus->torqueCommands.size();
    bus->set(bus->schema.motorImpPidKp, 0.0f);
    MdNodeTestAccess::expireMotionVerification(*node);
    MdNodeTestAccess::tickMotion(*node, .24, -.1, -.1);
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_EQ(bus->state(), 64u);
    EXPECT_EQ(bus->torqueCommands.size(), commandCount);
    EXPECT_EQ(bus->zeroWrites, 0u);
    MdNodeTestAccess::loseCommunication(*node);
    EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
}

TEST_F(MdNodeCallbacks, OpeningReadbackFromWrongCanIdCannotContinueTheRamp)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    bus->set(bus->schema.canID, static_cast<u32>(447));
    MdNodeTestAccess::expireMotionVerification(*node);
    MdNodeTestAccess::tickMotion(*node, .24, -.1, -.1);
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_EQ(bus->state(), 64u);
    EXPECT_EQ(bus->zeroWrites, 0u);
}

TEST_F(MdNodeCallbacks, UncalibratedOpeningIsRejectedWithoutStartingHoming)
{
    MdNodeTestAccess::clearDiagnosticCalibration(*node);
    const auto response = MdNodeTestAccess::open(*node);
    ASSERT_EQ(response->success.size(), 1u);
    EXPECT_FALSE(response->success[0]);
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_TRUE(MdNodeTestAccess::needsHome(*node));
    EXPECT_EQ(bus->writes, 0u);
}

TEST_F(MdNodeCallbacks, AlreadyOpenReleasesAtDeadlineWithoutSearchOrDisableEnableCycle)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    bus->set(bus->schema.mainEncoderPosition, 0.0f);
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    for (int i = 0; i < 55; ++i)
        MdNodeTestAccess::tickMotion(*node, 0.0);
    ASSERT_TRUE(MdNodeTestAccess::opening(*node));
    ASSERT_EQ(MdNodeTestAccess::motionControl(*node).state, ImpedanceClose::State::TorqueHold);
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_FALSE(MdNodeTestAccess::inhibited(*node));
    EXPECT_EQ(MdNodeTestAccess::reference(*node), PositionTracker::State::Tracking);
    EXPECT_EQ(bus->stateWrites, 0u);
    EXPECT_EQ(bus->modeWrites, 0u);
    EXPECT_EQ(bus->zeroWrites, 0u);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKp), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKd), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.targetTorque), 0.0f);
    for (float torque : bus->torqueCommands) EXPECT_FLOAT_EQ(torque, 0.0f);

    const auto reads = bus->reads;
    MdNodeTestAccess::expireMotionVerification(*node);
    MdNodeTestAccess::tickMotion(*node, .06, .2);
    EXPECT_EQ(MdNodeTestAccess::motionControl(*node).state, ImpedanceClose::State::TorqueHold);
    EXPECT_EQ(bus->reads, reads); // Passive release has no command-verification loop.
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKp), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.motorImpPidKd), 0.0f);
    EXPECT_FLOAT_EQ(bus->get(bus->schema.targetTorque), 0.0f);
    EXPECT_DOUBLE_EQ(MdNodeTestAccess::calibration(*node).closedPosition, .75);
    EXPECT_DOUBLE_EQ(MdNodeTestAccess::calibration(*node).openPosition, 0.0);
}

TEST_F(MdNodeCallbacks, OpeningStillStopsOnFreshWrongMode)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    MdNodeTestAccess::setFeedbackMode(*node, mab::MdMode_E::IDLE);
    MdNodeTestAccess::tickMotion(*node, .24);
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_EQ(bus->state(), 64u);
    EXPECT_EQ(bus->zeroWrites, 0u);
    MdNodeTestAccess::loseCommunication(*node);
    EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
}

TEST_F(MdNodeCallbacks, OpeningRebootInvalidatesReferenceEvenWhenCommandVerificationIsDue)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    ASSERT_TRUE(MdNodeTestAccess::calibrated(*node));
    MdNodeTestAccess::expireMotionVerification(*node);

    // A power cycle first appears in the fresh feedback frame as disabled/IDLE.
    // It invalidates zero before command readback can classify it as a mismatch.
    bus->set(bus->schema.state, static_cast<u16>(64));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IDLE));
    MdNodeTestAccess::setFeedbackState(*node, 64);
    MdNodeTestAccess::setFeedbackMode(*node, mab::MdMode_E::IDLE);
    const auto reads = bus->reads;
    MdNodeTestAccess::tickMotion(*node, .24);
    EXPECT_EQ(bus->reads, reads);
    EXPECT_FALSE(MdNodeTestAccess::calibrated(*node));
    EXPECT_EQ(MdNodeTestAccess::reference(*node), PositionTracker::State::Recovering);
    EXPECT_TRUE(MdNodeTestAccess::needsHome(*node));
    EXPECT_TRUE(MdNodeTestAccess::autoHomePending(*node));
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::acceptsMotion(*node));
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_EQ(bus->zeroWrites, 0u);
}

TEST_F(MdNodeCallbacks, OpeningDiagnosticsCaptureObstructionDwellWithoutEndpointAcceptance)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    bus->set(bus->schema.mainEncoderPosition, .02f);
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    for (int i = 0; i < 10; ++i)
        MdNodeTestAccess::tickMotion(*node, .02, 0.0, -.4);
    const auto& control = MdNodeTestAccess::motionControl(*node);
    const auto obstructionDwell = control.contactTime;
    EXPECT_GT(obstructionDwell, .09);
    EXPECT_DOUBLE_EQ(control.endpointTime, 0.0);

    // Fault capture must preserve this evidence before erasing the active job.
    MdNodeTestAccess::recordAndStop(*node);
    const auto& history = MdNodeTestAccess::diagnosticHistory(*node);
    EXPECT_EQ(history.latest.operation, 4);
    EXPECT_DOUBLE_EQ(history.latest.settled, obstructionDwell);
    ASSERT_TRUE(history.pending.has_value());
    EXPECT_DOUBLE_EQ(history.pending->atStop.settled, obstructionDwell);
    EXPECT_DOUBLE_EQ(history.pending->settledAtCapture, obstructionDwell);
}

TEST_F(MdNodeCallbacks, OpeningStillStopsOnStaleFeedbackWithoutRehoming)
{
    bus->set(bus->schema.state, static_cast<u16>(39));
    bus->set(bus->schema.motionModeStatus, static_cast<u8>(mab::MdMode_E::IMPEDANCE));
    ASSERT_TRUE(MdNodeTestAccess::open(*node)->success[0]);
    MdNodeTestAccess::tickMotion(*node, .24, 0.0, 0.0, 300);
    EXPECT_TRUE(MdNodeTestAccess::inhibited(*node));
    EXPECT_FALSE(MdNodeTestAccess::opening(*node));
    EXPECT_FALSE(MdNodeTestAccess::homing(*node));
    EXPECT_EQ(bus->state(), 64u);
    EXPECT_EQ(bus->zeroWrites, 0u);
    MdNodeTestAccess::loseCommunication(*node);
    EXPECT_FALSE(MdNodeTestAccess::autoHomePending(*node));
}
}
