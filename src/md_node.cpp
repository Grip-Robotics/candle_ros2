#include "candle_ros2/md_node.hpp"
#include "candle_ros2/drive_state.hpp"
#include "candle_ros2/startup_reboot.hpp"
#include <unordered_set>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

MdNode::MdNode(const rclcpp::NodeOptions&   options,
               std::shared_ptr<mab::Candle> candle,
               const candleParams_S&        params)
    : Node("candle_md_node", options),
      m_candle(std::move(candle)),
      jointNamePrefix(params.joint_name_prefix),
      gripperOpenPositionRad(params.gripper_open_position_rad),
      gripperClosedPositionRad(params.gripper_closed_position_rad),
      gripperImpedanceKp(static_cast<float>(params.gripper_impedance_kp)),
      gripperImpedanceKd(static_cast<float>(params.gripper_impedance_kd)),
      gripperVelocityLimitRadS(static_cast<float>(params.gripper_velocity_limit_rad_s)),
      gripperOpenVelocityLimitRadS(static_cast<float>(params.gripper_open_velocity_limit_rad_s)),
      gripperTorqueLimitNm(static_cast<float>(params.gripper_torque_limit_nm)),
      softCloseSlowTorqueLimitNm(static_cast<float>(params.soft_close_slow_torque_limit_nm)),
      closeConfig(params.close),
      homeConfig(params.home),
      homingTravelMarginRad(params.homing_travel_margin_rad),
      homingRequired(params.homing_required),
      jointStatePublishPeriodMs(params.joint_state_publish_period_ms),
      healthPublishPeriodMs(params.health_publish_period_ms),
      mdCanResponseTimeout100us(params.md_can_response_timeout_100us),
      mdHostTimeoutMs(params.md_host_timeout_ms),
      encoderWrapPeriodRad(params.encoder_wrap_period_rad),
      positionRecoveryMaxDeltaRad(params.position_recovery_max_delta_rad),
      positionRecoverySamples(params.position_recovery_samples),
      positionRecoveryRetryMs(params.position_recovery_retry_ms),
      initDevicesZero(params.init_devices_zero)
{
    if (jointNamePrefix.empty())
        throw std::invalid_argument("joint_name_prefix must not be empty");
    if (!std::isfinite(gripperOpenPositionRad) || !std::isfinite(gripperClosedPositionRad) ||
        gripperOpenPositionRad == gripperClosedPositionRad ||
        !std::isfinite(gripperImpedanceKp) || gripperImpedanceKp < 0.0f ||
        !std::isfinite(gripperImpedanceKd) || gripperImpedanceKd < 0.0f ||
        !std::isfinite(gripperVelocityLimitRadS) || gripperVelocityLimitRadS <= 0.0f ||
        !std::isfinite(gripperOpenVelocityLimitRadS) || gripperOpenVelocityLimitRadS <= 0.0f ||
        !std::isfinite(gripperTorqueLimitNm) || gripperTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseSlowTorqueLimitNm) || softCloseSlowTorqueLimitNm <= 0.0f ||
        jointStatePublishPeriodMs <= 0 ||
        healthPublishPeriodMs <= 0 || mdCanResponseTimeout100us <= 0 ||
        mdCanResponseTimeout100us > std::numeric_limits<u8>::max() ||
        mdHostTimeoutMs <= (mdCanResponseTimeout100us + 9) / 10 ||
        !std::isfinite(encoderWrapPeriodRad) ||
        encoderWrapPeriodRad <= 0.0 || !std::isfinite(positionRecoveryMaxDeltaRad) ||
        positionRecoveryMaxDeltaRad <= 0.0 ||
        positionRecoveryMaxDeltaRad >= encoderWrapPeriodRad / 2.0 ||
        positionRecoverySamples <= 0 || positionRecoveryRetryMs <= 0)
        throw std::invalid_argument(
            "invalid gripper, telemetry, communication-timeout, or health parameter");

    // Validate the same settings used by every per-drive close controller.
    closeConfig.kp = gripperImpedanceKp;
    closeConfig.kd = gripperImpedanceKd;
    closeConfig.torqueLimit = std::min(gripperTorqueLimitNm, softCloseSlowTorqueLimitNm);
    ImpedanceClose(gripperOpenPositionRad, gripperClosedPositionRad,
                   gripperVelocityLimitRadS, closeConfig);
    ImpedanceHome(0.0, -1.0, homeConfig, closeConfig);
    if (!std::isfinite(homingTravelMarginRad) || homingTravelMarginRad < 0.0)
        throw std::invalid_argument("homing travel margin must be finite and non-negative");
    if (homingRequired)
        GripperHoming(0.0, gripperClosedPositionRad > gripperOpenPositionRad ? -1.0 : 1.0,
                      homeConfig, closeConfig, homingTravelMarginRad);
    if (homingRequired && (initDevicesZero || gripperOpenPositionRad != 0.0))
        throw std::invalid_argument("homing requires open target zero and init_devices_zero=false");
    if (!closeConfig.withinTravel(gripperOpenPositionRad) ||
        !closeConfig.withinTravel(gripperClosedPositionRad))
        throw std::invalid_argument("gripper targets must be within the absolute travel limits");
    if (gripperImpedanceKp <= 0.0f ||
        closeConfig.contactTorque >= std::min<double>(
            {gripperTorqueLimitNm, softCloseSlowTorqueLimitNm,
             gripperImpedanceKp * std::min(closeConfig.lead, closeConfig.openLead)}))
        throw std::invalid_argument("contact torque must be below the available bounded torque in both directions");

    rclcpp::QoS defaultQoS(10);
    defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        defaultQoS.best_effort();

    pubJointState = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(NODE_PREFIX) + "joint_states", defaultQoS);
    pubHealth = this->create_publisher<candle_ros2::msg::MdHealth>(
        std::string(NODE_PREFIX) + "health", defaultQoS);

    subMotionCmd = this->create_subscription<candle_ros2::msg::MotionCmd>(
        std::string(NODE_PREFIX) + "motion_command",
        10,
        std::bind(&MdNode::cbMotionCmd, this, std::placeholders::_1));
    subPositionCmd = this->create_subscription<candle_ros2::msg::PositionPidCmd>(
        std::string(NODE_PREFIX) + "position_command",
        10,
        std::bind(&MdNode::cbPositionCmd, this, std::placeholders::_1));
    subVelocityCmd = this->create_subscription<candle_ros2::msg::VelocityPidCmd>(
        std::string(NODE_PREFIX) + "velocity_command",
        10,
        std::bind(&MdNode::cbVelocityCmd, this, std::placeholders::_1));
    subImpedanceCmd = this->create_subscription<candle_ros2::msg::ImpedanceCmd>(
        std::string(NODE_PREFIX) + "impedance_command",
        10,
        std::bind(&MdNode::cbImpedanceCmd, this, std::placeholders::_1));

    srvAddMd = this->create_service<candle_ros2::srv::AddDevices>(
        std::string(NODE_PREFIX) + "add_mds",
        std::bind(&MdNode::cbAddMd, this, std::placeholders::_1, std::placeholders::_2));
    srvInitDevices = this->create_service<candle_ros2::srv::InitDevices>(
        std::string(NODE_PREFIX) + "init_devices",
        std::bind(&MdNode::cbInitDevices, this, std::placeholders::_1, std::placeholders::_2));
    srvStartupReboot = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "startup_reboot",
        std::bind(&MdNode::cbStartupReboot, this, std::placeholders::_1, std::placeholders::_2));
    srvZero = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "zero",
        std::bind(&MdNode::cbZero, this, std::placeholders::_1, std::placeholders::_2));
    srvHome = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "home",
        std::bind(&MdNode::cbHome, this, std::placeholders::_1, std::placeholders::_2));
    srvSetMode = this->create_service<candle_ros2::srv::SetMode>(
        std::string(NODE_PREFIX) + "set_mode",
        std::bind(&MdNode::cbSetMode, this, std::placeholders::_1, std::placeholders::_2));
    srvEnable = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "enable",
        std::bind(&MdNode::cbEnable, this, std::placeholders::_1, std::placeholders::_2));
    srvDisable = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "disable",
        std::bind(&MdNode::cbDisable, this, std::placeholders::_1, std::placeholders::_2));
    srvSetLimits = this->create_service<candle_ros2::srv::SetLimits>(
        std::string(NODE_PREFIX) + "set_limits",
        std::bind(&MdNode::cbSetLimits, this, std::placeholders::_1, std::placeholders::_2));
    srvOpen = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "open_gripper",
        std::bind(&MdNode::cbOpenGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvClose = this->create_service<candle_ros2::srv::Generic>(
        std::string(NODE_PREFIX) + "close_gripper",
        std::bind(&MdNode::cbCloseGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvConfigureGripper = this->create_service<candle_ros2::srv::ConfigureGripper>(
        std::string(NODE_PREFIX) + "configure_gripper",
        std::bind(&MdNode::cbConfigureGripper, this, std::placeholders::_1, std::placeholders::_2));
    srvSetGripperTargets = this->create_service<candle_ros2::srv::SetGripperTargets>(
        std::string(NODE_PREFIX) + "set_gripper_targets",
        std::bind(
            &MdNode::cbSetGripperTargets, this, std::placeholders::_1, std::placeholders::_2));
    srvSoftClose = this->create_service<candle_ros2::srv::SoftCloseGripper>(
        std::string(NODE_PREFIX) + "soft_close_gripper",
        std::bind(&MdNode::cbSoftCloseGripper, this, std::placeholders::_1, std::placeholders::_2));

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(jointStatePublishPeriodMs),
                                     std::bind(&MdNode::publishJointStates, this));
    tmrHealth = this->create_wall_timer(std::chrono::milliseconds(healthPublishPeriodMs),
                                        std::bind(&MdNode::publishHealth, this));

    RCLCPP_INFO(this->get_logger(),
                "Candle ROS2 MD node started: joint states every %d ms, "
                "CAN response timeout %.1f ms, host USB timeout %d ms.",
                jointStatePublishPeriodMs,
                mdCanResponseTimeout100us / 10.0,
                mdHostTimeoutMs);
}

MdNode::~MdNode()
{
    // The bus is still owned here. A normal exit must not leave a latched hold.
    // A disconnected bus/process crash still requires the MD hardware watchdog.
    m_softCloseJobs.clear();
    m_homingJobs.clear();
    m_homingGroup.reset();
    m_pendingDisables.clear();
    // Request every stop before waiting for any one motor to finish braking.
    for (auto& md : m_mds)
    {
        try
        {
            m_pendingDisables.try_emplace(md.m_canId, md);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(this->get_logger(), "Drive %d shutdown disable failed: %s",
                         md.m_canId, error.what());
        }
    }
    while (!m_pendingDisables.empty())
    {
        for (auto it = m_pendingDisables.begin(); it != m_pendingDisables.end();)
        {
            const auto id = it->first;
            auto md = findMd(m_mds, id);
            auto status = candle_ros2::DisableTransition::Status::Failed;
            try { status = it->second.poll(*md); }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(this->get_logger(), "Drive %d shutdown read failed: %s", id, error.what());
            }
            if (status == candle_ros2::DisableTransition::Status::Pending)
            {
                ++it;
                continue;
            }
            if (status == candle_ros2::DisableTransition::Status::Disabled)
                RCLCPP_INFO(this->get_logger(), "Drive %d disabled on shutdown", id);
            else
                RCLCPP_ERROR(this->get_logger(), "Drive %d shutdown disable unconfirmed", id);
            it = m_pendingDisables.erase(it);
        }
        if (!m_pendingDisables.empty())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Pure logging only: initialization can exit before the next health tick.
    flushDiagnosticFaults();
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::publishJointStates()
{
    tickDisableJobs();
    tickRecoveryJobs();
    if (!m_homingGroup)
        for (auto& md : m_mds)
            if (m_autoHoming.pending(md.m_canId) &&
                !m_recoveryContexts.contains(md.m_canId) &&
                !m_pendingDisables.contains(md.m_canId))
            {
                m_autoHoming.take(md.m_canId); // One attempt; failed homing never retries itself.
                startHoming(md);
                break;
            }

    sensor_msgs::msg::JointState msgJointStates;

    msgJointStates.name.reserve(m_mds.size());
    msgJointStates.position.reserve(m_mds.size());
    msgJointStates.velocity.reserve(m_mds.size());
    msgJointStates.effort.reserve(m_mds.size());

    msgJointStates.header.stamp = this->get_clock()->now();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (auto& md : m_mds)
    {
        msgJointStates.name.push_back(jointNamePrefix + std::to_string(md.m_canId));

        // Skip the bus entirely while the drive is recovering or faulted; the
        // NaN entries tell downstream consumers the reading is invalid.
        auto tracker = m_positionTrackers.find(md.m_canId);
        if (tracker == m_positionTrackers.end() ||
            tracker->second.state() == PositionTracker::State::Recovering ||
            tracker->second.state() == PositionTracker::State::Faulted)
        {
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        // One batched register read per drive is a third of the bus and USB
        // load of the per-register getters, and a failed read must never be
        // published as a measurement (the getters return a literal 0.0 on
        // error, which downstream consumers cannot tell from a real reading).
        if (md.readRegisters(md.m_mdRegisters.mainEncoderPosition,
                             md.m_mdRegisters.mainEncoderVelocity,
                             md.m_mdRegisters.motorTorque,
                             md.m_mdRegisters.state,
                             md.m_mdRegisters.motionModeStatus) != mab::MD::Error_t::OK)
        {
            recordDiagnosticFeedback(md, false, nan);
            beginRecovery(md.m_canId);
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        if (!std::isfinite(md.m_mdRegisters.mainEncoderPosition.value) ||
            !std::isfinite(md.m_mdRegisters.mainEncoderVelocity.value) ||
            !std::isfinite(md.m_mdRegisters.motorTorque.value))
        {
            recordDiagnosticFeedback(md, false, nan);
            beginRecovery(md.m_canId);
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        // Reuse this fresh batched reading for control and telemetry.
        const auto logicalPosition = tracker->second.observe(
            static_cast<double>(md.m_mdRegisters.mainEncoderPosition.value));
        if (!logicalPosition.has_value())
        {
            recordDiagnosticFeedback(md, false, nan);
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        recordDiagnosticFeedback(md, true, *logicalPosition);

        if (!isHoming(md.m_canId) && !m_motionInhibited.contains(md.m_canId) &&
            !withinCalibratedTravel(md.m_canId, *logicalPosition))
            stopDrive(md, "measured position outside calibrated travel; explicit recovery required");
        else if (isHoming(md.m_canId))
            tickHoming(md, *logicalPosition, md.m_mdRegisters.mainEncoderVelocity.value,
                       md.m_mdRegisters.motorTorque.value);
        else
            tickSoftClose(md, *logicalPosition, md.m_mdRegisters.mainEncoderVelocity.value,
                          md.m_mdRegisters.motorTorque.value);
        // Homing can change the reference on this tick; don't publish the old frame.
        msgJointStates.position.push_back(tracker->second.continuousPosition());
        msgJointStates.velocity.push_back(md.m_mdRegisters.mainEncoderVelocity.value);
        msgJointStates.effort.push_back(md.m_mdRegisters.motorTorque.value);
    }
    tickHomingGroup();
    // Group barriers can zero several trackers after the feedback loop above.
    // Never publish samples in the old frame after changing the reference.
    for (size_t i = 0; i < m_mds.size(); ++i)
        if (std::isfinite(msgJointStates.position[i]))
            msgJointStates.position[i] = m_positionTrackers.at(m_mds[i].m_canId).continuousPosition();
    this->pubJointState->publish(msgJointStates);
    return;
}

namespace
{
/// Appends every raised flag of one detailed status map as "<category>: <flag name>".
/// Returns true when at least one raised flag is a real error (not a warning).
template <class BitsT>
bool appendActiveFlags(const std::unordered_map<BitsT, mab::MDStatus::StatusItem_S>& status,
                       const char*  category,
                       std::string& description)
{
    bool anyError = false;
    for (const auto& [bit, item] : status)
    {
        if (!item.isSet())
            continue;
        if (!description.empty())
            description += "; ";
        description += std::string(category) + ": " + item.name;
        anyError = anyError || item.isError;
    }
    return anyError;
}
}  // namespace

void MdNode::publishHealth()
{
    candle_ros2::msg::MdHealth msg;
    msg.header.stamp = this->get_clock()->now();

    msg.device_ids.reserve(m_mds.size());
    msg.responsive.reserve(m_mds.size());
    msg.error.reserve(m_mds.size());
    msg.active_errors.reserve(m_mds.size());
    msg.motor_temperature_c.reserve(m_mds.size());
    msg.mosfet_temperature_c.reserve(m_mds.size());
    msg.motor_shutdown_temperature_c.reserve(m_mds.size());

    for (auto& md : m_mds)
    {
        msg.device_ids.push_back(md.m_canId);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        msg.motor_temperature_c.push_back(nan);
        msg.mosfet_temperature_c.push_back(nan);
        msg.motor_shutdown_temperature_c.push_back(nan);

        const auto trackerIt = m_positionTrackers.find(md.m_canId);
        const bool positionLost =
            trackerIt != m_positionTrackers.end() &&
            trackerIt->second.state() == PositionTracker::State::Faulted;
        const bool positionRecovering =
            !positionLost &&
            (m_recoveryContexts.find(md.m_canId) != m_recoveryContexts.end() ||
             (trackerIt != m_positionTrackers.end() &&
              trackerIt->second.state() == PositionTracker::State::Recovering));

        const auto [quickStatus, statusErr] = md.getQuickStatus();
        if (statusErr != mab::MD::Error_t::OK)
        {
            if (!m_motionInhibited.contains(md.m_canId) || m_softCloseJobs.contains(md.m_canId) ||
                isHoming(md.m_canId))
                beginRecovery(md.m_canId);
            msg.responsive.push_back(false);
            msg.error.push_back(true);
            msg.active_errors.emplace_back(
                positionLost ? "drive did not answer the status query; position: reference "
                               "lost, manual /md/zero calibration required"
                             : "drive did not answer the status query");
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "Health: failed to read quick status for drive %d",
                                 md.m_canId);
            continue;
        }

        using Bits = mab::MDStatus::QuickStatusBits;
        bool        anyError = false;
        std::string description;

        // Position-tracker state matters to the stack: while it is not
        // Tracking, the node rejects every motion command for this drive.
        if (positionLost)
        {
            description = "position: reference lost, manual /md/zero calibration required";
            anyError    = true;
        }
        else if (positionRecovering)
        {
            description = "position: recovering after communication loss, commands rejected";
            anyError    = true;
        }

        const auto commandError = m_commandErrors.find(md.m_canId);
        if (commandError != m_commandErrors.end())
        {
            if (!description.empty())
                description += "; ";
            description += commandError->second;
            anyError = true;
        }
        if (m_pendingDisables.contains(md.m_canId) && !releasingDisabled(md.m_canId))
        {
            if (!description.empty())
                description += "; ";
            description += "hardware disable pending";
            anyError = true;
        }

        // One read-only batch provides thermal telemetry; never change limits
        // or clear faults as part of a health query.
        mab::MDRegisters_S temperatures;
        if (md.readRegisters(temperatures.motorTemperature, temperatures.mosfetTemperature,
                             temperatures.motorShutdownTemp) == mab::MD::Error_t::OK &&
            candle_ros2::healthyTemperatures(temperatures.motorTemperature.value,
                temperatures.mosfetTemperature.value, temperatures.motorShutdownTemp.value))
        {
            msg.motor_temperature_c.back() = temperatures.motorTemperature.value;
            msg.mosfet_temperature_c.back() = temperatures.mosfetTemperature.value;
            msg.motor_shutdown_temperature_c.back() = temperatures.motorShutdownTemp.value;
        }
        else
        {
            if (!description.empty())
                description += "; ";
            description += "temperature: invalid, unavailable, or above cutoff";
            anyError = true;
        }

        // Detailed subsystem status is fetched only for flagged categories.
        const auto collect = [&](Bits bit, const char* category, auto&& readDetailedStatus)
        {
            if (!quickStatus.at(bit).isSet())
                return;
            const auto [detailedStatus, err] = readDetailedStatus();
            if (err != mab::MD::Error_t::OK)
            {
                if (!description.empty())
                    description += "; ";
                description += std::string(category) + ": status unavailable";
                anyError = true;
                return;
            }
            anyError = appendActiveFlags(detailedStatus, category, description) || anyError;
        };

        collect(Bits::MainEncoderStatus,
                "main encoder",
                [&md] { return md.getMainEncoderStatus(); });
        collect(Bits::OutputEncoderStatus,
                "output encoder",
                [&md] { return md.getOutputEncoderStatus(); });
        collect(Bits::CalibrationEncoderStatus,
                "calibration",
                [&md] { return md.getCalibrationStatus(); });
        collect(Bits::MosfetBridgeStatus,
                "mosfet bridge",
                [&md] { return md.getBridgeStatus(); });
        collect(Bits::HardwareStatus, "hardware", [&md] { return md.getHardwareStatus(); });
        collect(Bits::CommunicationStatus,
                "communication",
                [&md] { return md.getCommunicationStatus(); });
        collect(Bits::MotionStatus, "motion", [&md] { return md.getMotionStatus(); });

        if (anyError)
        {
            if (!m_motionInhibited.contains(md.m_canId) || m_softCloseJobs.contains(md.m_canId) ||
                isHoming(md.m_canId))
                stopDrive(md, "gripper motion aborted: " + description);
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "Health: drive %d reports errors: %s",
                                 md.m_canId,
                                 description.c_str());
        }

        // Reference readiness is reported as unhealthy, but is not a physical
        // fault that should abort the homing which establishes that reference.
        if (m_homingRequired.contains(md.m_canId))
        {
            if (!description.empty()) description += "; ";
            description += isHoming(md.m_canId) ? "position: homing" : "position: homing required";
            anyError = true;
        }
        msg.responsive.push_back(true);
        msg.error.push_back(anyError);
        msg.active_errors.push_back(std::move(description));
    }

    this->pubHealth->publish(msg);
    publishDiagnostics();
}

void MdNode::cbMotionCmd(const candle_ros2::msg::MotionCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.target_position.size() || n != msg.target_velocity.size() ||
        n != msg.target_torque.size())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Motion Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        if (!candle_ros2::validDeviceId(msg.device_ids[i]))
        {
            RCLCPP_WARN(get_logger(), "Invalid motion device ID %u rejected before narrowing", msg.device_ids[i]);
            continue;
        }
        const auto id = static_cast<u16>(msg.device_ids[i]);
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }
        if (!canAcceptPositionCommand(msg.device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; motion command rejected",
                        msg.device_ids[i]);
            continue;
        }

        if (!withinCalibratedTravel(msg.device_ids[i], msg.target_position[i]) ||
            !candle_ros2::finiteRegisterValue(m_positionTrackers.at(id).logicalToRaw(msg.target_position[i])) ||
            !std::isfinite(msg.target_velocity[i]) || !std::isfinite(msg.target_torque[i]) ||
            !cancelSoftClose(msg.device_ids[i]))
            continue;
        mab::MDRegisters_S mdRegisters;
        mdRegisters.targetPosition =
            m_positionTrackers.at(msg.device_ids[i]).logicalToRaw(msg.target_position[i]);
        mdRegisters.targetVelocity = msg.target_velocity[i];
        mdRegisters.targetTorque   = msg.target_torque[i];

        if (md->writeRegisters(mdRegisters.targetPosition,
                               mdRegisters.targetVelocity,
                               mdRegisters.targetTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Motion Command for drive with ID: %d",
                        msg.device_ids[i]);
            beginRecovery(msg.device_ids[i]);
        }
    }
    return;
}

void MdNode::cbPositionCmd(const candle_ros2::msg::PositionPidCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.position_pid.size() || (!msg.velocity_pid.empty() && n != msg.velocity_pid.size()))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Position Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        if (isHoming(msg.device_ids[i]) || m_softCloseJobs.contains(msg.device_ids[i]) || m_pendingDisables.contains(msg.device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(), "Drive %d is closing/holding; configuration rejected",
                        msg.device_ids[i]);
            continue;
        }

        if (!candle_ros2::validPid(msg.position_pid[i]) ||
            (!msg.velocity_pid.empty() && !candle_ros2::validPid(msg.velocity_pid[i])))
        {
            RCLCPP_WARN(get_logger(), "Invalid PID configuration rejected for drive %d", msg.device_ids[i]);
            continue;
        }
        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorPosPidKp     = msg.position_pid[i].kp;
        mdRegisters.motorPosPidKi     = msg.position_pid[i].ki;
        mdRegisters.motorPosPidKd     = msg.position_pid[i].kd;
        mdRegisters.motorPosPidWindup = msg.position_pid[i].i_windup;
        mdRegisters.profileVelocity   = msg.position_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorPosPidKp,
                               mdRegisters.motorPosPidKi,
                               mdRegisters.motorPosPidKd,
                               mdRegisters.motorPosPidWindup,
                               mdRegisters.profileVelocity) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Position PID parameters for drive with ID: %d",
                        msg.device_ids[i]);
            stopDrive(*md, "position PID write failed; explicit enable required");
            continue;
        }

        if (i < (size_t)msg.velocity_pid.size())
        {
            mdRegisters.motorVelPidKp     = msg.velocity_pid[i].kp;
            mdRegisters.motorVelPidKi     = msg.velocity_pid[i].ki;
            mdRegisters.motorVelPidKd     = msg.velocity_pid[i].kd;
            mdRegisters.motorVelPidWindup = msg.velocity_pid[i].i_windup;
            mdRegisters.maxTorque         = msg.velocity_pid[i].max_output;
            if (md->writeRegisters(mdRegisters.motorVelPidKp,
                                   mdRegisters.motorVelPidKi,
                                   mdRegisters.motorVelPidKd,
                                   mdRegisters.motorVelPidWindup,
                                   mdRegisters.maxTorque) != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to set Velocity PID parameters for drive with ID: %d",
                            msg.device_ids[i]);
                stopDrive(*md, "velocity PID write failed; explicit enable required");
            }
        }
    }
    return;
}

void MdNode::cbVelocityCmd(const candle_ros2::msg::VelocityPidCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.velocity_pid.size())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Velocity Command message incomplete. Sizes of arrays do not match! Ignoring message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        if (isHoming(msg.device_ids[i]) || m_softCloseJobs.contains(msg.device_ids[i]) || m_pendingDisables.contains(msg.device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(), "Drive %d is closing/holding; configuration rejected",
                        msg.device_ids[i]);
            continue;
        }

        if (!candle_ros2::validPid(msg.velocity_pid[i]))
        {
            RCLCPP_WARN(get_logger(), "Invalid velocity PID configuration rejected for drive %d", msg.device_ids[i]);
            continue;
        }
        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorVelPidKp     = msg.velocity_pid[i].kp;
        mdRegisters.motorVelPidKi     = msg.velocity_pid[i].ki;
        mdRegisters.motorVelPidKd     = msg.velocity_pid[i].kd;
        mdRegisters.motorVelPidWindup = msg.velocity_pid[i].i_windup;
        mdRegisters.maxTorque         = msg.velocity_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorVelPidKp,
                               mdRegisters.motorVelPidKi,
                               mdRegisters.motorVelPidKd,
                               mdRegisters.motorVelPidWindup,
                               mdRegisters.maxTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Velocity PID parameters for drive with ID: %d",
                        msg.device_ids[i]);
            stopDrive(*md, "velocity PID write failed; explicit enable required");
        }
    }
    return;
}

void MdNode::cbImpedanceCmd(const candle_ros2::msg::ImpedanceCmd& msg)
{
    size_t n = msg.device_ids.size();

    if (n != msg.kp.size() || n != msg.kd.size() || n != msg.max_output.size())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Impedance Command message incomplete. Sizes of arrays do not match! Ignoring "
                    "message.");
        return;
    }

    for (size_t i = 0; i < n; i++)
    {
        auto md = findMd(m_mds, msg.device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", msg.device_ids[i]);
            continue;
        }

        if (isHoming(msg.device_ids[i]) || m_softCloseJobs.contains(msg.device_ids[i]) || m_pendingDisables.contains(msg.device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(), "Drive %d is closing/holding; configuration rejected",
                        msg.device_ids[i]);
            continue;
        }

        if (!candle_ros2::nonnegativeRegisterValue(msg.kp[i]) ||
            !candle_ros2::nonnegativeRegisterValue(msg.kd[i]) ||
            !candle_ros2::positiveRegisterValue(msg.max_output[i]))
        {
            RCLCPP_WARN(get_logger(), "Invalid impedance configuration rejected for drive %d", msg.device_ids[i]);
            continue;
        }
        mab::MDRegisters_S mdRegisters;
        mdRegisters.motorImpPidKp = msg.kp[i];
        mdRegisters.motorImpPidKd = msg.kd[i];
        mdRegisters.maxTorque     = msg.max_output[i];
        if (md->writeRegisters(mdRegisters.motorImpPidKp,
                               mdRegisters.motorImpPidKd,
                               mdRegisters.maxTorque) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Impedance parameters for drive with ID: %d",
                        msg.device_ids[i]);
            stopDrive(*md, "impedance configuration write failed; explicit enable required");
        }
    }
    return;
}

void MdNode::cbStartupReboot(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                            std::shared_ptr<candle_ros2::srv::Generic::Response> rsp)
{
    rsp->success.assign(req->device_ids.size(), false);
    // Blocking boot waits are permitted only before there are controlled drives.
    // This node uses a single-threaded executor, so no other callback can add one
    // during the reboot. Never expose reboot as an in-motion recovery operation.
    if (!m_mds.empty() || m_startupRebootAttempted || req->device_ids.empty()) return;
    std::unordered_set<u16> unique;
    for (auto id : req->device_ids)
        if (!candle_ros2::validDeviceId(id) || !unique.insert(id).second) return;
    m_startupRebootAttempted = true;
    const auto sleep = [](auto duration) { std::this_thread::sleep_for(duration); };
    std::vector<mab::MD> drives;
    for (auto id : req->device_ids)
    {
        drives.emplace_back(id, m_candle.get());
        drives.back().m_timeout = static_cast<u32>(mdCanResponseTimeout100us);
    }
    bool prepared = true;
    // Stop all selected drives before waiting on any reboot. Try every selected
    // drive even if another cannot be reached; do not reset any on failed preflight.
    for (auto& md : drives)
    {
        auto& r = md.m_mdRegisters;
        const auto logRegister = [&](auto& reg, const char* name) {
            if (md.readRegister(reg) == mab::MD::Error_t::OK)
                RCLCPP_INFO(get_logger(), "Drive %d before startup reboot: %s=%u",
                            md.m_canId, name, static_cast<unsigned>(reg.value));
            else RCLCPP_WARN(get_logger(), "Drive %d before startup reboot: %s unreadable", md.m_canId, name);
        };
        logRegister(r.state, "state"); logRegister(r.quickStatus, "quick_status");
        logRegister(r.hardwareStatus, "hardware_status");
        logRegister(r.communicationStatus, "communication_status");
        logRegister(r.motionStatus, "motion_status");
        logRegister(r.bridgeStatus, "bridge_status");
        if (!candle_ros2::prepareStartupReboot(md, sleep))
        {
            prepared = false;
            RCLCPP_ERROR(get_logger(), "Drive %d startup reboot preflight failed", md.m_canId);
        }
    }
    if (!prepared) return;
    for (size_t i = 0; i < drives.size(); ++i)
    {
        RCLCPP_INFO(get_logger(), "Rebooting drive %d before homing", drives[i].m_canId);
        if (!candle_ros2::rebootStartupDrive(drives[i], sleep))
        {
            RCLCPP_ERROR(get_logger(), "Drive %d startup reboot failed verification; homing blocked", drives[i].m_canId);
            // Even an unexpected powered post-boot state must receive a stop.
            // These temporary drives are not covered by the node's destructor.
            for (auto& drive : drives)
            {
                candle_ros2::DisableTransition stop(drive);
                auto status = stop.poll(drive);
                while (status == candle_ros2::DisableTransition::Status::Pending)
                {
                    sleep(std::chrono::milliseconds(10));
                    status = stop.poll(drive);
                }
                if (status != candle_ros2::DisableTransition::Status::Disabled)
                    RCLCPP_ERROR(get_logger(), "Drive %d disable after failed startup reboot unconfirmed", drive.m_canId);
            }
            return;
        }
        rsp->success[i] = true;
        RCLCPP_INFO(get_logger(), "Drive %d reboot verified: disabled, idle, no quick fault, zero target torque, watchdog valid", drives[i].m_canId);
    }
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                     std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        if (!candle_ros2::validDeviceId(id))
        {
            rsp->success.push_back(false);
            continue;
        }
        if (findMd(m_mds, id) != m_mds.end())
        {
            rsp->success.push_back(true);
            continue;
        }

        mab::MD md(id, m_candle.get());
        // The CANdle firmware deadline and host USB deadline use
        // different units and must not be derived from the same integer.
        // A missing drive therefore blocks this single-threaded node only
        // for the bounded host timeout, not 10x the intended CAN window.
        md.m_timeout = static_cast<u32>(mdCanResponseTimeout100us);
        if (md.init() != mab::MD::Error_t::OK)
        {
            rsp->success.push_back(false);
            continue;
        }

        m_positionTrackers.emplace(
            id,
            PositionTracker(encoderWrapPeriodRad,
                            positionRecoveryMaxDeltaRad,
                            static_cast<std::size_t>(positionRecoverySamples)));
        m_mds.push_back(std::move(md));
        if (homingRequired) m_homingRequired.insert(id);
        rsp->success.push_back(true);
    }
    rsp->total_devices = static_cast<u16>(m_mds.size());
    return;
}

void MdNode::cbInitDevices(const std::shared_ptr<candle_ros2::srv::InitDevices::Request> req,
                           std::shared_ptr<candle_ros2::srv::InitDevices::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);

    auto addReq        = std::make_shared<candle_ros2::srv::AddDevices::Request>();
    auto addRsp        = std::make_shared<candle_ros2::srv::AddDevices::Response>();
    addReq->device_ids = req->device_ids;
    cbAddMd(addReq, addRsp);

    for (size_t i = 0; i < n; ++i)
    {
        const bool added = i < addRsp->success.size() && addRsp->success[i];
        if (!added)
            continue;

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
            continue;

        if (isHoming(md->m_canId) || m_homingRequired.contains(md->m_canId))
            continue;

        // Reinitialization must not allow a previously scheduled soft-close
        // stage to command this drive again after it is enabled.
        m_softCloseJobs.erase(req->device_ids[i]);
        m_motionInhibited.insert(req->device_ids[i]);

        // Disable first: MD motion mode resets on disable, and a previous
        // close/soft-close target may still be latched until overwritten in
        // the active motion mode.
        if (m_pendingDisables.contains(req->device_ids[i]) ||
            !resetDriveErrorsIfNeeded(*md) || !prepareDriveDisabled(*md))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Init devices: failed to prepare drive %d for safe configuration",
                        req->device_ids[i]);
            continue;
        }

        if (initDevicesZero && md->zero() != mab::MD::Error_t::OK)
        {
            m_gripperCalibrations.erase(md->m_canId);
            m_positionTrackers.at(md->m_canId).markFaulted();
            stopDrive(*md, "init zero acknowledgement failed; reference invalid");
            continue;
        }
        if (initDevicesZero)
        {
            const auto zero = candle_ros2::readZeroPosition(*md, closeConfig.tolerance);
            if (!zero)
            {
                m_positionTrackers.at(req->device_ids[i]).markFaulted();
                stopDrive(*md, "init zero readback failed; reference invalid");
                continue;
            }
            m_positionTrackers.at(req->device_ids[i]).resetAtZero(*zero);
        }

        const bool configured =
            req->mode != "IMPEDANCE" ||
            configureGripper(*md,
                             gripperImpedanceKp,
                             gripperImpedanceKd,
                             gripperVelocityLimitRadS,
                             gripperTorqueLimitNm);
        if (!configured)
            continue;

        // Mode must be selected before the hold target is written. Targets set
        // while the drive is IDLE after disable are not reliably used on enable,
        // so a stale closed target from soft-close can take effect otherwise.
        auto modeReq        = std::make_shared<candle_ros2::srv::SetMode::Request>();
        auto modeRsp        = std::make_shared<candle_ros2::srv::SetMode::Response>();
        modeReq->device_ids = {req->device_ids[i]};
        modeReq->mode       = {req->mode};
        cbSetMode(modeReq, modeRsp);
        if (modeRsp->success.empty() || !modeRsp->success.front())
            continue;

        const auto holdPos = readLogicalPosition(*md, false);
        if (!holdPos.has_value() || !setGripperTarget(*md, *holdPos))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Init devices: failed to latch hold target for drive %d",
                        req->device_ids[i]);
            continue;
        }

        if (!candle_ros2::enableDrive(*md, md->m_mdRegisters.motionModeCommand.value))
        {
            stopDrive(*md, "init enable/state verification failed; explicit enable required");
            continue;
        }

        // Reassert hold after enable so a briefly restored stale target cannot
        // start a close motion.
        const auto holdPosAfterEnable = readLogicalPosition(*md, false);
        if (!holdPosAfterEnable.has_value() ||
            !setGripperTarget(*md, *holdPosAfterEnable))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Init devices: failed to reassert hold target for drive %d after enable",
                        req->device_ids[i]);
            stopDrive(*md, "init hold failed; explicit enable required");
            continue;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Init devices: drive %d holding at %.4f rad",
                    req->device_ids[i],
                    *holdPosAfterEnable);
        m_motionInhibited.erase(req->device_ids[i]);
        m_autoHoming.activate(req->device_ids[i]);
        m_commandErrors.erase(req->device_ids[i]);
        rsp->success[i] = true;
    }
}

void MdNode::cbZero(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros2::srv::Generic::Response> rsp)
{
    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end() || isHoming(id) || m_pendingDisables.contains(id))
        {
            rsp->success.push_back(false);
            continue;
        }
        // Reference changes are never made against powered PD or a latched target.
        m_motionInhibited.insert(id);
        m_autoHoming.disable(id);
        m_softCloseJobs.erase(id);
        if (!prepareDriveDisabled(*md))
        {
            rsp->success.push_back(false);
            continue;
        }
        if (m_gripperCalibrations.erase(id) || homingRequired)
            m_homingRequired.insert(id);
        if (md->zero() != mab::MD::Error_t::OK)
        {
            m_positionTrackers.at(id).markFaulted();
            stopDrive(*md, "zero acknowledgement failed; reference invalid");
            rsp->success.push_back(false);
            continue;
        }
        const auto raw = candle_ros2::readZeroPosition(*md, closeConfig.tolerance);
        if (!raw)
        {
            m_positionTrackers.at(id).markFaulted();
            stopDrive(*md, "zero readback failed; reference invalid");
            rsp->success.push_back(false);
            continue;
        }
        m_recoveryContexts.erase(id);
        m_positionTrackers.at(id).resetAtZero(*raw);
        if (!writeRawTarget(*md, *raw))
        {
            stopDrive(*md, "zero target write failed; explicit enable required");
            rsp->success.push_back(false);
            continue;
        }
        m_commandErrors[id] = "zeroed while disabled; explicit enable or homing required";
        RCLCPP_INFO(get_logger(), "Drive %d zero verified while disabled; explicit enable or homing required", id);
        rsp->success.push_back(true);
    }
}

void MdNode::cbSetLimits(const std::shared_ptr<candle_ros2::srv::SetLimits::Request> req,
                         std::shared_ptr<candle_ros2::srv::SetLimits::Response>      rsp)
{
    if (req->device_ids.size() != req->velocity_limit.size() ||
        req->device_ids.size() != req->torque_limit.size())
    {
        rsp->success.assign(req->device_ids.size(), false);
        RCLCPP_WARN(this->get_logger(),
                    "SetLimits request incomplete. Sizes of arrays do not match!");
        return;
    }

    rsp->success.reserve(req->device_ids.size());
    for (size_t i = 0; i < req->device_ids.size(); i++)
    {
        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (isHoming(req->device_ids[i]) || m_softCloseJobs.contains(req->device_ids[i]) || m_pendingDisables.contains(req->device_ids[i]))
        {
            rsp->success.push_back(false);
            continue;
        }

        if (!candle_ros2::positiveRegisterValue(req->velocity_limit[i]) ||
            !candle_ros2::positiveRegisterValue(req->torque_limit[i]))
        {
            rsp->success.push_back(false);
            continue;
        }
        mab::MDRegisters_S mdRegisters;
        mdRegisters.profileVelocity = req->velocity_limit[i];
        mdRegisters.maxTorque       = req->torque_limit[i];
        if (md->writeRegisters(mdRegisters.profileVelocity, mdRegisters.maxTorque) ==
            mab::MD::Error_t::OK)
        {
            rsp->success.push_back(true);
        }
        else
        {
            rsp->success.push_back(false);
        }
    }
    return;
}

bool MdNode::configureGripper(
    mab::MD& md, double kp, double kd, double velocityLimit, double torqueLimit)
{
    if (!candle_ros2::nonnegativeRegisterValue(kp) || !candle_ros2::nonnegativeRegisterValue(kd) ||
        !candle_ros2::positiveRegisterValue(velocityLimit) || !candle_ros2::positiveRegisterValue(torqueLimit))
        return false;
    mab::MDRegisters_S impedanceRegs;
    impedanceRegs.motorImpPidKp = static_cast<float>(kp);
    impedanceRegs.motorImpPidKd = static_cast<float>(kd);
    impedanceRegs.maxTorque     = static_cast<float>(torqueLimit);
    if (md.writeRegisters(impedanceRegs.motorImpPidKp,
                          impedanceRegs.motorImpPidKd,
                          impedanceRegs.maxTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set impedance gains for drive with ID: %d", md.m_canId);
        return false;
    }

    if (!candle_ros2::configureGripperLimits(md, static_cast<float>(velocityLimit),
                                            static_cast<float>(torqueLimit)))
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set/verify limits for drive with ID: %d", md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::setGripperTarget(mab::MD& md, double targetPos)
{
    auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking() ||
        !withinCalibratedTravel(md.m_canId, targetPos))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d has no valid position reference; target rejected",
                    md.m_canId);
        return false;
    }

    return writeRawTarget(md, tracker->second.logicalToRaw(targetPos));
}

bool MdNode::setGripperCommand(mab::MD& md, const ImpedanceClose::Command& command)
{
    auto& diagnosticCommand = m_diagnostics[md.m_canId].lastCommand;
    diagnosticCommand.commandRosNs = get_clock()->now().nanoseconds();
    diagnosticCommand.desiredPosition = command.position;
    diagnosticCommand.desiredTorque = command.torque;
    diagnosticCommand.kp = command.kp;
    diagnosticCommand.kd = command.kd;
    diagnosticCommand.commandValid = false;
    const auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking() ||
        !withinCalibratedTravel(md.m_canId, command.position) ||
        !candle_ros2::finiteRegisterValue(tracker->second.logicalToRaw(command.position)) ||
        !candle_ros2::finiteRegisterValue(command.torque) ||
        !candle_ros2::finiteRegisterValue(command.kp) || !candle_ros2::finiteRegisterValue(command.kd) ||
        command.kp < 0.0 || command.kd < 0.0 ||
        std::abs(command.torque) > closeConfig.torqueLimit)
    {
        stopDrive(md, "invalid impedance command or position reference");
        return false;
    }
    // One frame switches PD+I to pure torque without doubling holding effort.
    mab::MDRegisters_S regs;
    regs.motorImpPidKp = command.kp;
    regs.motorImpPidKd = command.kd;
    regs.targetPosition = tracker->second.logicalToRaw(command.position);
    regs.targetVelocity = 0.0;
    regs.targetTorque = command.torque;
    if (md.writeRegisters(regs.motorImpPidKp, regs.motorImpPidKd,
                          regs.targetPosition, regs.targetVelocity, regs.targetTorque) !=
        mab::MD::Error_t::OK)
    {
        stopDrive(md, "impedance command write failed; explicit enable required");
        return false;
    }
    diagnosticCommand.commandValid = true;
    return true;
}

bool MdNode::verifyGripperCommand(mab::MD& md, const ImpedanceClose::Command& command, double expectedTorqueLimit)
{
    const auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end()) return false;
    mab::MDRegisters_S r;
    const auto same = [](double actual, double expected) {
        return std::isfinite(actual) && std::abs(actual - expected) <= 1e-5 * std::max(1.0, std::abs(expected));
    };
    // Keep each synchronous MD request <=32 bytes. Also verify the output cap
    // after enable, not just configuration RAM before the transition.
    const double torqueLimit = expectedTorqueLimit >= 0.0 ? expectedTorqueLimit :
        (isHoming(md.m_canId) ? homeConfig.torque :
         std::min(gripperTorqueLimitNm, softCloseSlowTorqueLimitNm));
    const bool readOk =
        md.readRegisters(r.canID, r.state, r.motionModeStatus, r.canWatchdog, r.maxTorque) == mab::MD::Error_t::OK &&
        md.readRegisters(r.motorImpPidKp, r.motorImpPidKd, r.targetPosition, r.targetVelocity, r.targetTorque) == mab::MD::Error_t::OK &&
        md.readRegister(r.maxVelocity) == mab::MD::Error_t::OK;
    const bool matches = readOk && r.canID.value == md.m_canId &&
        candle_ros2::driveEnabled(r.state.value) && r.motionModeStatus.value == mab::MdMode_E::IMPEDANCE &&
        r.canWatchdog.value > 0 && r.canWatchdog.value <= 250 &&
        same(r.maxTorque.value, torqueLimit) &&
        same(r.maxVelocity.value, candle_ros2::gripperFirmwareVelocityCeilingRadS) &&
        same(r.motorImpPidKp.value, command.kp) && same(r.motorImpPidKd.value, command.kd) &&
        same(r.targetPosition.value, tracker->second.logicalToRaw(command.position)) &&
        same(r.targetVelocity.value, 0.0) && same(r.targetTorque.value, command.torque);
    candle_ros2::DiagnosticJson evidence;
    evidence.string("event", "command_readback"); evidence.integer("schema", 1);
    evidence.integer("drive_id", md.m_canId); evidence.integer("ros_ns", get_clock()->now().nanoseconds());
    evidence.boolean("read_ok", readOk); evidence.boolean("matches", matches);
    evidence.number("expected_torque_nm", command.torque);
    evidence.number("expected_kp", command.kp); evidence.number("expected_kd", command.kd);
    evidence.number("expected_raw_position_rad", tracker->second.logicalToRaw(command.position));
    evidence.number("expected_max_torque_nm", torqueLimit);
    if (readOk)
    {
        evidence.integer("can_id", r.canID.value); evidence.integer("state", r.state.value);
        evidence.integer("mode", r.motionModeStatus.value); evidence.integer("watchdog_ms", r.canWatchdog.value);
        evidence.number("kp", r.motorImpPidKp.value); evidence.number("kd", r.motorImpPidKd.value);
        evidence.number("target_raw_position_rad", r.targetPosition.value);
        evidence.number("target_velocity_rad_s", r.targetVelocity.value);
        evidence.number("target_torque_nm", r.targetTorque.value);
        evidence.number("max_torque_nm", r.maxTorque.value); evidence.number("max_velocity_rad_s", r.maxVelocity.value);
    }
    // During startup these readbacks are otherwise completely absent. Normal
    // motion logs only mismatches to keep production log volume bounded.
    if (isHoming(md.m_canId) || !matches)
        RCLCPP_INFO(get_logger(), "CANDLE_DIAG %s", evidence.str().c_str());
    if (!matches)
    {
        stopDrive(md, "impedance command/state readback mismatch; explicit enable required");
        return false;
    }
    return true;
}

bool MdNode::writeRawTarget(mab::MD& md, double rawTargetPos)
{
    if (!candle_ros2::finiteRegisterValue(rawTargetPos)) return false;
    mab::MDRegisters_S motionRegs;
    motionRegs.targetPosition = rawTargetPos;
    motionRegs.targetVelocity = 0.0;
    motionRegs.targetTorque   = 0.0;
    if (md.writeRegisters(motionRegs.targetPosition,
                          motionRegs.targetVelocity,
                          motionRegs.targetTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set target position for drive with ID: %d", md.m_canId);
        beginRecovery(md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::moveGripper(mab::MD& md, double targetPos)
{
    if (!prepareDriveDisabled(md))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to disable drive with ID: %d before impedance move",
                    md.m_canId);
        return false;
    }

    if (!configureGripper(md,
                          gripperImpedanceKp,
                          gripperImpedanceKd,
                          gripperVelocityLimitRadS,
                          gripperTorqueLimitNm) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperTarget(md, targetPos) ||
        !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE) ||
        !setGripperTarget(md, targetPos))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to start impedance move for drive with ID: %d",
                    md.m_canId);
        stopDrive(md, "impedance move setup failed; explicit enable required");
        return false;
    }

    return true;
}

bool MdNode::resetDriveErrorsIfNeeded(mab::MD& md)
{
    const auto [quickStatus, statusErr] = md.getQuickStatus();
    using Bits = mab::MDStatus::QuickStatusBits;
    const bool statusUnavailable = statusErr != mab::MD::Error_t::OK;
    const bool inError =
        !statusUnavailable &&
        (quickStatus.at(Bits::MainEncoderStatus).isSet() ||
         quickStatus.at(Bits::OutputEncoderStatus).isSet() ||
         quickStatus.at(Bits::CalibrationEncoderStatus).isSet() ||
         quickStatus.at(Bits::MosfetBridgeStatus).isSet() ||
         quickStatus.at(Bits::HardwareStatus).isSet() ||
         quickStatus.at(Bits::CommunicationStatus).isSet() ||
         quickStatus.at(Bits::MotionStatus).isSet());
    if (!inError)
    {
        if (!statusUnavailable)
            return true;

        RCLCPP_WARN(this->get_logger(),
                    "Failed to read quick status for drive %d; motion rejected until recovery",
                    md.m_canId);
        beginRecovery(md.m_canId);
        return false;
    }
    else
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d is in error state; clearing errors before motion command",
                    md.m_canId);
    }

    if (md.clearErrors() != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to clear errors for drive %d", md.m_canId);
        return false;
    }

    const auto [statusAfterClear, statusAfterClearErr] = md.getQuickStatus();
    if (statusAfterClearErr != mab::MD::Error_t::OK ||
        statusAfterClear.at(Bits::MainEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::OutputEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::CalibrationEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::MosfetBridgeStatus).isSet() ||
        statusAfterClear.at(Bits::HardwareStatus).isSet() ||
        statusAfterClear.at(Bits::CommunicationStatus).isSet() ||
        statusAfterClear.at(Bits::MotionStatus).isSet())
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d is not healthy after clearing errors; motion rejected",
                    md.m_canId);
        beginRecovery(md.m_canId);
        return false;
    }

    return true;
}

std::optional<double> MdNode::readLogicalPosition(mab::MD& md, bool triggerRecovery)
{
    auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end())
        return std::nullopt;

    if (tracker->second.state() == PositionTracker::State::Recovering ||
        tracker->second.state() == PositionTracker::State::Faulted)
        return std::nullopt;

    const auto [rawPosition, positionErr] = md.getPosition();
    if (positionErr != mab::MD::Error_t::OK)
    {
        if (triggerRecovery)
            beginRecovery(md.m_canId);
        return std::nullopt;
    }

    return tracker->second.observe(static_cast<double>(rawPosition));
}

void MdNode::beginRecovery(u16 id)
{
    const bool calibrated = m_gripperCalibrations.erase(id) != 0;
    if (homingRequired || calibrated || m_homingRequired.contains(id))
    {
        m_homingRequired.insert(id);
        m_autoHoming.requestRecovery(id);
    }
    auto md = findMd(m_mds, id);
    if (md != m_mds.end())
        stopDrive(*md, "communication lost; fresh reference required after recovery", true);
    auto tracker = m_positionTrackers.find(id);
    if (tracker == m_positionTrackers.end() ||
        tracker->second.state() == PositionTracker::State::Faulted)
        return;

    tracker->second.markCommunicationLost();
    m_recoveryContexts.try_emplace(
        id, RecoveryContext{std::chrono::steady_clock::now(), false});
}

bool MdNode::canAcceptPositionCommand(u16 id) const
{
    const auto tracker = m_positionTrackers.find(id);
    return tracker != m_positionTrackers.end() && tracker->second.isTracking() &&
           m_recoveryContexts.find(id) == m_recoveryContexts.end() &&
           !m_motionInhibited.contains(id) && !m_homingRequired.contains(id) && !isHoming(id);
}

void MdNode::tickRecoveryJobs()
{
    if (m_recoveryContexts.empty())
        return;

    const auto now = std::chrono::steady_clock::now();
    std::vector<u16> recovered;
    std::vector<u16> faulted;

    for (auto& [id, context] : m_recoveryContexts)
    {
        if (now < context.nextAttempt || m_pendingDisables.contains(id))
            continue;

        context.nextAttempt = now + std::chrono::milliseconds(positionRecoveryRetryMs);
        auto md = findMd(m_mds, id);
        auto tracker = m_positionTrackers.find(id);
        if (md == m_mds.end() || tracker == m_positionTrackers.end())
        {
            faulted.push_back(id);
            continue;
        }

        if (!context.errorsCleared)
        {
            if (md->clearErrors() != mab::MD::Error_t::OK)
                continue;
            context.errorsCleared = true;
        }

        const auto [quickStatus, statusErr] = md->getQuickStatus();
        using Bits = mab::MDStatus::QuickStatusBits;
        if (statusErr != mab::MD::Error_t::OK)
            continue;

        const bool criticalError =
            quickStatus.at(Bits::MainEncoderStatus).isSet() ||
            quickStatus.at(Bits::OutputEncoderStatus).isSet() ||
            quickStatus.at(Bits::CalibrationEncoderStatus).isSet() ||
            quickStatus.at(Bits::MosfetBridgeStatus).isSet() ||
            quickStatus.at(Bits::HardwareStatus).isSet() ||
            quickStatus.at(Bits::CommunicationStatus).isSet() ||
            quickStatus.at(Bits::MotionStatus).isSet();
        if (criticalError)
        {
            context.errorsCleared = false;
            continue;
        }

        const auto [rawPosition, positionErr] = md->getPosition();
        if (positionErr != mab::MD::Error_t::OK)
            continue;

        if (homingRequired || m_homingRequired.contains(id))
        {
            // A returning geared drive has no trustworthy multi-turn reference.
            // Raw coordinates are only used for the bounded opening search.
            if (!std::isfinite(rawPosition)) continue;
            tracker->second.initialize(static_cast<double>(rawPosition));
        }
        else if (tracker->second.state() == PositionTracker::State::Uninitialized)
        {
            tracker->second.initialize(static_cast<double>(rawPosition));
        }
        else if (!tracker->second.isTracking())
        {
            const auto result = tracker->second.observeRecovery(static_cast<double>(rawPosition));
            if (result == PositionTracker::RecoveryResult::Rejected)
            {
                stopDrive(*md, "position recovery ambiguous; manual open calibration required");
                RCLCPP_ERROR(this->get_logger(),
                             "Drive %d recovery is ambiguous; manual open calibration is required", id);
                faulted.push_back(id);
                continue;
            }
            if (result == PositionTracker::RecoveryResult::Pending)
            {
                context.nextAttempt = now;
                continue;
            }
        }

        if (prepareDriveDisabled(*md))
        {
            recovered.push_back(id);
        }
    }

    for (u16 id : recovered)
        m_recoveryContexts.erase(id);
    for (u16 id : faulted)
        m_recoveryContexts.erase(id);
}

void MdNode::stopDrive(mab::MD& md, const std::string& reason, bool recovering, bool diagnosticFault)
{
    // Copy bounded in-memory evidence before erasing jobs or writing disable.
    // Formatting, logging and any new reads happen later in the health callback.
    if (diagnosticFault) captureDiagnosticFault(md, reason);
    if (!recovering) m_autoHoming.disable(md.m_canId);
    // Latch before the bus write: a failed disable must also cancel future work.
    m_motionInhibited.insert(md.m_canId);
    m_softCloseJobs.erase(md.m_canId);
    if (m_homingGroup && std::find(m_homingGroup->ids.begin(), m_homingGroup->ids.end(), md.m_canId) != m_homingGroup->ids.end())
        finishHoming(false);
    m_commandErrors[md.m_canId] = reason;
    RCLCPP_WARN(get_logger(), "Drive %d stopped: %s", md.m_canId, reason.c_str());
    // Repeated stop requests join the same transition, not restart its deadline.
    m_pendingDisables.try_emplace(md.m_canId, md);
}

bool MdNode::prepareDriveDisabled(mab::MD& md)
{
    if (m_pendingDisables.contains(md.m_canId))
        return false;
    candle_ros2::DisableTransition stop(md);
    // Keep the existing short setup fast path. If the motor needs longer to
    // brake, finish asynchronously and require the caller to retry explicitly;
    // never configure/re-enable it while quick-stop is still in progress.
    const auto deadline = stop.started + std::chrono::milliseconds(10);
    do
    {
        const auto status = stop.poll(md);
        if (status == candle_ros2::DisableTransition::Status::Disabled)
            return true;
        if (status == candle_ros2::DisableTransition::Status::Failed)
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    m_pendingDisables.emplace(md.m_canId, std::move(stop));
    stopDrive(md, "configuration deferred until disabled; explicit enable required");
    return false;
}

void MdNode::tickDisableJobs()
{
    using Status = candle_ros2::DisableTransition::Status;
    bool homingFailed = false;
    for (auto& [id, stop] : m_pendingDisables)
    {
        if (stop.reported)
            continue;
        auto md = findMd(m_mds, id);
        const auto status = stop.poll(*md);
        if (status == Status::Pending)
            continue;
        stop.reported = true;
        if (status == Status::Disabled)
            RCLCPP_INFO(this->get_logger(), "Drive %d disable confirmed after %.1f ms", id,
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - stop.started).count());
        else
        {
            m_commandErrors[id] += "; hardware disable unconfirmed";
            m_autoHoming.disable(id);
            m_motionInhibited.insert(id);
            if (isHoming(id)) homingFailed = true;
            RCLCPP_ERROR(this->get_logger(), "Drive %d disable unconfirmed", id);
            if (stop.communicationLost)
            {
                // Losing a disable ACK/readback also loses the reference.
                // Do not schedule motion here: explicit disable must remain authoritative.
                if (m_gripperCalibrations.erase(id) || homingRequired)
                    m_homingRequired.insert(id);
                m_positionTrackers.at(id).markCommunicationLost();
                m_recoveryContexts.try_emplace(id,
                    RecoveryContext{std::chrono::steady_clock::now(), false});
            }
        }
    }
    // Aborting the group can insert other pending disables; do it after iteration.
    if (homingFailed) finishHoming(false);
    for (auto it = m_disableRequests.begin(); it != m_disableRequests.end();)
    {
        bool pending = false;
        candle_ros2::srv::Generic::Response response;
        for (auto id : it->ids)
        {
            const auto stop = m_pendingDisables.find(id);
            pending |= stop != m_pendingDisables.end() && stop->second.status == Status::Pending;
            response.success.push_back(stop != m_pendingDisables.end() &&
                                       stop->second.status == Status::Disabled);
        }
        if (pending)
        {
            ++it;
            continue;
        }
        try { srvDisable->send_response(*it->header, response); }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(this->get_logger(), "Disable response failed: %s", error.what());
        }
        it = m_disableRequests.erase(it);
    }
    // Retain completed results while another ID in a multi-drive request brakes.
    std::erase_if(m_pendingDisables, [&](const auto& entry) {
        return entry.second.status != Status::Pending &&
            std::none_of(m_disableRequests.begin(), m_disableRequests.end(), [&](const auto& request) {
                return std::find(request.ids.begin(), request.ids.end(), entry.first) != request.ids.end();
            });
    });
}

bool MdNode::cancelSoftClose(u16 id)
{
    if (m_pendingDisables.contains(id) || isHoming(id))
        return false;
    const auto job = m_softCloseJobs.find(id);
    if (job == m_softCloseJobs.end())
        return true;
    const auto config = job->second.control.config;
    m_softCloseJobs.erase(job);
    auto md = findMd(m_mds, id);
    if (md == m_mds.end())
        return false;
    const auto position = readLogicalPosition(*md);
    // Clear integral/captured torque AND restore PD before any new command.
    if (!position.has_value() ||
        !setGripperCommand(*md, {*position, 0.0, config.kp, config.kd}))
    {
        stopDrive(*md, "failed to cancel gripper motion; explicit enable required");
        return false;
    }
    return true;
}

bool MdNode::isHoming(u16 id) const
{
    return m_homingJobs.contains(id);
}

bool MdNode::withinCalibratedTravel(u16 id, double position) const
{
    const auto calibrated = m_gripperCalibrations.find(id);
    return std::isfinite(position) && (calibrated == m_gripperCalibrations.end() ||
                                      calibrated->second.motion.withinTravel(position));
}

void MdNode::cbHome(std::shared_ptr<rmw_request_id_t> header,
                    std::shared_ptr<candle_ros2::srv::Generic::Request> req)
{
    // Reply only after the entire group has calibrated and reopened, or aborted.
    if (startHomingGroup(req->device_ids, header)) return;
    candle_ros2::srv::Generic::Response response;
    response.success.assign(req->device_ids.size(), false);
    srvHome->send_response(*header, response);
}

bool MdNode::startHoming(mab::MD& md, std::shared_ptr<rmw_request_id_t> header)
{
    return startHomingGroup({md.m_canId}, std::move(header));
}

bool MdNode::startHomingGroup(const std::vector<u16>& ids, std::shared_ptr<rmw_request_id_t> header)
{
    if (ids.empty() || m_homingGroup || !m_homingJobs.empty()) return false;
    std::unordered_set<u16> unique;
    for (auto id : ids)
        if (!unique.insert(id).second || findMd(m_mds, id) == m_mds.end() ||
            m_pendingDisables.contains(id) || m_recoveryContexts.contains(id)) return false;

    m_homingGroup.emplace(HomingGroup{ids, std::move(header)});
    // Invalidate every member before setup. No drive is enabled until all
    // preflights and configuration writes have passed with the group disabled.
    for (auto id : ids)
    {
        m_gripperCalibrations.erase(id);
        m_homingRequired.insert(id);
        m_motionInhibited.insert(id);
        m_softCloseJobs.erase(id);
    }
    for (auto id : ids)
        if (!prepareHoming(*findMd(m_mds, id))) { finishHoming(false); return true; }
    for (auto id : ids)
        if (!prepareHomingSearch(*findMd(m_mds, id))) { finishHoming(false); return true; }
    RCLCPP_INFO(get_logger(), "Homing group: opening %zu drives together", ids.size());
    for (auto id : ids)
        if (!enableHomingSearch(*findMd(m_mds, id))) { finishHoming(false); return true; }
    return true;
}

bool MdNode::prepareHoming(mab::MD& md)
{
    const auto id = md.m_canId;
    if (gripperOpenPositionRad != 0.0 || initDevicesZero)
    {
        RCLCPP_WARN(get_logger(), "Homing requires open target zero and init_devices_zero=false");
        return false;
    }
    m_gripperCalibrations.erase(id);
    m_homingRequired.insert(id);
    m_motionInhibited.insert(id);
    m_softCloseJobs.erase(id);
    if (!prepareDriveDisabled(md) || !resetDriveErrorsIfNeeded(md)) return false;

    mab::MDRegisters_S r;
    if (md.readRegisters(r.mainEncoderPosition, r.mainEncoderVelocity, r.motorTorque,
                         r.motorTemperature, r.mosfetTemperature, r.motorShutdownTemp) != mab::MD::Error_t::OK ||
        !std::isfinite(r.mainEncoderPosition.value) || !std::isfinite(r.mainEncoderVelocity.value) ||
        !std::isfinite(r.motorTorque.value) ||
        std::abs(r.motorTorque.value) > 0.1 ||
        !candle_ros2::healthyTemperatures(r.motorTemperature.value,
            r.mosfetTemperature.value, r.motorShutdownTemp.value))
    {
        stopDrive(md, "homing preflight failed: unhealthy or loaded drive");
        return false;
    }
    // An old absolute firmware position limit is invalid in the cold frame.
    // Do not silently remove a configured safety limit.
    if (md.readRegisters(r.positionLimitMin, r.positionLimitMax, r.canWatchdog) != mab::MD::Error_t::OK ||
        !ImpedanceHome::unrestrictedPositionLimits(r.positionLimitMin.value, r.positionLimitMax.value) ||
        r.canWatchdog.value == 0 || r.canWatchdog.value > 250)
    {
        stopDrive(md, "homing rejected: check firmware position limits and watchdog (1..250 ms required)");
        return false;
    }
    const double raw = r.mainEncoderPosition.value;
    m_positionTrackers.at(id).initialize(raw);
    const double direction = gripperClosedPositionRad > gripperOpenPositionRad ? -1.0 : 1.0;
    GripperHoming sequence(raw, direction, homeConfig, closeConfig, homingTravelMarginRad);
    m_commandErrors.erase(id);
    m_autoHoming.activate(id);
    m_homingJobs.emplace(id, HomingJob{id, sequence, std::chrono::steady_clock::now()});
    RCLCPP_INFO(get_logger(), "Drive %d homing: raw %.5f rad, opening effort %.2f Nm, damping %.2f, cap %.2f Nm, no preset travel bounds, firmware velocity ceiling %.3f rad/s",
                id, raw, homeConfig.seekTorque, homeConfig.damping, homeConfig.torque,
                static_cast<double>(candle_ros2::gripperFirmwareVelocityCeilingRadS));
    return true;
}

void MdNode::finishHoming(bool success)
{
    if (!m_homingGroup) return;
    const auto group = std::move(*m_homingGroup);
    // Clear before stopping members: stopDrive must not recursively abort us.
    m_homingGroup.reset();
    m_homingJobs.clear();
    for (auto id : group.ids)
    {
        if (success)
        {
            m_homingRequired.erase(id);
            m_motionInhibited.erase(id);
            m_commandErrors.erase(id);
        }
        else
        {
            m_gripperCalibrations.erase(id);
            m_homingRequired.insert(id);
            stopDrive(*findMd(m_mds, id), "homing group aborted; explicit retry required");
        }
    }
    RCLCPP_INFO(get_logger(), "Homing group %s: %zu drives, %.3f s",
        success ? "complete" : "failed", group.ids.size(),
        std::chrono::duration<double>(std::chrono::steady_clock::now() - group.started).count());
    if (group.header)
    {
        candle_ros2::srv::Generic::Response response;
        response.success.assign(group.ids.size(), success);
        try { srvHome->send_response(*group.header, response); }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "Homing response failed: %s", error.what());
        }
    }
}

void MdNode::tickHomingGroup()
{
    if (!m_homingGroup) return;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - m_homingGroup->started).count() >
        3 * (homeConfig.timeout + homeConfig.releaseTimeout) + 5.0)
    {
        finishHoming(false);
        return;
    }
    if (m_homingJobs.size() != m_homingGroup->ids.size() ||
        std::any_of(m_homingJobs.begin(), m_homingJobs.end(), [](const auto& entry) {
            return !entry.second.waitingForGroup ||
                entry.second.sequence.control.state != ImpedanceHome::State::Ready;
        })) return;
    const auto ids = m_homingGroup->ids; // A failed hardware operation can erase the group.
    const auto phase = m_homingJobs.at(ids.front()).sequence.phase;
    for (auto id : ids)
    {
        auto& md = *findMd(m_mds, id);
        auto& job = m_homingJobs.at(id);
        if (job.sequence.phase != phase || m_pendingDisables.contains(id) ||
            !candle_ros2::driveDisabled(md.m_mdRegisters.state.value))
        {
            stopDrive(md, "homing phase barrier lost disabled state");
            return;
        }
    }
    for (auto id : ids)
    {
        auto& md = *findMd(m_mds, id);
        if (phase == GripperHoming::Phase::Opening)
            completeHomingZero(md);
        else if (phase == GripperHoming::Phase::Closing)
        {
            if (!m_homingJobs.at(id).sequence.startReopening(m_positionTrackers.at(id).continuousPosition()))
                stopDrive(md, "homing reopening transition failed; calibration required");
        }
        else if (phase == GripperHoming::Phase::Ready)
            completeHomingCalibration(md);
        else
            stopDrive(md, "invalid homing group phase");
        if (!m_homingGroup) return;
    }
    if (phase == GripperHoming::Phase::Ready)
    {
        finishHoming(true);
        return;
    }
    for (auto id : ids)
        if (!prepareHomingSearch(*findMd(m_mds, id))) { finishHoming(false); return; }
    RCLCPP_INFO(get_logger(), "Homing group: %s %zu drives together",
        phase == GripperHoming::Phase::Opening ? "closing" : "reopening", ids.size());
    for (auto id : ids)
        if (!enableHomingSearch(*findMd(m_mds, id))) { finishHoming(false); return; }
}

void MdNode::tickHoming(mab::MD& md, double position, double velocity, double torque)
{
    auto& job = m_homingJobs.at(md.m_canId);
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - job.lastUpdate).count();
    job.lastUpdate = now;
    if (job.waitingForGroup)
    {
        // Passive rebound may resume while peers are still releasing. Keep
        // checking the original settling window and release deadline, rather
        // than freezing the first settled position or restarting the timeout.
        job.sequence.control.state = ImpedanceHome::State::Releasing;
        if (!candle_ros2::driveDisabled(md.m_mdRegisters.state.value) ||
            job.sequence.control.updateRelease(position, velocity, torque, dt) == ImpedanceHome::State::Failed)
            stopDrive(md, "homing phase barrier lost disabled/settled feedback before release deadline");
        return;
    }
    const bool releasing = job.sequence.control.state == ImpedanceHome::State::Releasing;
    if (releasing)
    {
        if (m_pendingDisables.contains(md.m_canId)) return;
        if (!candle_ros2::driveDisabled(md.m_mdRegisters.state.value))
        {
            stopDrive(md, "homing release is not disabled; reference remains invalid");
            return;
        }
    }
    else if (!candle_ros2::driveEnabled(md.m_mdRegisters.state.value) ||
             md.m_mdRegisters.motionModeStatus.value != mab::MdMode_E::IMPEDANCE)
    {
        if (m_homingRequired.contains(md.m_canId) &&
            candle_ros2::driveDisabled(md.m_mdRegisters.state.value) &&
            md.m_mdRegisters.motionModeStatus.value == mab::MdMode_E::IDLE)
            beginRecovery(md.m_canId);
        else
            stopDrive(md, "homing drive state/mode lost; reference remains invalid");
        return;
    }
    const auto appliedCommand = job.sequence.control.command();
    if (!releasing && now - job.lastVerification >= std::chrono::milliseconds(100))
    {
        if (!verifyGripperCommand(md, appliedCommand)) return;
        job.lastVerification = now;
    }
    const auto searchPhase = job.sequence.phase;
    const auto state = job.sequence.update(position, velocity, torque, dt);
    if (state == ImpedanceHome::State::Failed)
    {
        RCLCPP_ERROR(get_logger(), "Drive %d homing failed at %.5f rad, velocity %.3f, torque %.3f, elapsed %.3f",
                     md.m_canId, position, velocity, torque, job.sequence.control.elapsed);
        // Preserve which search failed; GripperHoming::fail changes phase to
        // Failed before the node captures the evidence.
        job.sequence.phase = searchPhase;
        stopDrive(md, "homing search/release/feedback failed; calibration required");
        return;
    }
    if (releasing)
    {
        if (state != ImpedanceHome::State::Ready) return;
        job.waitingForGroup = true;
        RCLCPP_INFO(get_logger(), "Drive %d homing phase settled; waiting for group", md.m_canId);
        return;
    }
    // Verify the search command before clearing it or accepting a reference.
    if (state == ImpedanceHome::State::Found && !verifyGripperCommand(md, appliedCommand)) return;
    // Copy before any write: a failed command can cancel the job.
    const auto command = job.sequence.control.command();
    if (!setGripperCommand(md, command) || state != ImpedanceHome::State::Found) return;

    const char* stage = job.sequence.phase == GripperHoming::Phase::Closing ? "closing" :
                        job.sequence.phase == GripperHoming::Phase::Reopening ? "reopening" : "opening";
    RCLCPP_INFO(get_logger(), "Drive %d %s stop inferred at %.5f rad, torque %.3f Nm",
                md.m_canId, stage, position, torque);
    // Disable asynchronously: braking may exceed a control tick, but no zero or
    // new command is allowed before confirmed disabled feedback and release dwell.
    if (!m_homingJobs.at(md.m_canId).sequence.beginRelease())
    {
        stopDrive(md, "homing release transition failed; calibration required");
        return;
    }
    m_pendingDisables.try_emplace(md.m_canId, md);
    m_homingJobs.at(md.m_canId).lastUpdate = std::chrono::steady_clock::now();
}

void MdNode::completeHomingZero(mab::MD& md)
{
    // The unpowered jaw has now settled after release. This is
    // the relaxed open reference; zeroing a loaded stop would bake in deflection.
    if (md.zero() != mab::MD::Error_t::OK)
    {
        stopDrive(md, "homing zero failed; reference remains invalid");
        return;
    }
    // runZero is acknowledged before the position register updates on this
    // firmware. Poll the readback briefly while disabled; never reissue zero.
    const auto zeroPosition = candle_ros2::readZeroPosition(md, closeConfig.tolerance);
    if (!zeroPosition)
    {
        stopDrive(md, "homing zero readback failed; reference remains invalid");
        return;
    }
    const double raw = *zeroPosition;
    m_positionTrackers.at(md.m_canId).resetAtZero();
    m_positionTrackers.at(md.m_canId).observe(raw);
    if (!m_homingJobs.at(md.m_canId).sequence.startClosing(raw))
    {
        stopDrive(md, "homing zero-to-close transition failed; calibration required");
        return;
    }
    RCLCPP_INFO(get_logger(), "Drive %d open reference zeroed; seeking closed stop (travel margin %.3f rad)",
                md.m_canId, homingTravelMarginRad);
}

bool MdNode::prepareHomingSearch(mab::MD& md)
{
    // Each new search starts with a fresh command in the current encoder frame.
    // The previous phase has already confirmed disabled and unloaded feedback.
    const auto command = m_homingJobs.at(md.m_canId).sequence.control.command();
    const bool closing = m_homingJobs.at(md.m_canId).sequence.phase == GripperHoming::Phase::Closing;
    const double speed = closing ? gripperVelocityLimitRadS : gripperOpenVelocityLimitRadS;
    if (!configureGripper(md, 0.0, homeConfig.damping, speed, homeConfig.torque) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperCommand(md, command))
    {
        stopDrive(md, "homing stage setup failed; calibration required");
        return false;
    }
    return true;
}

bool MdNode::enableHomingSearch(mab::MD& md)
{
    const auto command = m_homingJobs.at(md.m_canId).sequence.control.command();
    if (!candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE) ||
        !setGripperCommand(md, command) || !verifyGripperCommand(md, command))
    {
        stopDrive(md, "homing stage enable/readback failed; calibration required");
        return false;
    }
    auto& job = m_homingJobs.at(md.m_canId);
    job.waitingForGroup = false;
    job.lastUpdate = std::chrono::steady_clock::now();
    job.lastVerification = job.lastUpdate;
    return true;
}

void MdNode::completeHomingCalibration(mab::MD& md)
{
    const auto calibration = m_homingJobs.at(md.m_canId).sequence.calibration();
    if (!calibration)
    {
        stopDrive(md, "homing calibration incomplete; calibration required");
        return;
    }
    const double position = m_positionTrackers.at(md.m_canId).continuousPosition();
    if (!configureGripper(md, 0.0, 0.0, gripperOpenVelocityLimitRadS, closeConfig.torqueLimit) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperCommand(md, {position, 0.0, 0.0, 0.0}) ||
        !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE) ||
        !setGripperCommand(md, {position, 0.0, 0.0, 0.0}) ||
        !verifyGripperCommand(md, {position, 0.0, 0.0, 0.0}, closeConfig.torqueLimit))
    {
        stopDrive(md, "homing final release setup failed; calibration required");
        return;
    }
    ImpedanceClose release(position, 0.0, gripperOpenVelocityLimitRadS, calibration->motion, true);
    release.state = ImpedanceClose::State::TorqueHold;
    m_softCloseJobs.insert_or_assign(md.m_canId, SoftCloseJob{release, std::chrono::steady_clock::now(), true});
    m_gripperCalibrations.insert_or_assign(md.m_canId, *calibration);
    RCLCPP_INFO(get_logger(), "Drive %d homing complete: closed stop %.5f rad, travel [%.5f, %.5f] rad; "
                "reopened at %.5f rad, released at 0 Nm; loaded open stop %.5f rad; normal limits restored", md.m_canId,
                calibration->closedPosition, calibration->motion.travelMin, calibration->motion.travelMax, position,
                calibration->openPosition);
}

bool MdNode::releasingDisabled(u16 id) const
{
    return isHoming(id) && (m_homingJobs.at(id).waitingForGroup ||
        m_homingJobs.at(id).sequence.control.state == ImpedanceHome::State::Releasing);
}

double MdNode::normalizedSpeedToRadS(double normalizedSpeed)
{
    return SOFT_CLOSE_MIN_SPEED_RAD_S +
           normalizedSpeed * (SOFT_CLOSE_MAX_SPEED_RAD_S - SOFT_CLOSE_MIN_SPEED_RAD_S);
}

bool MdNode::startImpedanceMotion(mab::MD& md, double speed, bool opening)
{
    if (!canAcceptPositionCommand(md.m_canId))
        return false;
    // Replace the old job directly: cancelSoftClose's zero-torque PD command
    // would introduce a release between repeated close requests.
    m_softCloseJobs.erase(md.m_canId);
    const auto position = readLogicalPosition(md);
    if (!position.has_value())
    {
        stopDrive(md, "gripper motion position unavailable; explicit enable required");
        return false;
    }
    const auto calibrated = m_gripperCalibrations.find(md.m_canId);
    const auto& motion = calibrated == m_gripperCalibrations.end() ? closeConfig : calibrated->second.motion;
    const double closedPosition = calibrated == m_gripperCalibrations.end() ?
                                  gripperClosedPositionRad : calibrated->second.closedPosition;
    if (!motion.withinTravel(*position))
    {
        stopDrive(md, "gripper motion rejected: position outside calibrated travel");
        return false;
    }

    const double torqueLimit = std::min(gripperTorqueLimitNm, softCloseSlowTorqueLimitNm);
    if (!resetDriveErrorsIfNeeded(md) ||
        md.readRegisters(md.m_mdRegisters.state, md.m_mdRegisters.motionModeStatus) !=
            mab::MD::Error_t::OK)
    {
        stopDrive(md, "impedance motion state unavailable; explicit enable required");
        return false;
    }
    // Do not toggle a healthy, already-enabled impedance drive between every
    // open/close command. Replace its old hold with the new controller's initial
    // command. Other modes still require a confirmed disabled transition.
    const bool alreadyReady = candle_ros2::driveEnabled(md.m_mdRegisters.state.value) &&
                             md.m_mdRegisters.motionModeStatus.value == mab::MdMode_E::IMPEDANCE;
    // Calibrated direction keeps the closing floor positive toward closure even
    // when starting at/slightly beyond the target. Opening never gets the floor.
    const double direction = (closedPosition > gripperOpenPositionRad ? 1.0 : -1.0) *
                             (opening ? -1.0 : 1.0);
    ImpedanceClose control(*position, opening ? gripperOpenPositionRad : closedPosition,
                           speed, motion, opening, direction);
    const auto hold = control.command();
    if ((!alreadyReady && (!prepareDriveDisabled(md) ||
                          md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK)) ||
        !setGripperCommand(md, hold) ||
        !configureGripper(md, hold.kp, hold.kd, speed, torqueLimit) ||
        (!alreadyReady && !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE)) ||
        !setGripperCommand(md, hold) || !verifyGripperCommand(md, hold, torqueLimit))
    {
        stopDrive(md, "impedance motion setup failed; explicit enable required");
        return false;
    }

    m_softCloseJobs.insert_or_assign(md.m_canId, SoftCloseJob{
        control, std::chrono::steady_clock::now(), opening});
    RCLCPP_INFO(this->get_logger(), "Drive %d %s in impedance: ramp %.3f rad/s, "
                "lead %.4f rad, torque limit %.3f Nm, Ki %.3f, firmware velocity ceiling %.3f rad/s", md.m_canId,
                opening ? "opening" : "closing", speed,
                opening ? closeConfig.openLead : closeConfig.lead,
                torqueLimit, opening ? 0.0 : closeConfig.ki,
                static_cast<double>(candle_ros2::gripperFirmwareVelocityCeilingRadS));
    return true;
}

void MdNode::tickSoftClose(mab::MD& md, double position, double velocity, double torque)
{
    auto it = m_softCloseJobs.find(md.m_canId);
    if (it == m_softCloseJobs.end())
        return;
    auto& job = it->second;
    const bool opening = job.opening;
    const char* operation = opening ? "open" : "close";
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - job.lastUpdate).count();
    job.lastUpdate = now;
    const auto previous = job.control.state;
    const auto previousCommand = job.control.command();
    if (!job.control.config.withinTravel(position))
    {
        stopDrive(md, std::string(operation) + " failed: measured position outside calibrated travel");
        return;
    }
    // These registers came from the same fresh feedback frame as position and
    // torque. Healthy quick-status flags alone do not mean PWM is enabled.
    if (!candle_ros2::driveEnabled(md.m_mdRegisters.state.value) ||
        md.m_mdRegisters.motionModeStatus.value != mab::MdMode_E::IMPEDANCE)
    {
        const std::string reason = std::string(operation) +
            " failed: drive not enabled in impedance mode (state=" +
            std::to_string(md.m_mdRegisters.state.value) + ", mode=" +
            std::to_string(md.m_mdRegisters.motionModeStatus.value) +
            "); explicit enable required";
        RCLCPP_ERROR(this->get_logger(), "Drive %d %s", md.m_canId, reason.c_str());
        if ((homingRequired || m_gripperCalibrations.contains(md.m_canId)) &&
            candle_ros2::driveDisabled(md.m_mdRegisters.state.value) &&
            md.m_mdRegisters.motionModeStatus.value == mab::MdMode_E::IDLE)
            beginRecovery(md.m_canId); // A brief power loss may occur between telemetry ticks.
        else
            stopDrive(md, reason);
        return;
    }
    // Handle reference loss/reset from fresh state before checking command
    // readback; a reset must not retain an old calibration behind a fault latch.
    // Stagger the six drives instead of issuing six verification batches together.
    if (now - job.lastVerification >= std::chrono::milliseconds(100) &&
        now - m_lastCommandVerification >= std::chrono::milliseconds(jointStatePublishPeriodMs))
    {
        m_lastCommandVerification = now;
        if (!verifyGripperCommand(md, previousCommand)) return;
        job.lastVerification = now;
    }
    const auto state = job.control.update(position, velocity, torque, dt);
    if (state == ImpedanceClose::State::Failed)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d %s aborted at %.4f rad; target %.4f rad, torque %.3f Nm",
                    md.m_canId, operation, position, job.control.target, torque);
        stopDrive(md, std::string(operation) +
                      " failed: stale/invalid feedback; "
                      "explicit enable required");
        return;
    }
    if (opening && state == ImpedanceClose::State::Contact)
    {
        // Opening resistance is an obstruction, not a successful grasp.
        stopDrive(md, "open blocked: sustained resistance before endpoint; explicit enable required");
        return;
    }
    // A failed write can erase the job: copy everything used afterwards.
    const auto command = job.control.command();
    const bool hasObject = job.control.hasObject;
    const double desiredTorque = job.control.direction *
        std::clamp(state == ImpedanceClose::State::Closed ? closeConfig.closedHoldTorque :
                   closeConfig.holdTorque, std::min(closeConfig.closedHoldTorque, closeConfig.torqueLimit),
                   closeConfig.torqueLimit);
    if ((command.position != previousCommand.position ||
         command.torque != previousCommand.torque ||
         command.kp != previousCommand.kp || command.kd != previousCommand.kd) &&
        !setGripperCommand(md, command))
        return;
    if (state == previous)
        return;
    if (state == ImpedanceClose::State::Closing)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d close: recovering from measured position %.4f rad; "
                    "old hold torque and integral cleared; closing command floor %.4f Nm",
                    md.m_canId, position, std::min(closeConfig.closedHoldTorque, closeConfig.torqueLimit));
    else if (state == ImpedanceClose::State::Contact && hasObject)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d close: contact inferred at %.4f rad; torque ramp %.4f -> %.4f Nm",
                    md.m_canId, position, command.torque,
                    desiredTorque);
    else if (opening && state == ImpedanceClose::State::TorqueHold)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d open: timeout released at %.4f rad; commanded torque 0 Nm, "
                    "PD disabled, endpoint unconfirmed", md.m_canId, position);
    else if (state == ImpedanceClose::State::TorqueHold)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d close: %s at %.4f rad; holding torque %.4f Nm",
                    md.m_canId, hasObject ? "object torque hold" : "timeout torque hold",
                    position, command.torque);
    else if (opening && state == ImpedanceClose::State::Closed)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d open: resistance within allowed endpoint band at %.5f rad; "
                    "commanded torque 0 Nm, PD disabled, calibration unchanged", md.m_canId, position);
    else if (!opening && state == ImpedanceClose::State::Closed)
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d close: closed endpoint at %.4f rad; torque ramp %.4f -> %.4f Nm",
                    md.m_canId, position, command.torque, desiredTorque);
    else
        RCLCPP_INFO(this->get_logger(), "Drive %d %s: %s at %.4f rad; holding target %.4f rad",
                    md.m_canId, operation,
                    state == ImpedanceClose::State::Contact ? "contact inferred" :
                        (opening ? "open endpoint" : "closed endpoint"),
                    position, command.position);
}

void MdNode::cbOpenGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                           std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        rsp->success.push_back(md != m_mds.end() &&
                               startImpedanceMotion(*md, gripperOpenVelocityLimitRadS, true));
    }
}

void MdNode::cbCloseGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                            std::shared_ptr<candle_ros2::srv::Generic::Response> rsp)
{
    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        rsp->success.push_back(md != m_mds.end() &&
                               startImpedanceMotion(*md, gripperVelocityLimitRadS));
    }
}

void MdNode::cbSoftCloseGripper(
    const std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Request> req,
    std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Response> rsp)
{
    if (!std::isfinite(req->slow_speed) || req->slow_speed < 0.0 || req->slow_speed > 1.0)
    {
        rsp->success.assign(req->device_ids.size(), false);
        return;
    }
    if (req->pre_close_enabled)
        RCLCPP_WARN(this->get_logger(), "Pre-close has been removed; using continuous impedance close");
    const double speed = std::min<double>(gripperVelocityLimitRadS,
                                          normalizedSpeedToRadS(req->slow_speed));
    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        rsp->success.push_back(md != m_mds.end() && startImpedanceMotion(*md, speed));
    }
}

void MdNode::cbSetGripperTargets(
    const std::shared_ptr<candle_ros2::srv::SetGripperTargets::Request> req,
    std::shared_ptr<candle_ros2::srv::SetGripperTargets::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);
    if (n == 0 || req->target_position_rad.size() != n)
    {
        RCLCPP_WARN(this->get_logger(),
                    "SetGripperTargets request arrays must be non-empty and equally sized");
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        const double target = req->target_position_rad[i];
        if (!std::isfinite(target))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid gripper command values for drive with ID: %d",
                        req->device_ids[i]);
            continue;
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", req->device_ids[i]);
            continue;
        }
        if (!canAcceptPositionCommand(req->device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; target rejected",
                        req->device_ids[i]);
            continue;
        }
        if (!withinCalibratedTravel(md->m_canId, target) ||
            !candle_ros2::finiteRegisterValue(m_positionTrackers.at(md->m_canId).logicalToRaw(target)))
            continue; // Invalid requests must not cancel a valid active hold.
        rsp->success[i] = cancelSoftClose(req->device_ids[i]) && setGripperTarget(*md, target);
    }
}

void MdNode::cbConfigureGripper(
    const std::shared_ptr<candle_ros2::srv::ConfigureGripper::Request> req,
    std::shared_ptr<candle_ros2::srv::ConfigureGripper::Response>      rsp)
{
    const size_t n = req->device_ids.size();
    rsp->success.assign(n, false);
    if (n == 0 || req->kp.size() != n || req->kd.size() != n ||
        req->velocity_limit_rad_s.size() != n || req->torque_limit_nm.size() != n)
    {
        RCLCPP_WARN(this->get_logger(),
                    "ConfigureGripper request arrays must be non-empty and equally sized");
        return;
    }

    for (size_t i = 0; i < n; ++i)
    {
        const double kp            = req->kp[i];
        const double kd            = req->kd[i];
        const double velocityLimit = req->velocity_limit_rad_s[i];
        const double torqueLimit   = req->torque_limit_nm[i];
        if (!std::isfinite(kp) || kp < 0.0 || !std::isfinite(kd) || kd < 0.0 ||
            !std::isfinite(velocityLimit) || velocityLimit <= 0.0 || !std::isfinite(torqueLimit) ||
            torqueLimit <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Invalid gripper configuration for drive with ID: %d",
                        req->device_ids[i]);
            continue;
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", req->device_ids[i]);
            continue;
        }
        if (isHoming(req->device_ids[i]) || m_softCloseJobs.contains(req->device_ids[i]) || m_pendingDisables.contains(req->device_ids[i]))
            continue;
        rsp->success[i] = configureGripper(*md, kp, kd, velocityLimit, torqueLimit);
        if (!rsp->success[i]) stopDrive(*md, "gripper configuration failed; explicit enable required");
    }
}

void MdNode::cbSetMode(const std::shared_ptr<candle_ros2::srv::SetMode::Request> req,
                       std::shared_ptr<candle_ros2::srv::SetMode::Response>      rsp)
{
    if (req->device_ids.size() != req->mode.size())
    {
        rsp->success.assign(req->device_ids.size(), false);

        RCLCPP_WARN(this->get_logger(),
                    "SetMode request incomplete. Sizes of arrays do not match!");
        return;
    }

    rsp->success.reserve(req->device_ids.size());

    for (size_t i = 0; i < req->device_ids.size(); i++)
    {
        mab::MdMode_E      mode    = mab::MdMode_E::IDLE;
        const std::string& reqMode = req->mode[i];

        if (reqMode == "IMPEDANCE")
            mode = mab::MdMode_E::IMPEDANCE;
        else if (reqMode == "POSITION_PID")
            mode = mab::MdMode_E::POSITION_PID;
        else if (reqMode == "POSITION_PROFILE")
            mode = mab::MdMode_E::POSITION_PROFILE;
        else if (reqMode == "VELOCITY_PID")
            mode = mab::MdMode_E::VELOCITY_PID;
        else if (reqMode == "RAW_TORQUE")
            mode = mab::MdMode_E::RAW_TORQUE;
        else if (reqMode != "IDLE")
        {
            RCLCPP_WARN(this->get_logger(), "Unknown mode %s rejected for drive %d",
                        reqMode.c_str(), req->device_ids[i]);
            rsp->success.push_back(false);
            continue;
        }

        auto md = findMd(m_mds, req->device_ids[i]);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (!cancelSoftClose(req->device_ids[i]))
        {
            rsp->success.push_back(false);
            continue;
        }
        if (md->setMotionMode(mode) == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
        else
            rsp->success.push_back(false);
    }
    return;
}

void MdNode::cbEnable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                      std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            rsp->success.push_back(false);
            continue;
        }

        if (!cancelSoftClose(id) || m_recoveryContexts.contains(id) || m_homingRequired.contains(id))
        {
            rsp->success.push_back(false);
            continue;
        }
        const auto position = readLogicalPosition(*md, false);
        if (position.has_value() && moveGripper(*md, *position))
        {
            m_motionInhibited.erase(id);
            m_autoHoming.activate(id);
            m_commandErrors.erase(id);
            rsp->success.push_back(true);
        }
        else
        {
            stopDrive(*md, "enable failed; position/reference must be valid");
            rsp->success.push_back(false);
        }
    }
    return;
}

void MdNode::cbDisable(std::shared_ptr<rmw_request_id_t> header,
                       std::shared_ptr<candle_ros2::srv::Generic::Request> req)
{
    for (auto id : req->device_ids)
    {
        m_autoHoming.disable(id); // Explicit stop persists across later communication losses.
        auto md = findMd(m_mds, id);
        if (md != m_mds.end())
            stopDrive(*md, "disabled by request; explicit enable required", false, false);
    }
    m_disableRequests.push_back({std::move(header), req->device_ids});
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
