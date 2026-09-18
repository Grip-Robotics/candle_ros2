#include "candle_ros2/md_node.hpp"
#include "candle_ros2/drive_state.hpp"

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
    m_homingJob.reset();
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
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::publishJointStates()
{
    tickDisableJobs();
    tickRecoveryJobs();
    if (!m_homingJob)
        for (auto& md : m_mds)
            if (m_autoHomePending.contains(md.m_canId) &&
                !m_recoveryContexts.contains(md.m_canId) &&
                !m_pendingDisables.contains(md.m_canId))
            {
                m_autoHomePending.erase(md.m_canId); // One attempt; failed homing never retries itself.
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
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        if (isHoming(md.m_canId))
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
            if (m_softCloseJobs.contains(md.m_canId) || isHoming(md.m_canId))
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
        if (m_pendingDisables.contains(md.m_canId))
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
            std::isfinite(temperatures.motorTemperature.value) &&
            std::isfinite(temperatures.mosfetTemperature.value))
        {
            msg.motor_temperature_c.back() = temperatures.motorTemperature.value;
            msg.mosfet_temperature_c.back() = temperatures.mosfetTemperature.value;
            msg.motor_shutdown_temperature_c.back() = temperatures.motorShutdownTemp.value;
        }
        else
        {
            if (!description.empty())
                description += "; ";
            description += "temperature: invalid or unavailable";
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
            if (m_softCloseJobs.contains(md.m_canId) || isHoming(md.m_canId))
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
        auto md = findMd(m_mds, msg.device_ids[i]);
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

        if (!std::isfinite(msg.target_position[i]) ||
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

    if (n != msg.position_pid.size())
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
        }
    }
    return;
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                     std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
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
            RCLCPP_WARN(
                this->get_logger(), "Init devices: failed to zero drive %d", req->device_ids[i]);
            continue;
        }
        if (initDevicesZero)
            m_positionTrackers.at(req->device_ids[i]).resetAtZero();

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
        m_commandErrors.erase(req->device_ids[i]);
        rsp->success[i] = true;
    }
}

void MdNode::cbZero(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
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

        if (!cancelSoftClose(id))
        {
            rsp->success.push_back(false);
            continue;
        }

        if (md->zero() != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to zero drive with ID: %d", id);
            rsp->success.push_back(false);
            continue;
        }

        m_recoveryContexts.erase(id);
        m_positionTrackers.at(id).resetAtZero();

        // Zero changes the position frame but does not update the active
        // target. Replace the pre-zero target immediately so the drive holds
        // the new mechanical-open reference instead of moving toward it.
        if (!setGripperTarget(*md, 0.0))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Failed to hold drive %d at zero after calibration; disabling it",
                        id);
            stopDrive(*md, "zero hold failed; explicit enable required");
            rsp->success.push_back(false);
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Drive %d zeroed and holding at logical 0 rad", id);
        rsp->success.push_back(true);
    }
    return;
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

    mab::MDRegisters_S limitRegs;
    limitRegs.profileVelocity = static_cast<float>(velocityLimit);
    limitRegs.maxVelocity = std::max(gripperVelocityLimitRadS, gripperOpenVelocityLimitRadS);
    limitRegs.maxTorque       = static_cast<float>(torqueLimit);
    if (md.writeRegisters(limitRegs.profileVelocity, limitRegs.maxVelocity, limitRegs.maxTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set limits for drive with ID: %d", md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::setGripperTarget(mab::MD& md, double targetPos)
{
    auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking())
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
    const auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking() ||
        !std::isfinite(command.position) || !std::isfinite(command.torque) ||
        !std::isfinite(command.kp) || !std::isfinite(command.kd) ||
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
    return true;
}

bool MdNode::writeRawTarget(mab::MD& md, double rawTargetPos)
{
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
    if (homingRequired)
    {
        m_homingRequired.insert(id);
        m_autoHomePending.insert(id);
    }
    auto md = findMd(m_mds, id);
    if (md != m_mds.end())
        stopDrive(*md, "communication lost; explicit enable required after recovery");
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
            quickStatus.at(Bits::MotionStatus).isSet();
        if (criticalError)
        {
            context.errorsCleared = false;
            continue;
        }

        const auto [rawPosition, positionErr] = md->getPosition();
        if (positionErr != mab::MD::Error_t::OK)
            continue;

        if (homingRequired)
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

void MdNode::stopDrive(mab::MD& md, const std::string& reason)
{
    // Latch before the bus write: a failed disable must also cancel future work.
    m_motionInhibited.insert(md.m_canId);
    m_softCloseJobs.erase(md.m_canId);
    if (isHoming(md.m_canId)) finishHoming(false);
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
            RCLCPP_ERROR(this->get_logger(), "Drive %d disable unconfirmed", id);
            if (stop.communicationLost)
            {
                m_positionTrackers.at(id).markCommunicationLost();
                m_recoveryContexts.try_emplace(id,
                    RecoveryContext{std::chrono::steady_clock::now(), false});
            }
        }
    }
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
    return m_homingJob && m_homingJob->id == id;
}

void MdNode::cbHome(std::shared_ptr<rmw_request_id_t> header,
                    std::shared_ptr<candle_ros2::srv::Generic::Request> req)
{
    // Deferred reply means success is a completed zero, never just an ACK.
    if (req->device_ids.size() == 1)
    {
        auto md = findMd(m_mds, req->device_ids.front());
        if (md != m_mds.end() && startHoming(*md, header)) return;
    }
    candle_ros2::srv::Generic::Response response;
    response.success.assign(req->device_ids.size(), false);
    srvHome->send_response(*header, response);
}

bool MdNode::startHoming(mab::MD& md, std::shared_ptr<rmw_request_id_t> header)
{
    const auto id = md.m_canId;
    if (m_homingJob || m_pendingDisables.contains(id) || m_recoveryContexts.contains(id))
        return false;
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
        !std::isfinite(r.motorTemperature.value) || !std::isfinite(r.mosfetTemperature.value) ||
        !std::isfinite(r.motorShutdownTemp.value) || r.motorShutdownTemp.value <= 0.0 ||
        r.motorTemperature.value <= 0.0 || r.motorTemperature.value >= r.motorShutdownTemp.value ||
        r.mosfetTemperature.value <= 0.0 || r.mosfetTemperature.value >= 90.0)
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
    ImpedanceHome control(raw, direction, homeConfig, closeConfig);
    // Read back the homing torque cap and ordinary firmware velocity protection.
    // Opening torque with velocity damping, no position spring or absolute endpoint.
    if (!configureGripper(md, 0.0, homeConfig.damping,
                          gripperOpenVelocityLimitRadS, homeConfig.torque) ||
        md.readRegisters(r.maxVelocity, r.maxTorque) != mab::MD::Error_t::OK ||
        !std::isfinite(r.maxVelocity.value) || !std::isfinite(r.maxTorque.value) ||
        std::abs(r.maxVelocity.value - std::max(gripperVelocityLimitRadS, gripperOpenVelocityLimitRadS)) > 1e-6 ||
        std::abs(r.maxTorque.value - homeConfig.torque) > 1e-6 ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperCommand(md, control.command()) ||
        !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE) ||
        !setGripperCommand(md, control.command()))
    {
        stopDrive(md, "homing setup failed; reference remains invalid");
        return false;
    }
    m_commandErrors.erase(id);
    m_autoHomePending.erase(id);
    m_homingJob.emplace(HomingJob{id, control, std::chrono::steady_clock::now(), std::move(header)});
    RCLCPP_INFO(get_logger(), "Drive %d homing: raw %.5f rad, opening effort %.2f Nm, damping %.2f, cap %.2f Nm, max travel %.2f rad",
                id, raw, homeConfig.seekTorque, homeConfig.damping, homeConfig.torque, homeConfig.maxTravel);
    return true;
}

void MdNode::finishHoming(bool success)
{
    if (!m_homingJob) return;
    auto header = m_homingJob->header;
    const auto id = m_homingJob->id;
    m_homingJob.reset();
    if (success)
    {
        m_homingRequired.erase(id);
        m_motionInhibited.erase(id);
        m_commandErrors.erase(id);
    }
    if (header)
    {
        candle_ros2::srv::Generic::Response response;
        response.success = {success};
        try { srvHome->send_response(*header, response); }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "Homing response failed: %s", error.what());
        }
    }
}

void MdNode::tickHoming(mab::MD& md, double position, double velocity, double torque)
{
    auto& job = *m_homingJob;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - job.lastUpdate).count();
    job.lastUpdate = now;
    if (job.control.state == ImpedanceHome::State::Releasing)
    {
        if (!candle_ros2::driveDisabled(md.m_mdRegisters.state.value))
        {
            stopDrive(md, "homing release is not disabled; reference remains invalid");
            return;
        }
        const auto result = job.control.updateRelease(position, velocity, torque, dt);
        if (result == ImpedanceHome::State::Failed)
            stopDrive(md, "homing release settling/feedback timeout; reference remains invalid");
        else if (result == ImpedanceHome::State::Ready)
            completeHomingZero(md);
        return;
    }
    if (!candle_ros2::driveEnabled(md.m_mdRegisters.state.value) ||
        md.m_mdRegisters.motionModeStatus.value != mab::MdMode_E::IMPEDANCE)
    {
        if (homingRequired && candle_ros2::driveDisabled(md.m_mdRegisters.state.value) &&
            md.m_mdRegisters.motionModeStatus.value == mab::MdMode_E::IDLE)
            beginRecovery(md.m_canId);
        else
            stopDrive(md, "homing drive state/mode lost; reference remains invalid");
        return;
    }
    const auto state = job.control.update(position, velocity, torque, dt);
    if (state == ImpedanceHome::State::Failed)
    {
        RCLCPP_ERROR(get_logger(), "Drive %d homing failed at raw %.5f rad, velocity %.3f, torque %.3f, elapsed %.3f",
                     md.m_canId, position, velocity, torque, job.control.elapsed);
        stopDrive(md, "homing timeout/travel/torque/feedback guard; reference remains invalid");
        return;
    }
    // Copy before any write: a failed command can cancel the job.
    const auto command = job.control.command();
    if (!setGripperCommand(md, command) || state != ImpedanceHome::State::Found) return;

    RCLCPP_INFO(get_logger(), "Drive %d opening stop inferred at raw %.5f rad, torque %.3f Nm",
                md.m_canId, position, torque);
    // Never zero under an active spring or re-enable with a pre-zero target.
    if (!prepareDriveDisabled(md))
    {
        stopDrive(md, "homing disable failed; reference remains invalid");
        return;
    }
    m_homingJob->control.beginRelease();
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
    if (!configureGripper(md, 0.0, 0.0, gripperOpenVelocityLimitRadS, closeConfig.torqueLimit) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperCommand(md, {raw, 0.0, 0.0, 0.0}) ||
        !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE) ||
        !setGripperCommand(md, {raw, 0.0, 0.0, 0.0}))
    {
        stopDrive(md, "homing release setup failed; reference remains invalid");
        return;
    }
    ImpedanceClose release(raw, gripperOpenPositionRad, gripperOpenVelocityLimitRadS, closeConfig, true);
    release.state = ImpedanceClose::State::TorqueHold;
    m_softCloseJobs.insert_or_assign(md.m_canId, SoftCloseJob{release, std::chrono::steady_clock::now(), true});
    finishHoming(true);
    RCLCPP_INFO(get_logger(), "Drive %d homed to zero; released at 0 Nm; normal limits restored", md.m_canId);
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
    if (!closeConfig.withinTravel(*position))
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
    const double direction = (gripperClosedPositionRad > gripperOpenPositionRad ? 1.0 : -1.0) *
                             (opening ? -1.0 : 1.0);
    ImpedanceClose control(*position, opening ? gripperOpenPositionRad : gripperClosedPositionRad,
                           speed, closeConfig, opening, direction);
    const auto hold = control.command();
    if ((!alreadyReady && (!prepareDriveDisabled(md) ||
                          md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK)) ||
        !setGripperCommand(md, hold) ||
        !configureGripper(md, hold.kp, hold.kd, speed, torqueLimit) ||
        (!alreadyReady && !candle_ros2::enableDrive(md, mab::MdMode_E::IMPEDANCE)) ||
        !setGripperCommand(md, hold))
    {
        stopDrive(md, "impedance motion setup failed; explicit enable required");
        return false;
    }

    m_softCloseJobs.insert_or_assign(md.m_canId, SoftCloseJob{
        control, std::chrono::steady_clock::now(), opening});
    RCLCPP_INFO(this->get_logger(), "Drive %d %s in impedance: ramp %.3f rad/s, "
                "lead %.4f rad, torque limit %.3f Nm, Ki %.3f", md.m_canId,
                opening ? "opening" : "closing", speed,
                opening ? closeConfig.openLead : closeConfig.lead,
                torqueLimit, opening ? 0.0 : closeConfig.ki);
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
    if (!closeConfig.withinTravel(position))
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
        if (homingRequired && candle_ros2::driveDisabled(md.m_mdRegisters.state.value) &&
            md.m_mdRegisters.motionModeStatus.value == mab::MdMode_E::IDLE)
            beginRecovery(md.m_canId); // A brief power loss may occur between telemetry ticks.
        else
            stopDrive(md, reason);
        return;
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
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "MODE %s not recognized, setting IDLE for drive with ID: %d",
                        reqMode.c_str(),
                        req->device_ids[i]);
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
        m_autoHomePending.erase(id); // Explicit stop cancels an automatic restart.
        auto md = findMd(m_mds, id);
        if (md != m_mds.end())
            stopDrive(*md, "disabled by request; explicit enable required");
    }
    m_disableRequests.push_back({std::move(header), req->device_ids});
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
