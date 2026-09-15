#include "candle_ros2/md_node.hpp"

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
      gripperTorqueLimitNm(static_cast<float>(params.gripper_torque_limit_nm)),
      softCloseSlowTorqueLimitNm(static_cast<float>(params.soft_close_slow_torque_limit_nm)),
      closeConfig(params.close),
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
        !std::isfinite(gripperImpedanceKp) || gripperImpedanceKp < 0.0f ||
        !std::isfinite(gripperImpedanceKd) || gripperImpedanceKd < 0.0f ||
        !std::isfinite(gripperVelocityLimitRadS) || gripperVelocityLimitRadS <= 0.0f ||
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
    ImpedanceClose(gripperOpenPositionRad, gripperClosedPositionRad,
                   gripperVelocityLimitRadS, closeConfig);
    if (gripperImpedanceKp <= 0.0f ||
        closeConfig.contactTorque >= std::min<double>(
            {gripperTorqueLimitNm, softCloseSlowTorqueLimitNm,
             gripperImpedanceKp * closeConfig.lead}))
        throw std::invalid_argument("close contact torque must be below the available bounded torque");

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
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::publishJointStates()
{
    tickRecoveryJobs();

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
                             md.m_mdRegisters.motorTorque) != mab::MD::Error_t::OK)
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

        tickSoftClose(md, *logicalPosition, md.m_mdRegisters.mainEncoderVelocity.value,
                      md.m_mdRegisters.motorTorque.value);
        msgJointStates.position.push_back(*logicalPosition);
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

    for (auto& md : m_mds)
    {
        msg.device_ids.push_back(md.m_canId);

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
            if (m_softCloseJobs.contains(md.m_canId))
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

        // Detailed subsystem status is fetched only for categories flagged in the
        // quick status, so a healthy drive costs a single CAN read per cycle.
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
            if (m_softCloseJobs.contains(md.m_canId))
                stopDrive(md, "close aborted: drive health error");
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "Health: drive %d reports errors: %s",
                                 md.m_canId,
                                 description.c_str());
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

        if (m_softCloseJobs.contains(msg.device_ids[i]))
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

        if (m_softCloseJobs.contains(msg.device_ids[i]))
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

        if (m_softCloseJobs.contains(msg.device_ids[i]))
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

        // Reinitialization must not allow a previously scheduled soft-close
        // stage to command this drive again after it is enabled.
        m_softCloseJobs.erase(req->device_ids[i]);
        m_motionInhibited.insert(req->device_ids[i]);

        // Disable first: MD motion mode resets on disable, and a previous
        // close/soft-close target may still be latched until overwritten in
        // the active motion mode.
        if (!resetDriveErrorsIfNeeded(*md) || md->disable() != mab::MD::Error_t::OK)
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

        if (md->enable() != mab::MD::Error_t::OK)
            continue;

        // Reassert hold after enable so a briefly restored stale target cannot
        // start a close motion.
        const auto holdPosAfterEnable = readLogicalPosition(*md, false);
        if (!holdPosAfterEnable.has_value() ||
            !setGripperTarget(*md, *holdPosAfterEnable))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Init devices: failed to reassert hold target for drive %d after enable",
                        req->device_ids[i]);
            md->disable();
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
            md->disable();
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

        if (m_softCloseJobs.contains(req->device_ids[i]))
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
    limitRegs.maxTorque       = static_cast<float>(torqueLimit);
    if (md.writeRegisters(limitRegs.profileVelocity, limitRegs.maxTorque) != mab::MD::Error_t::OK)
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
    if (md.disable() != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to disable drive with ID: %d before impedance move",
                    md.m_canId);
        beginRecovery(md.m_canId);
        return false;
    }

    if (!configureGripper(md,
                          gripperImpedanceKp,
                          gripperImpedanceKd,
                          gripperVelocityLimitRadS,
                          gripperTorqueLimitNm) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperTarget(md, targetPos) ||
        md.enable() != mab::MD::Error_t::OK ||
        !setGripperTarget(md, targetPos))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to start impedance move for drive with ID: %d",
                    md.m_canId);
        beginRecovery(md.m_canId);
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
           !m_motionInhibited.contains(id);
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
        if (now < context.nextAttempt)
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

        if (tracker->second.state() == PositionTracker::State::Uninitialized)
        {
            tracker->second.initialize(static_cast<double>(rawPosition));
            recovered.push_back(id);
            continue;
        }

        const auto result = tracker->second.observeRecovery(static_cast<double>(rawPosition));
        if (result == PositionTracker::RecoveryResult::Rejected)
        {
            md->disable();
            RCLCPP_ERROR(this->get_logger(),
                         "Drive %d recovery is ambiguous; manual open calibration is required",
                         id);
            faulted.push_back(id);
            continue;
        }
        if (result == PositionTracker::RecoveryResult::Pending)
        {
            context.nextAttempt = now;
            continue;
        }

        if (md->disable() == mab::MD::Error_t::OK)
        {
            recovered.push_back(id);
        }
        else
        {
            tracker->second.markCommunicationLost();
            context.errorsCleared = false;
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
    m_commandErrors[md.m_canId] = reason;
    if (md.disable() != mab::MD::Error_t::OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Drive %d disable unconfirmed: %s",
                     md.m_canId, reason.c_str());
        m_positionTrackers.at(md.m_canId).markCommunicationLost();
        m_recoveryContexts.try_emplace(md.m_canId,
            RecoveryContext{std::chrono::steady_clock::now(), false});
    }
}

bool MdNode::cancelSoftClose(u16 id)
{
    if (!m_softCloseJobs.contains(id))
        return true;
    m_softCloseJobs.erase(id);
    auto md = findMd(m_mds, id);
    if (md == m_mds.end())
        return false;
    const auto position = readLogicalPosition(*md);
    if (!position.has_value() || !setGripperTarget(*md, *position))
    {
        stopDrive(*md, "failed to cancel close; explicit enable required");
        return false;
    }
    return true;
}

double MdNode::normalizedSpeedToRadS(double normalizedSpeed)
{
    return SOFT_CLOSE_MIN_SPEED_RAD_S +
           normalizedSpeed * (SOFT_CLOSE_MAX_SPEED_RAD_S - SOFT_CLOSE_MIN_SPEED_RAD_S);
}

bool MdNode::startImpedanceClose(mab::MD& md, double speed)
{
    if (!canAcceptPositionCommand(md.m_canId) || !cancelSoftClose(md.m_canId))
        return false;
    const auto position = readLogicalPosition(md);
    if (!position.has_value())
        return false;
    const double lower = std::min(gripperOpenPositionRad, gripperClosedPositionRad);
    const double upper = std::max(gripperOpenPositionRad, gripperClosedPositionRad);
    if (*position < lower - closeConfig.tolerance || *position > upper + closeConfig.tolerance)
    {
        stopDrive(md, "close rejected: position outside calibrated travel");
        return false;
    }

    const double torqueLimit = std::min(gripperTorqueLimitNm, softCloseSlowTorqueLimitNm);
    // Latch the measured hold position before enable; no closed-endpoint step.
    if (!resetDriveErrorsIfNeeded(md) ||
        md.disable() != mab::MD::Error_t::OK ||
        !configureGripper(md, gripperImpedanceKp, gripperImpedanceKd, speed, torqueLimit) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperTarget(md, *position) ||
        md.enable() != mab::MD::Error_t::OK ||
        !setGripperTarget(md, *position))
    {
        stopDrive(md, "impedance close setup failed; explicit enable required");
        return false;
    }

    m_softCloseJobs.insert_or_assign(md.m_canId, SoftCloseJob{
        ImpedanceClose(*position, gripperClosedPositionRad, speed, closeConfig),
        std::chrono::steady_clock::now()});
    RCLCPP_INFO(this->get_logger(), "Drive %d closing in impedance: ramp %.3f rad/s, "
                "lead %.4f rad, torque limit %.3f Nm", md.m_canId, speed,
                closeConfig.lead, torqueLimit);
    return true;
}

void MdNode::tickSoftClose(mab::MD& md, double position, double velocity, double torque)
{
    auto it = m_softCloseJobs.find(md.m_canId);
    if (it == m_softCloseJobs.end())
        return;
    auto& job = it->second;
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - job.lastUpdate).count();
    job.lastUpdate = now;
    const auto previous = job.control.state;
    const double previousTarget = job.control.target;
    const double lower = std::min(gripperOpenPositionRad, gripperClosedPositionRad);
    const double upper = std::max(gripperOpenPositionRad, gripperClosedPositionRad);
    if (position < lower - closeConfig.tolerance || position > upper + closeConfig.tolerance)
    {
        stopDrive(md, "close failed: measured position outside calibrated travel");
        return;
    }
    const auto state = job.control.update(position, velocity, torque, dt);
    if (state == ImpedanceClose::State::Failed)
    {
        stopDrive(md, "close failed: timeout, stale or invalid feedback; explicit enable required");
        return;
    }
    // writeRawTarget may erase this job on a communication failure.
    const double target = job.control.target;
    if (target != previousTarget && !setGripperTarget(md, target))
        return;
    if (state != previous)
        RCLCPP_INFO(this->get_logger(), "Drive %d close: %s at %.4f rad; holding target %.4f rad",
                    md.m_canId,
                    state == ImpedanceClose::State::Contact ? "contact inferred" : "closed endpoint",
                    position, target);
}

void MdNode::cbOpenGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                           std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", id);
            rsp->success.push_back(false);
            continue;
        }

        if (!cancelSoftClose(id))
        {
            rsp->success.push_back(false);
            continue;
        }
        if (!canAcceptPositionCommand(id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; open rejected",
                        id);
            rsp->success.push_back(false);
            continue;
        }
        if (!resetDriveErrorsIfNeeded(*md))
        {
            rsp->success.push_back(false);
            continue;
        }
        rsp->success.push_back(moveGripper(*md, gripperOpenPositionRad));
    }
}

void MdNode::cbCloseGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                            std::shared_ptr<candle_ros2::srv::Generic::Response> rsp)
{
    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        rsp->success.push_back(md != m_mds.end() &&
                               startImpedanceClose(*md, gripperVelocityLimitRadS));
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
        rsp->success.push_back(md != m_mds.end() && startImpedanceClose(*md, speed));
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
        if (m_softCloseJobs.contains(req->device_ids[i]))
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

        if (!cancelSoftClose(id) || m_recoveryContexts.contains(id))
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

void MdNode::cbDisable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
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

        m_motionInhibited.insert(id);
        m_softCloseJobs.erase(id);
        m_commandErrors[id] = "disabled by request; explicit enable required";
        if (md->disable() == mab::MD::Error_t::OK)
        {
            rsp->success.push_back(true);
        }
        else
        {
            m_positionTrackers.at(id).markCommunicationLost();
            m_recoveryContexts.try_emplace(id,
                RecoveryContext{std::chrono::steady_clock::now(), false});
            rsp->success.push_back(false);
        }
    }
    return;
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
