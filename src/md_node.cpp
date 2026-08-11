#include "candle_ros2/md_node.hpp"
#include "candle_ros2/drive_health.hpp"
#include "candle_ros2/gripper_state.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace
{
template <typename BitsT>
bool appendActiveFlags(const std::unordered_map<BitsT, mab::MDStatus::StatusItem_S>& status,
                       const char*                                                    category,
                       std::string&                                                   description)
{
    bool anyError = false;
    for (const auto& [bit, item] : status)
    {
        (void)bit;
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

MdNode::MdNode(const rclcpp::NodeOptions&   options,
               std::shared_ptr<mab::Candle> candle,
               const candleParams_S&        params)
    : Node("candle_md_node", options),
      m_candle(std::move(candle)),
      m_positionStateStore(params.position_state_file),
      m_lastStateWrite(std::chrono::steady_clock::now()),
      jointNamePrefix(params.joint_name_prefix),
      gripperOpenPositionRad(params.gripper_open_position_rad),
      gripperClosedPositionRad(params.gripper_closed_position_rad),
      gripperImpedanceKp(static_cast<float>(params.gripper_impedance_kp)),
      gripperImpedanceKd(static_cast<float>(params.gripper_impedance_kd)),
      gripperVelocityLimitRadS(static_cast<float>(params.gripper_velocity_limit_rad_s)),
      gripperTorqueLimitNm(static_cast<float>(params.gripper_torque_limit_nm)),
      softCloseFastTorqueLimitNm(static_cast<float>(params.soft_close_fast_torque_limit_nm)),
      softCloseSlowTorqueLimitNm(static_cast<float>(params.soft_close_slow_torque_limit_nm)),
      softCloseProfileAccelerationRadS2(
          static_cast<float>(params.soft_close_profile_acceleration_rad_s2)),
      softCloseProfileDecelerationRadS2(
          static_cast<float>(params.soft_close_profile_deceleration_rad_s2)),
      softCloseClosedTolRad(params.soft_close_closed_tol_rad),
      softCloseFastDurationMs(params.soft_close_fast_duration_ms),
      healthPublishPeriodMs(params.health_publish_period_ms),
      gripperStatePositionToleranceRad(params.gripper_state_position_tolerance_rad),
      gripperStateMovingVelocityRadS(params.gripper_state_moving_velocity_rad_s),
      encoderWrapPeriodRad(params.encoder_wrap_period_rad),
      positionRecoveryMaxDeltaRad(params.position_recovery_max_delta_rad),
      positionRecoverySamples(params.position_recovery_samples),
      positionRecoveryRetryMs(params.position_recovery_retry_ms),
      startupPositionPolicy(params.startup_position_policy),
      positionStateWritePeriodMs(params.position_state_write_period_ms),
      positionStateMinChangeRad(params.position_state_min_change_rad),
      homingTorqueNm(params.homing_torque_nm),
      homingSecondPassTorqueNm(params.homing_second_pass_torque_nm),
      homingStopVerificationTorqueNm(params.homing_stop_verification_torque_nm),
      homingStopVerificationRampMs(params.homing_stop_verification_ramp_ms),
      homingTorqueRampMs(params.homing_torque_ramp_ms),
      homingVelocityTripRadS(params.homing_velocity_trip_rad_s),
      homingMinBusVoltageV(params.homing_min_bus_voltage_v),
      homingStallVelocityRadS(params.homing_stall_velocity_rad_s),
      homingStallDwellMs(params.homing_stall_dwell_ms),
      homingMaxTravelRad(params.homing_max_travel_rad),
      homingTimeoutMs(params.homing_timeout_ms),
      homingBackoffRad(params.homing_backoff_rad),
      homingRepeatabilityRad(params.homing_repeatability_rad),
      homingMinMotionRad(params.homing_min_motion_rad),
      initDevicesZero(params.init_devices_zero)
{
    if (jointNamePrefix.empty())
        throw std::invalid_argument("joint_name_prefix must not be empty");
    if (!std::isfinite(gripperOpenPositionRad) || !std::isfinite(gripperClosedPositionRad) ||
        !std::isfinite(gripperImpedanceKp) || gripperImpedanceKp < 0.0f ||
        !std::isfinite(gripperImpedanceKd) || gripperImpedanceKd < 0.0f ||
        !std::isfinite(gripperVelocityLimitRadS) || gripperVelocityLimitRadS <= 0.0f ||
        !std::isfinite(gripperTorqueLimitNm) || gripperTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseFastTorqueLimitNm) || softCloseFastTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseSlowTorqueLimitNm) || softCloseSlowTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseProfileAccelerationRadS2) ||
        softCloseProfileAccelerationRadS2 <= 0.0f ||
        !std::isfinite(softCloseProfileDecelerationRadS2) ||
        softCloseProfileDecelerationRadS2 <= 0.0f ||
        !std::isfinite(softCloseClosedTolRad) || softCloseClosedTolRad <= 0.0 ||
        softCloseFastDurationMs <= 0 || healthPublishPeriodMs <= 0 ||
        !std::isfinite(gripperStatePositionToleranceRad) ||
        gripperStatePositionToleranceRad <= 0.0 ||
        !std::isfinite(gripperStateMovingVelocityRadS) ||
        gripperStateMovingVelocityRadS <= 0.0 ||
        !std::isfinite(encoderWrapPeriodRad) ||
        encoderWrapPeriodRad <= 0.0 || !std::isfinite(positionRecoveryMaxDeltaRad) ||
        positionRecoveryMaxDeltaRad <= 0.0 ||
        positionRecoveryMaxDeltaRad >= encoderWrapPeriodRad / 2.0 ||
        positionRecoverySamples <= 0 || positionRecoveryRetryMs <= 0 ||
        (startupPositionPolicy != "restore_or_home" &&
         startupPositionPolicy != "restore_only" && startupPositionPolicy != "always_home") ||
        positionStateWritePeriodMs <= 0 || !std::isfinite(positionStateMinChangeRad) ||
        positionStateMinChangeRad < 0.0 || !std::isfinite(homingTorqueNm) ||
        homingTorqueNm <= 0.0 || !std::isfinite(homingSecondPassTorqueNm) ||
        homingSecondPassTorqueNm <= 0.0 ||
        homingSecondPassTorqueNm > homingTorqueNm || homingTorqueRampMs <= 0 ||
        !std::isfinite(homingStopVerificationTorqueNm) ||
        homingStopVerificationTorqueNm < homingTorqueNm ||
        homingStopVerificationTorqueNm > gripperTorqueLimitNm ||
        homingStopVerificationRampMs <= 0 ||
        !std::isfinite(homingVelocityTripRadS) || homingVelocityTripRadS <= 0.0 ||
        !std::isfinite(homingMinBusVoltageV) || homingMinBusVoltageV <= 0.0 ||
        !std::isfinite(homingStallVelocityRadS) || homingStallVelocityRadS < 0.0 ||
        homingStallVelocityRadS >= homingVelocityTripRadS ||
        homingStallDwellMs <= 0 || !std::isfinite(homingMaxTravelRad) ||
        homingMaxTravelRad <= 0.0 || homingTimeoutMs <= 0 ||
        !std::isfinite(homingBackoffRad) || homingBackoffRad <= 0.0 ||
        homingBackoffRad >= homingMaxTravelRad ||
        !std::isfinite(homingRepeatabilityRad) || homingRepeatabilityRad <= 0.0 ||
        homingRepeatabilityRad >= homingBackoffRad ||
        !std::isfinite(homingMinMotionRad) || homingMinMotionRad <= 0.0 ||
        homingMinMotionRad >= homingMaxTravelRad)
        throw std::invalid_argument(
            "invalid gripper position, gain, velocity, torque, or soft-close parameter");

    parseHomingDirections(params.homing_direction_by_id);
    std::string stateLoadError;
    if (!m_positionStateStore.load(&stateLoadError))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Position state file %s is invalid: %s; startup restore disabled until "
                    "homing succeeds",
                    m_positionStateStore.path().c_str(),
                    stateLoadError.c_str());
    }

    rclcpp::QoS defaultQoS(10);
    defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        defaultQoS.best_effort();

    pubJointState = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(NODE_PREFIX) + "joint_states", defaultQoS);
    pubHealth = this->create_publisher<candle_ros2::msg::MdHealth>(
        std::string(NODE_PREFIX) + "health", defaultQoS);
    pubGripperState = this->create_publisher<candle_ros2::msg::GripperState>(
        std::string(NODE_PREFIX) + "gripper_state", defaultQoS);

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

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(PUB_TIMER_MS),
                                     std::bind(&MdNode::publishJointStates, this));
    tmrHealth = this->create_wall_timer(std::chrono::milliseconds(healthPublishPeriodMs),
                                        std::bind(&MdNode::publishHealth, this));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node started.");
}

MdNode::~MdNode()
{
    if (m_activeHoming.has_value())
    {
        auto md = findMd(m_mds, m_activeHoming->first);
        if (md != m_mds.end())
        {
            writeTargetTorque(*md, 0.0);
            md->disable();
        }
    }
    maybePersistPositionState(true);
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::publishJointStates()
{
    tickHoming();
    tickRecoveryJobs();
    tickSoftCloseJobs();

    sensor_msgs::msg::JointState msgJointStates;

    msgJointStates.name.reserve(m_mds.size());
    msgJointStates.position.reserve(m_mds.size());
    msgJointStates.velocity.reserve(m_mds.size());
    msgJointStates.effort.reserve(m_mds.size());

    msgJointStates.header.stamp = this->get_clock()->now();
    for (auto& md : m_mds)
    {
        msgJointStates.name.push_back(jointNamePrefix + std::to_string(md.m_canId));
        const auto logicalPosition = readLogicalPosition(md);
        if (!logicalPosition.has_value())
        {
            m_gripperSamples.erase(md.m_canId);
            const double nan = std::numeric_limits<double>::quiet_NaN();
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        const auto [velocity, velocityErr] = md.getVelocity();
        const auto [torque, torqueErr]     = md.getTorque();
        if (velocityErr != mab::MD::Error_t::OK || torqueErr != mab::MD::Error_t::OK)
        {
            beginRecovery(md.m_canId);
            m_gripperSamples.erase(md.m_canId);
            const double nan = std::numeric_limits<double>::quiet_NaN();
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        m_gripperSamples[md.m_canId] =
            GripperSample{*logicalPosition, static_cast<double>(velocity)};
        msgJointStates.position.push_back(*logicalPosition);
        msgJointStates.velocity.push_back(velocity);
        msgJointStates.effort.push_back(torque);
    }
    this->pubJointState->publish(msgJointStates);
    maybePersistPositionState();
    return;
}

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

        const auto startupIt = m_startupStates.find(md.m_canId);
        const auto trackerIt = m_positionTrackers.find(md.m_canId);
        const bool trackerFaulted =
            trackerIt != m_positionTrackers.end() &&
            trackerIt->second.state() == PositionTracker::State::Faulted;
        const bool trackerRecovering =
            m_recoveryContexts.find(md.m_canId) != m_recoveryContexts.end() ||
            (trackerIt != m_positionTrackers.end() &&
             trackerIt->second.state() == PositionTracker::State::Recovering);

        const char* stateName =
            startupIt == m_startupStates.end()
                ? "UNINITIALIZED"
                : startupStateName(startupIt->second);
        if (trackerFaulted)
            stateName = "FAULTED";
        else if (trackerRecovering)
            stateName = "RECOVERING";
        const StartupHealth startupHealth = startupHealthForState(stateName);

        const auto [quickStatus, statusErr] = md.getQuickStatus();
        if (statusErr != mab::MD::Error_t::OK)
        {
            msg.responsive.push_back(false);
            msg.error.push_back(true);
            std::string description = "drive did not answer the status query";
            if (!startupHealth.description.empty())
                description += "; " + startupHealth.description;
            msg.active_errors.push_back(std::move(description));
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 5000,
                                 "Health: failed to read quick status for drive %d",
                                 md.m_canId);
            continue;
        }

        using Bits = mab::MDStatus::QuickStatusBits;
        bool anyError = startupHealth.error;
        std::string description = startupHealth.description;

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

    pubHealth->publish(msg);
    publishGripperStates();
}

void MdNode::publishGripperStates()
{
    candle_ros2::msg::GripperState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.device_ids.reserve(m_mds.size());
    msg.states.reserve(m_mds.size());

    for (const auto& md : m_mds)
    {
        const auto startupIt = m_startupStates.find(md.m_canId);
        const auto trackerIt = m_positionTrackers.find(md.m_canId);
        const auto sampleIt = m_gripperSamples.find(md.m_canId);

        const bool homing =
            startupIt != m_startupStates.end() &&
            startupIt->second == DriveStartupState::Homing;
        const bool sampleAvailable =
            startupIt != m_startupStates.end() &&
            startupIt->second == DriveStartupState::Tracking &&
            trackerIt != m_positionTrackers.end() && trackerIt->second.isTracking() &&
            m_recoveryContexts.find(md.m_canId) == m_recoveryContexts.end() &&
            sampleIt != m_gripperSamples.end();
        const double position =
            sampleIt == m_gripperSamples.end() ? 0.0 : sampleIt->second.position;
        const double velocity =
            sampleIt == m_gripperSamples.end() ? 0.0 : sampleIt->second.velocity;

        const auto state = classifyGripperState(homing,
                                                sampleAvailable,
                                                position,
                                                velocity,
                                                gripperOpenPositionRad,
                                                gripperClosedPositionRad,
                                                gripperStatePositionToleranceRad,
                                                gripperStateMovingVelocityRadS);
        msg.device_ids.push_back(md.m_canId);
        msg.states.emplace_back(gripperStateName(state));
    }

    pubGripperState->publish(msg);
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
        else
        {
            rememberResumeCommand(msg.device_ids[i],
                                  msg.target_position[i],
                                  false,
                                  gripperVelocityLimitRadS);
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
        md.m_timeout = 10;  // ms
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
        m_startupStates[id] = DriveStartupState::Uninitialized;
        if (const auto persisted = m_positionStateStore.get(id); persisted.has_value())
            m_calibrationGenerations[id] = persisted->calibrationGeneration;
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

        bool restoredPosition = false;
        if (initDevicesZero &&
            (md->zero() != mab::MD::Error_t::OK || md->save() != mab::MD::Error_t::OK))
        {
            RCLCPP_WARN(
                this->get_logger(), "Init devices: failed to zero drive %d", req->device_ids[i]);
            continue;
        }
        if (initDevicesZero)
        {
            m_positionTrackers.at(req->device_ids[i]).resetAtZero();
            m_startupStates[req->device_ids[i]] = DriveStartupState::Tracking;
            ++m_calibrationGenerations[req->device_ids[i]];
        }
        else if (m_startupStates[req->device_ids[i]] != DriveStartupState::Tracking)
        {
            m_startupStates[req->device_ids[i]] = DriveStartupState::Restoring;
            restoredPosition =
                startupPositionPolicy != "always_home" && tryRestorePosition(*md);
            if (!restoredPosition)
            {
                if (startupPositionPolicy == "restore_only")
                {
                    m_startupStates[req->device_ids[i]] = DriveStartupState::Faulted;
                    RCLCPP_ERROR(this->get_logger(),
                                 "Init devices: drive %d has no valid persisted position; "
                                 "manual /md/home is required",
                                 req->device_ids[i]);
                }
                else
                {
                    queueHoming(req->device_ids[i]);
                }
                continue;
            }
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

        const auto currentPos = readLogicalPosition(*md, false);
        const double holdPos =
            currentPos.has_value() ? *currentPos
                                   : std::numeric_limits<double>::quiet_NaN();
        if (!std::isfinite(holdPos) || !setGripperTarget(*md, holdPos))
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
            !setGripperTarget(*md, holdPos))
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
        rememberResumeCommand(req->device_ids[i], holdPos, false, gripperVelocityLimitRadS);
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

        if (m_activeHoming.has_value() && m_activeHoming->first == id)
            abortHoming(*md, "manual zero requested");
        m_homingQueue.erase(
            std::remove(m_homingQueue.begin(), m_homingQueue.end(), id),
            m_homingQueue.end());
        cancelSoftClose(id);

        if (md->zero() != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to zero drive with ID: %d", id);
            rsp->success.push_back(false);
            continue;
        }

        m_recoveryContexts.erase(id);
        m_positionTrackers.at(id).resetAtZero();
        m_startupStates[id] = DriveStartupState::Tracking;
        ++m_calibrationGenerations[id];

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

        rememberResumeCommand(id, 0.0, false, gripperVelocityLimitRadS);
        maybePersistPositionState(true);
        RCLCPP_INFO(this->get_logger(), "Drive %d zeroed and holding at logical 0 rad", id);
        rsp->success.push_back(true);
    }
    return;
}

void MdNode::cbHome(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                    std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());
    for (const auto id : req->device_ids)
        rsp->success.push_back(queueHoming(id));
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
    mab::MD& md,
    double   kp,
    double   kd,
    double   velocityLimit,
    double   torqueLimit)
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

    mab::MDRegisters_S profileRegs;
    profileRegs.profileVelocity = static_cast<float>(velocityLimit);
    if (md.writeRegisters(profileRegs.profileVelocity) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set profile velocity for drive with ID: %d",
                    md.m_canId);
        return false;
    }

    return true;
}

bool MdNode::profilePidReady(mab::MD& md)
{
    mab::MDRegisters_S regs;
    if (md.readRegisters(regs.motorPosPidKp,
                         regs.motorPosPidKi,
                         regs.motorPosPidKd,
                         regs.motorPosPidWindup) != mab::MD::Error_t::OK ||
        md.readRegisters(regs.motorVelPidKp,
                         regs.motorVelPidKi,
                         regs.motorVelPidKd,
                         regs.motorVelPidWindup) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: failed to read profile PID gains for drive %d",
                    md.m_canId);
        return false;
    }

    const bool finite =
        std::isfinite(regs.motorPosPidKp.value) && std::isfinite(regs.motorPosPidKi.value) &&
        std::isfinite(regs.motorPosPidKd.value) && std::isfinite(regs.motorPosPidWindup.value) &&
        std::isfinite(regs.motorVelPidKp.value) && std::isfinite(regs.motorVelPidKi.value) &&
        std::isfinite(regs.motorVelPidKd.value) && std::isfinite(regs.motorVelPidWindup.value);
    if (!finite || regs.motorPosPidKp.value <= 0.0f || regs.motorVelPidKp.value <= 0.0f)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: POSITION_PROFILE PID is not configured for drive %d "
                    "(position kp=%.3f, velocity kp=%.3f)",
                    md.m_canId,
                    regs.motorPosPidKp.value,
                    regs.motorVelPidKp.value);
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Soft-close PID drive %d: position[kp=%.3f ki=%.3f kd=%.3f windup=%.3f] "
                "velocity[kp=%.3f ki=%.3f kd=%.3f windup=%.3f]",
                md.m_canId,
                regs.motorPosPidKp.value,
                regs.motorPosPidKi.value,
                regs.motorPosPidKd.value,
                regs.motorPosPidWindup.value,
                regs.motorVelPidKp.value,
                regs.motorVelPidKi.value,
                regs.motorVelPidKd.value,
                regs.motorVelPidWindup.value);

    return true;
}

bool MdNode::configurePositionProfile(mab::MD& md, double velocityLimit, double torqueLimit)
{
    auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking())
        return false;

    mab::MDRegisters_S regs;
    regs.maxTorque = static_cast<float>(torqueLimit);
    if (md.writeRegisters(regs.maxTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: failed to set torque limit for drive %d",
                    md.m_canId);
        return false;
    }

    regs.maxAcceleration = softCloseProfileAccelerationRadS2;
    regs.maxDeceleration = softCloseProfileDecelerationRadS2;
    if (md.writeRegisters(regs.maxAcceleration, regs.maxDeceleration) !=
        mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: failed to set global profile limits for drive %d",
                    md.m_canId);
        return false;
    }

    regs.profileVelocity     = static_cast<float>(velocityLimit);
    regs.profileAcceleration = softCloseProfileAccelerationRadS2;
    regs.profileDeceleration = softCloseProfileDecelerationRadS2;
    regs.positionWindow      = static_cast<float>(softCloseClosedTolRad);
    if (md.writeRegisters(regs.profileVelocity,
                          regs.profileAcceleration,
                          regs.profileDeceleration,
                          regs.positionWindow) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: failed to configure position profile for drive %d",
                    md.m_canId);
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

bool MdNode::restoreNormalGripperConfig(mab::MD& md)
{
    return md.setMotionMode(mab::MdMode_E::IMPEDANCE) == mab::MD::Error_t::OK &&
           configureGripper(md,
                            gripperImpedanceKp,
                            gripperImpedanceKd,
                            gripperVelocityLimitRadS,
                            gripperTorqueLimitNm);
}

bool MdNode::resetDriveErrorsIfNeeded(mab::MD& md)
{
    const auto [quickStatus, statusErr] = md.getQuickStatus();
    using Bits = mab::MDStatus::QuickStatusBits;
    const bool statusUnavailable = statusErr != mab::MD::Error_t::OK;
    using MotionBits = mab::MDStatus::MotionStatusBits;
    bool motionError = false;
    if (!statusUnavailable && quickStatus.at(Bits::MotionStatus).isSet())
    {
        const auto [motionStatus, motionErr] = md.getMotionStatus();
        if (motionErr != mab::MD::Error_t::OK)
        {
            beginRecovery(md.m_canId);
            return false;
        }
        motionError =
            motionStatus.at(MotionBits::ErrorPositionLimit).isSet() ||
            motionStatus.at(MotionBits::ErrorVelocityLimit).isSet();
    }
    const bool inError =
        !statusUnavailable &&
        (quickStatus.at(Bits::MainEncoderStatus).isSet() ||
         quickStatus.at(Bits::OutputEncoderStatus).isSet() ||
         quickStatus.at(Bits::CalibrationEncoderStatus).isSet() ||
         quickStatus.at(Bits::MosfetBridgeStatus).isSet() ||
         quickStatus.at(Bits::HardwareStatus).isSet() ||
         motionError);
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
        const auto [hardwareStatus, hardwareErr] = md.getHardwareStatus();
        const auto [motionStatus, motionErr]     = md.getMotionStatus();
        if (hardwareErr == mab::MD::Error_t::OK &&
            motionErr == mab::MD::Error_t::OK)
        {
            using HardwareBits = mab::MDStatus::HardwareStatusBits;
            using MotionBits   = mab::MDStatus::MotionStatusBits;
            RCLCPP_WARN(
                this->get_logger(),
                "Drive %d fault details: undervoltage=%d overcurrent=%d "
                "position_limit=%d velocity_limit=%d",
                md.m_canId,
                hardwareStatus.at(HardwareBits::ErrorUnderVoltage).isSet(),
                hardwareStatus.at(HardwareBits::ErrorOverCurrent).isSet(),
                motionStatus.at(MotionBits::ErrorPositionLimit).isSet(),
                motionStatus.at(MotionBits::ErrorVelocityLimit).isSet());
        }
    }

    if (md.clearErrors() != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to clear errors for drive %d", md.m_canId);
        return false;
    }

    const auto [statusAfterClear, statusAfterClearErr] = md.getQuickStatus();
    bool motionErrorAfterClear = false;
    if (statusAfterClearErr == mab::MD::Error_t::OK &&
        statusAfterClear.at(Bits::MotionStatus).isSet())
    {
        const auto [motionStatus, motionErr] = md.getMotionStatus();
        if (motionErr != mab::MD::Error_t::OK)
        {
            beginRecovery(md.m_canId);
            return false;
        }
        motionErrorAfterClear =
            motionStatus.at(MotionBits::ErrorPositionLimit).isSet() ||
            motionStatus.at(MotionBits::ErrorVelocityLimit).isSet();
    }
    if (statusAfterClearErr != mab::MD::Error_t::OK ||
        statusAfterClear.at(Bits::MainEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::OutputEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::CalibrationEncoderStatus).isSet() ||
        statusAfterClear.at(Bits::MosfetBridgeStatus).isSet() ||
        statusAfterClear.at(Bits::HardwareStatus).isSet() ||
        motionErrorAfterClear)
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
    const auto startupState = m_startupStates.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || startupState == m_startupStates.end() ||
        startupState->second != DriveStartupState::Tracking)
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
    auto tracker = m_positionTrackers.find(id);
    auto startupState = m_startupStates.find(id);
    if (tracker == m_positionTrackers.end() || startupState == m_startupStates.end() ||
        startupState->second != DriveStartupState::Tracking ||
        tracker->second.state() == PositionTracker::State::Faulted)
        return;

    if (tracker->second.state() == PositionTracker::State::Tracking)
    {
        auto softClose = m_softCloseJobs.find(id);
        if (softClose != m_softCloseJobs.end())
        {
            rememberResumeCommand(
                id, gripperClosedPositionRad, true, softClose->second.slowVelocityRadS);
        }
        tracker->second.markCommunicationLost();
        startupState->second = DriveStartupState::Recovering;
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d lost communication; position invalid until automatic recovery",
                    id);
    }

    m_recoveryContexts.try_emplace(
        id, RecoveryContext{std::chrono::steady_clock::now(), false});
}

bool MdNode::resumeAfterRecovery(mab::MD& md)
{
    auto tracker = m_positionTrackers.find(md.m_canId);
    if (tracker == m_positionTrackers.end() || !tracker->second.isTracking())
        return false;

    const auto command = m_resumeCommands.find(md.m_canId);
    const double target =
        command != m_resumeCommands.end() ? command->second.targetPosition
                                          : tracker->second.continuousPosition();
    const bool softClose = command != m_resumeCommands.end() && command->second.softClose;
    const double velocity =
        softClose ? command->second.slowVelocity : gripperVelocityLimitRadS;
    const double torque = softClose ? softCloseSlowTorqueLimitNm : gripperTorqueLimitNm;

    if (md.disable() != mab::MD::Error_t::OK ||
        !configureGripper(md, gripperImpedanceKp, gripperImpedanceKd, velocity, torque) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperTarget(md, target) ||
        md.enable() != mab::MD::Error_t::OK ||
        !setGripperTarget(md, target))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d position recovered but target resume failed",
                    md.m_canId);
        return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Drive %d recovered at logical %.4f rad; resumed target %.4f rad",
                md.m_canId,
                tracker->second.continuousPosition(),
                target);
    return true;
}

bool MdNode::canAcceptPositionCommand(u16 id) const
{
    const auto tracker = m_positionTrackers.find(id);
    const auto startupState = m_startupStates.find(id);
    return tracker != m_positionTrackers.end() && tracker->second.isTracking() &&
           startupState != m_startupStates.end() &&
           startupState->second == DriveStartupState::Tracking &&
           m_recoveryContexts.find(id) == m_recoveryContexts.end();
}

void MdNode::rememberResumeCommand(u16 id, double target, bool softClose, double slowVelocity)
{
    m_resumeCommands[id] = ResumeCommand{target, softClose, slowVelocity};
    auto tracker = m_positionTrackers.find(id);
    if (tracker != m_positionTrackers.end())
        tracker->second.setLastTarget(target);
}

bool MdNode::tryRestorePosition(mab::MD& md)
{
    const auto persisted = m_positionStateStore.get(md.m_canId);
    if (!persisted.has_value() ||
        !isPersistedPositionStateCompatible(
            *persisted,
            encoderWrapPeriodRad,
            std::abs(gripperClosedPositionRad - gripperOpenPositionRad)))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Drive %d has no compatible persisted position state",
                    md.m_canId);
        return false;
    }

    std::optional<double> previousCandidate;
    double                lastRaw       = 0.0;
    double                lastCandidate = 0.0;
    using Bits = mab::MDStatus::QuickStatusBits;
    for (int sample = 0; sample < positionRecoverySamples; ++sample)
    {
        const auto [status, statusErr] = md.getQuickStatus();
        const bool encoderHealthy =
            statusErr == mab::MD::Error_t::OK &&
            !status.at(Bits::MainEncoderStatus).isSet() &&
            !status.at(Bits::OutputEncoderStatus).isSet() &&
            !status.at(Bits::CalibrationEncoderStatus).isSet();
        const auto [rawPosition, rawErr] = md.getPosition();
        if (!encoderHealthy || rawErr != mab::MD::Error_t::OK)
            return false;

        lastRaw = static_cast<double>(rawPosition);
        lastCandidate = PositionTracker::nearestEquivalent(
            lastRaw, persisted->logicalPosition, encoderWrapPeriodRad);
        if (std::abs(lastCandidate - persisted->logicalPosition) >
                positionRecoveryMaxDeltaRad ||
            (previousCandidate.has_value() &&
             std::abs(lastCandidate - *previousCandidate) > 0.02))
            return false;
        previousCandidate = lastCandidate;
    }

    const double rangeMin =
        std::min(gripperOpenPositionRad, gripperClosedPositionRad) - 0.02;
    const double rangeMax =
        std::max(gripperOpenPositionRad, gripperClosedPositionRad) + 0.02;
    if (lastCandidate < rangeMin || lastCandidate > rangeMax)
        return false;

    auto& tracker = m_positionTrackers.at(md.m_canId);
    if (!tracker.restore(lastRaw, lastCandidate, persisted->logicalTarget))
        return false;

    m_startupStates[md.m_canId] = DriveStartupState::Tracking;
    m_calibrationGenerations[md.m_canId] = persisted->calibrationGeneration;
    rememberResumeCommand(
        md.m_canId, persisted->logicalTarget, false, gripperVelocityLimitRadS);
    RCLCPP_INFO(this->get_logger(),
                "Drive %d restored from %s at logical %.4f rad (target %.4f rad)",
                md.m_canId,
                m_positionStateStore.path().c_str(),
                lastCandidate,
                persisted->logicalTarget);
    return true;
}

void MdNode::maybePersistPositionState(bool force)
{
    const auto now = std::chrono::steady_clock::now();
    if (!force &&
        now - m_lastStateWrite <
            std::chrono::milliseconds(positionStateWritePeriodMs))
        return;

    bool changed = false;
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
    for (const auto& [id, tracker] : m_positionTrackers)
    {
        const auto startupState = m_startupStates.find(id);
        if (startupState == m_startupStates.end() ||
            startupState->second != DriveStartupState::Tracking ||
            !tracker.isTracking() || !std::isfinite(tracker.continuousPosition()) ||
            !std::isfinite(tracker.rawPosition()))
            continue;

        const auto lastPersisted = m_lastPersistedLogicalPositions.find(id);
        if (!force && lastPersisted != m_lastPersistedLogicalPositions.end() &&
            std::abs(lastPersisted->second - tracker.continuousPosition()) <
                positionStateMinChangeRad)
            continue;

        const auto target = tracker.lastTarget().value_or(tracker.continuousPosition());
        m_positionStateStore.set(
            id,
            PersistedPositionState{
                tracker.continuousPosition(),
                tracker.rawPosition(),
                target,
                encoderWrapPeriodRad,
                std::abs(gripperClosedPositionRad - gripperOpenPositionRad),
                m_calibrationGenerations[id],
                timestamp,
                m_calibrationGenerations[id] > 0,
            });
        m_lastPersistedLogicalPositions[id] = tracker.continuousPosition();
        changed = true;
    }

    if (changed || force)
    {
        std::string error;
        if (!m_positionStateStore.saveAtomic(&error))
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to persist position state to %s: %s",
                         m_positionStateStore.path().c_str(),
                         error.c_str());
        }
        else
        {
            m_lastStateWrite = now;
        }
    }
}

void MdNode::parseHomingDirections(const std::string& setting)
{
    std::stringstream entries(setting);
    std::string       entry;
    while (std::getline(entries, entry, ','))
    {
        const auto separator = entry.find(':');
        if (separator == std::string::npos)
            throw std::invalid_argument("homing_direction_by_id entries must be ID:+1 or ID:-1");

        const auto id        = std::stoul(entry.substr(0, separator));
        const int  direction = std::stoi(entry.substr(separator + 1));
        if (id > std::numeric_limits<u16>::max() ||
            (direction != -1 && direction != 1))
            throw std::invalid_argument("invalid homing_direction_by_id entry");
        m_homingDirections[static_cast<u16>(id)] = direction;
    }
}

const char* MdNode::startupStateName(DriveStartupState state) const
{
    switch (state)
    {
        case DriveStartupState::Uninitialized:
            return "UNINITIALIZED";
        case DriveStartupState::Restoring:
            return "RESTORING";
        case DriveStartupState::Homing:
            return "HOMING";
        case DriveStartupState::Tracking:
            return "TRACKING";
        case DriveStartupState::Recovering:
            return "RECOVERING";
        case DriveStartupState::Faulted:
            return "FAULTED";
    }
    return "UNKNOWN";
}

bool MdNode::queueHoming(u16 id)
{
    auto md = findMd(m_mds, id);
    if (md == m_mds.end())
        return false;
    if (m_homingDirections.find(id) == m_homingDirections.end())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Drive %d cannot home: no direction in homing_direction_by_id",
                     id);
        m_startupStates[id] = DriveStartupState::Faulted;
        return false;
    }
    if ((m_activeHoming.has_value() && m_activeHoming->first == id) ||
        std::find(m_homingQueue.begin(), m_homingQueue.end(), id) != m_homingQueue.end())
        return true;

    m_softCloseJobs.erase(id);
    m_recoveryContexts.erase(id);
    if (md->disable() != mab::MD::Error_t::OK)
    {
        m_startupStates[id] = DriveStartupState::Faulted;
        return false;
    }
    m_startupStates[id] = DriveStartupState::Homing;
    m_homingQueue.push_back(id);
    RCLCPP_INFO(this->get_logger(), "Drive %d queued for open-stop homing", id);
    return startNextHoming();
}

bool MdNode::startNextHoming()
{
    if (m_activeHoming.has_value() || m_homingQueue.empty())
        return true;

    const u16 id = m_homingQueue.front();
    m_homingQueue.pop_front();
    auto md = findMd(m_mds, id);
    if (md == m_mds.end())
        return false;

    const auto [rawPosition, rawErr] = md->getPosition();
    if (rawErr != mab::MD::Error_t::OK)
    {
        m_startupStates[id] = DriveStartupState::Faulted;
        RCLCPP_ERROR(this->get_logger(),
                     "Drive %d homing could not read its startup position",
                     id);
        return false;
    }

    HomingContext context;
    context.startRawPosition = static_cast<double>(rawPosition);
    context.backoffTargetRawPosition =
        context.startRawPosition -
        static_cast<double>(m_homingDirections.at(id)) * homingBackoffRad;
    m_activeHoming.emplace(id, context);
    auto& activeContext = m_activeHoming->second;
    const double backoffKp = gripperImpedanceKp;
    if (md->disable() != mab::MD::Error_t::OK ||
        !configureGripper(*md,
                          backoffKp,
                          gripperImpedanceKd,
                          homingVelocityTripRadS,
                          gripperTorqueLimitNm) ||
        md->setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !writeRawTarget(*md, activeContext.backoffTargetRawPosition) ||
        md->enable() != mab::MD::Error_t::OK ||
        !writeRawTarget(*md, activeContext.backoffTargetRawPosition))
    {
        abortHoming(*md, "failed to perform initial open-stop backoff");
        return false;
    }
    activeContext.stage      = HomingStage::InitialBackoff;
    activeContext.stageStart = std::chrono::steady_clock::now();
    activeContext.lastBackoffKp = backoffKp;

    RCLCPP_INFO(this->get_logger(),
                "Drive %d homing started toward open with direction %d; "
                "dynamic backoff=%.3f rad effort ramp %.2f->%.2f Nm",
                id,
                m_homingDirections.at(id),
                homingBackoffRad,
                std::min<double>(backoffKp * homingBackoffRad, gripperTorqueLimitNm),
                std::min<double>(2.0, gripperTorqueLimitNm));
    return true;
}

bool MdNode::startTorqueSeek(mab::MD& md, HomingContext& context, HomingStage stage)
{
    const auto [rawPosition, rawErr] = md.getPosition();
    if (rawErr != mab::MD::Error_t::OK ||
        md.disable() != mab::MD::Error_t::OK ||
        !writeTargetTorque(md, 0.0) ||
        md.setMotionMode(mab::MdMode_E::RAW_TORQUE) != mab::MD::Error_t::OK ||
        md.enable() != mab::MD::Error_t::OK ||
        !writeTargetTorque(md, 0.0))
        return false;

    context.stage            = stage;
    context.stageStart       = std::chrono::steady_clock::now();
    context.nextSample       = context.stageStart;
    context.seekState        = HomingSeekState{};
    context.startRawPosition = static_cast<double>(rawPosition);
    context.lastTorqueCommand = 0.0;
    return true;
}

bool MdNode::writeTargetTorque(mab::MD& md, double torqueNm)
{
    mab::MDRegisters_S registers;
    registers.targetTorque = static_cast<float>(torqueNm);
    return md.writeRegisters(registers.targetTorque) == mab::MD::Error_t::OK;
}

void MdNode::tickHoming()
{
    if (!m_activeHoming.has_value())
    {
        startNextHoming();
        return;
    }

    const u16 id = m_activeHoming->first;
    auto md      = findMd(m_mds, id);
    if (md == m_mds.end())
    {
        m_activeHoming.reset();
        m_startupStates[id] = DriveStartupState::Faulted;
        return;
    }
    auto& context = m_activeHoming->second;
    const auto now = std::chrono::steady_clock::now();
    if (now < context.nextSample)
        return;
    context.nextSample = now + std::chrono::milliseconds(20);

    const auto [status, statusErr] = md->getQuickStatus();
    const auto [rawPositionValue, positionErr] = md->getPosition();
    const auto [velocityValue, velocityErr] = md->getVelocity();
    mab::MDRegisters_S powerRegisters;
    const auto voltageErr = md->readRegisters(powerRegisters.dcBusVoltage);
    using Bits = mab::MDStatus::QuickStatusBits;
    bool motionError = false;
    bool motionStatusUnavailable = false;
    if (statusErr == mab::MD::Error_t::OK &&
        status.at(Bits::MotionStatus).isSet())
    {
        const auto [motionStatus, motionErr] = md->getMotionStatus();
        motionStatusUnavailable = motionErr != mab::MD::Error_t::OK;
        if (!motionStatusUnavailable)
        {
            using MotionBits = mab::MDStatus::MotionStatusBits;
            motionError =
                motionStatus.at(MotionBits::ErrorPositionLimit).isSet() ||
                motionStatus.at(MotionBits::ErrorVelocityLimit).isSet();
        }
    }
    const bool statusAvailable = statusErr == mab::MD::Error_t::OK;
    const bool encoderError =
        statusAvailable &&
        (status.at(Bits::MainEncoderStatus).isSet() ||
         status.at(Bits::OutputEncoderStatus).isSet() ||
         status.at(Bits::CalibrationEncoderStatus).isSet());
    const bool bridgeError =
        statusAvailable && status.at(Bits::MosfetBridgeStatus).isSet();
    const bool hardwareError =
        statusAvailable && status.at(Bits::HardwareStatus).isSet();
    const bool criticalError = !statusAvailable || motionStatusUnavailable ||
                               encoderError || bridgeError || hardwareError || motionError;
    if (criticalError || positionErr != mab::MD::Error_t::OK ||
        velocityErr != mab::MD::Error_t::OK ||
        voltageErr != mab::MD::Error_t::OK ||
        !std::isfinite(powerRegisters.dcBusVoltage.value) ||
        powerRegisters.dcBusVoltage.value < homingMinBusVoltageV)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Drive %d homing health: status_comm=%d encoder=%d bridge=%d hardware=%d "
                     "motion_error=%d position_comm=%d velocity_comm=%d voltage_comm=%d "
                     "bus_voltage=%.2f V",
                     id,
                     !statusAvailable,
                     encoderError,
                     bridgeError,
                     hardwareError,
                     motionError,
                     positionErr != mab::MD::Error_t::OK,
                     velocityErr != mab::MD::Error_t::OK,
                     voltageErr != mab::MD::Error_t::OK,
                     static_cast<double>(powerRegisters.dcBusVoltage.value));
        abortHoming(*md, "communication or drive status failure");
        return;
    }

    const double rawPosition = static_cast<double>(rawPositionValue);
    const double velocity    = static_cast<double>(velocityValue);
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - context.stageStart);
    const auto updateBackoffStiffness = [&]()
    {
        constexpr double BACKOFF_MAX_EFFORT_NM = 2.0;
        constexpr double BACKOFF_KP_WRITE_STEP = 1.0;
        constexpr double BACKOFF_RAMP_MS = 1000.0;
        const double initialEffort =
            static_cast<double>(gripperImpedanceKp) * homingBackoffRad;
        const double maximumEffort =
            std::min<double>(BACKOFF_MAX_EFFORT_NM, gripperTorqueLimitNm);
        const double rampFraction =
            std::clamp(static_cast<double>(elapsed.count()) / BACKOFF_RAMP_MS, 0.0, 1.0);
        const double effort =
            initialEffort + (maximumEffort - initialEffort) * rampFraction;
        const double kp = std::max<double>(gripperImpedanceKp, effort / homingBackoffRad);
        if (std::abs(kp - context.lastBackoffKp) >= BACKOFF_KP_WRITE_STEP)
        {
            mab::MDRegisters_S registers;
            registers.motorImpPidKp = static_cast<float>(kp);
            if (md->writeRegisters(registers.motorImpPidKp) != mab::MD::Error_t::OK)
                return false;
            context.lastBackoffKp = kp;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             250,
                             "Drive %d homing backoff: error=%.4f rad velocity=%.4f rad/s "
                             "effort_limit=%.2f Nm kp=%.2f",
                             id,
                             context.backoffTargetRawPosition - rawPosition,
                             velocity,
                             effort,
                             kp);
        return true;
    };
    const auto backoffSettled = [&]()
    {
        constexpr double BACKOFF_SETTLE_VELOCITY_RAD_S  = 0.10;
        constexpr auto   BACKOFF_SETTLE_TIME = std::chrono::milliseconds(100);
        const double requiredClearance =
            std::max(homingMinMotionRad, homingBackoffRad * 0.5);
        const bool settled =
            std::abs(rawPosition - context.startRawPosition) >= requiredClearance &&
            std::abs(velocity) <= BACKOFF_SETTLE_VELOCITY_RAD_S;
        if (!settled)
        {
            context.backoffSettleStart.reset();
            return false;
        }
        if (!context.backoffSettleStart.has_value())
        {
            context.backoffSettleStart = now;
            return false;
        }
        return now - *context.backoffSettleStart >= BACKOFF_SETTLE_TIME;
    };

    if (context.stage == HomingStage::InitialBackoff)
    {
        if (std::abs(velocity) > homingVelocityTripRadS)
        {
            abortHoming(*md,
                        "initial backoff velocity safety limit exceeded: |velocity|=" +
                            std::to_string(std::abs(velocity)) +
                            " rad/s, limit=" + std::to_string(homingVelocityTripRadS));
            return;
        }
        if (elapsed > std::chrono::milliseconds(3000))
        {
            abortHoming(*md, "initial backoff timed out");
            return;
        }
        if (!updateBackoffStiffness())
        {
            abortHoming(*md, "failed to ramp initial backoff effort");
            return;
        }
        if (backoffSettled())
        {
            if (!startTorqueSeek(*md, context, HomingStage::FirstSeek))
                abortHoming(*md, "failed to enter first torque-seek stage");
        }
        return;
    }

    if (context.stage == HomingStage::Backoff)
    {
        if (std::abs(velocity) > homingVelocityTripRadS)
        {
            abortHoming(*md,
                        "second backoff velocity safety limit exceeded: |velocity|=" +
                            std::to_string(std::abs(velocity)) +
                            " rad/s, limit=" + std::to_string(homingVelocityTripRadS));
            return;
        }
        if (elapsed > std::chrono::milliseconds(3000))
        {
            abortHoming(*md, "backoff timed out");
            return;
        }
        if (!updateBackoffStiffness())
        {
            abortHoming(*md, "failed to ramp second backoff effort");
            return;
        }
        if (backoffSettled())
        {
            if (!startTorqueSeek(*md, context, HomingStage::SecondSeek))
                abortHoming(*md, "failed to enter second torque-seek stage");
        }
        return;
    }

    const double passTorque = context.stage == HomingStage::FirstSeek
                                  ? homingTorqueNm
                                  : homingSecondPassTorqueNm;
    const double baseTorqueFraction =
        std::clamp(static_cast<double>(elapsed.count()) /
                       static_cast<double>(homingTorqueRampMs),
                   0.0,
                   1.0);
    constexpr auto SEEK_VELOCITY_GRACE = std::chrono::milliseconds(100);
    const double monitoredVelocity =
        elapsed < SEEK_VELOCITY_GRACE ? 0.0 : velocity;
    const double positionDelta = rawPosition - context.startRawPosition;
    const double verificationFraction =
        updateHomingStopVerification(context.seekState,
                                     positionDelta,
                                     baseTorqueFraction,
                                     elapsed.count(),
                                     homingMinMotionRad,
                                     homingStopVerificationRampMs);
    const double torqueMagnitude =
        passTorque * baseTorqueFraction +
        (homingStopVerificationTorqueNm - passTorque) * verificationFraction;
    const double torqueCommand =
        static_cast<double>(m_homingDirections.at(id)) * torqueMagnitude;
    if (std::abs(torqueCommand - context.lastTorqueCommand) >= 0.005)
    {
        if (!writeTargetTorque(*md, torqueCommand))
        {
            abortHoming(*md, "failed to update homing torque");
            return;
        }
        context.lastTorqueCommand = torqueCommand;
    }

    const auto seekResult =
        updateHomingSeek(context.seekState,
                         positionDelta,
                         monitoredVelocity,
                         verificationFraction,
                         elapsed.count(),
                         homingMinMotionRad,
                         homingMaxTravelRad,
                         homingVelocityTripRadS,
                         homingStallVelocityRadS,
                         homingStallDwellMs,
                         homingTimeoutMs);
    RCLCPP_INFO_THROTTLE(this->get_logger(),
                         *this->get_clock(),
                         500,
                         "Drive %d homing seek: pass=%d delta=%.4f rad velocity=%.4f rad/s "
                         "torque=%.3f Nm verification=%.2f moved=%d",
                         id,
                         context.stage == HomingStage::FirstSeek ? 1 : 2,
                         positionDelta,
                         velocity,
                         torqueCommand,
                         verificationFraction,
                         context.seekState.moved);
    if (seekResult != HomingSeekResult::StopDetected)
    {
        if (seekResult == HomingSeekResult::OverVelocity)
            abortHoming(*md,
                        "homing velocity safety limit exceeded: |velocity|=" +
                            std::to_string(std::abs(velocity)) +
                            " rad/s, limit=" + std::to_string(homingVelocityTripRadS));
        else if (seekResult == HomingSeekResult::MaxTravel)
            abortHoming(*md, "maximum homing travel exceeded");
        else if (seekResult == HomingSeekResult::Timeout)
            abortHoming(*md, "seek timed out");
        return;
    }

    if (!writeTargetTorque(*md, 0.0) || md->disable() != mab::MD::Error_t::OK)
    {
        abortHoming(*md, "could not remove torque at detected stop");
        return;
    }

    if (context.stage == HomingStage::FirstSeek)
    {
        context.firstStopRawPosition = rawPosition;
        context.startRawPosition = rawPosition;
        context.backoffTargetRawPosition =
            rawPosition -
            static_cast<double>(m_homingDirections.at(id)) * homingBackoffRad;
        const double backoffKp = gripperImpedanceKp;
        if (!configureGripper(*md,
                              backoffKp,
                              gripperImpedanceKd,
                              homingVelocityTripRadS,
                              gripperTorqueLimitNm) ||
            md->setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
            !writeRawTarget(*md, context.backoffTargetRawPosition) ||
            md->enable() != mab::MD::Error_t::OK ||
            !writeRawTarget(*md, context.backoffTargetRawPosition))
        {
            abortHoming(*md, "failed to back off from first stop");
            return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "Drive %d homing second dynamic backoff: distance=%.3f rad "
                    "effort ramp %.2f->%.2f Nm",
                    id,
                    homingBackoffRad,
                    std::min<double>(backoffKp * homingBackoffRad, gripperTorqueLimitNm),
                    std::min<double>(2.0, gripperTorqueLimitNm));
        context.stage      = HomingStage::Backoff;
        context.stageStart = now;
        context.seekState  = HomingSeekState{};
        context.lastBackoffKp = backoffKp;
        context.backoffSettleStart.reset();
        return;
    }

    completeHoming(*md, context, rawPosition);
}

void MdNode::completeHoming(mab::MD& md, HomingContext& context, double stopRawPosition)
{
    if (!homingStopsRepeatable(
            context.firstStopRawPosition, stopRawPosition, homingRepeatabilityRad))
    {
        abortHoming(md, "two homing passes did not agree");
        return;
    }

    if (md.zero() != mab::MD::Error_t::OK)
    {
        abortHoming(md, "could not zero the open reference");
        return;
    }

    auto& tracker = m_positionTrackers.at(md.m_canId);
    tracker.resetAtZero();
    m_startupStates[md.m_canId] = DriveStartupState::Tracking;
    ++m_calibrationGenerations[md.m_canId];
    rememberResumeCommand(md.m_canId, 0.0, false, gripperVelocityLimitRadS);

    if (!configureGripper(md,
                          gripperImpedanceKp,
                          gripperImpedanceKd,
                          gripperVelocityLimitRadS,
                          gripperTorqueLimitNm) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        !setGripperTarget(md, 0.0) ||
        md.enable() != mab::MD::Error_t::OK ||
        !setGripperTarget(md, 0.0))
    {
        abortHoming(md, "zero succeeded but normal hold configuration failed");
        return;
    }

    const u16 id = md.m_canId;
    m_gripperSamples[id] = GripperSample{gripperOpenPositionRad, 0.0};
    m_activeHoming.reset();
    maybePersistPositionState(true);
    RCLCPP_INFO(this->get_logger(),
                "Drive %d homing completed; state=%s",
                id,
                startupStateName(m_startupStates[id]));
}

void MdNode::abortHoming(mab::MD& md, const std::string& reason)
{
    writeTargetTorque(md, 0.0);
    md.disable();
    const u16 id = md.m_canId;
    if (auto tracker = m_positionTrackers.find(id); tracker != m_positionTrackers.end())
        tracker->second.markFaulted();
    m_startupStates[id] = DriveStartupState::Faulted;
    m_activeHoming.reset();
    RCLCPP_ERROR(this->get_logger(),
                 "Drive %d homing aborted: %s; state=%s",
                 id,
                 reason.c_str(),
                 startupStateName(m_startupStates[id]));
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
            m_startupStates[id] = DriveStartupState::Tracking;
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
            m_startupStates[id] = DriveStartupState::Faulted;
            faulted.push_back(id);
            continue;
        }
        if (result == PositionTracker::RecoveryResult::Pending)
        {
            context.nextAttempt = now;
            continue;
        }

        if (resumeAfterRecovery(*md))
        {
            m_startupStates[id] = DriveStartupState::Tracking;
            recovered.push_back(id);
        }
        else
        {
            tracker->second.markCommunicationLost();
            m_startupStates[id] = DriveStartupState::Recovering;
            context.errorsCleared = false;
        }
    }

    for (u16 id : recovered)
        m_recoveryContexts.erase(id);
    for (u16 id : faulted)
        m_recoveryContexts.erase(id);
}

void MdNode::cancelSoftClose(u16 id)
{
    auto it = m_softCloseJobs.find(id);
    if (it == m_softCloseJobs.end())
        return;

    auto md = findMd(m_mds, id);
    if (md != m_mds.end())
    {
        const auto position = readLogicalPosition(*md);
        if (position.has_value())
        {
            setGripperTarget(*md, *position);
            restoreNormalGripperConfig(*md);
        }
    }

    m_softCloseJobs.erase(it);
}

double MdNode::fingerGapToMotorPos(double gapMm) const
{
    static constexpr std::array<double, 5> GAPS_MM = {0.0, 15.0, 30.0, 50.0, 90.0};
    static constexpr std::array<double, 5> MOTOR_POS_RAD = {0.63, 0.44, 0.366, 0.228, 0.05};

    const double clampedGapMm = std::clamp(gapMm, GAPS_MM.front(), GAPS_MM.back());
    const auto upper = std::upper_bound(GAPS_MM.begin(), GAPS_MM.end(), clampedGapMm);
    if (upper == GAPS_MM.begin())
        return MOTOR_POS_RAD.front();
    if (upper == GAPS_MM.end())
        return MOTOR_POS_RAD.back();

    const size_t upperIndex = static_cast<size_t>(std::distance(GAPS_MM.begin(), upper));
    const size_t lowerIndex = upperIndex - 1;
    const double fraction =
        (clampedGapMm - GAPS_MM[lowerIndex]) /
        (GAPS_MM[upperIndex] - GAPS_MM[lowerIndex]);

    return MOTOR_POS_RAD[lowerIndex] +
           fraction * (MOTOR_POS_RAD[upperIndex] - MOTOR_POS_RAD[lowerIndex]);
}

double MdNode::normalizedSpeedToRadS(double normalizedSpeed)
{
    return SOFT_CLOSE_MIN_SPEED_RAD_S +
           normalizedSpeed * (SOFT_CLOSE_MAX_SPEED_RAD_S - SOFT_CLOSE_MIN_SPEED_RAD_S);
}

void MdNode::tickSoftCloseJobs()
{
    if (m_softCloseJobs.empty())
        return;

    const auto     now = this->now();
    std::vector<u16> finished;

    for (auto& [id, job] : m_softCloseJobs)
    {
        if (m_recoveryContexts.find(id) != m_recoveryContexts.end())
        {
            finished.push_back(id);
            continue;
        }

        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            finished.push_back(id);
            continue;
        }

        const auto fastDuration =
            rclcpp::Duration(std::chrono::milliseconds(softCloseFastDurationMs));

        if (job.stage != SoftCloseStage::Slow && (now - job.requestStart) >= fastDuration)
        {
            if (md->disable() != mab::MD::Error_t::OK ||
                !configureGripper(*md,
                                  gripperImpedanceKp,
                                  gripperImpedanceKd,
                                  job.slowVelocityRadS,
                                  softCloseSlowTorqueLimitNm) ||
                md->setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
                !setGripperTarget(*md, gripperClosedPositionRad) ||
                md->enable() != mab::MD::Error_t::OK ||
                !setGripperTarget(*md, gripperClosedPositionRad))
            {
                RCLCPP_WARN(this->get_logger(),
                            "Soft-close: failed to start slow stage for drive %d",
                            id);
                if (m_recoveryContexts.find(id) == m_recoveryContexts.end())
                {
                    const auto [status, statusErr] = md->getQuickStatus();
                    (void)status;
                    if (statusErr != mab::MD::Error_t::OK)
                        beginRecovery(id);
                    else
                    {
                        restoreNormalGripperConfig(*md);
                        md->enable();
                    }
                }
                finished.push_back(id);
                continue;
            }

            job.stage          = SoftCloseStage::Slow;
            job.slowStageStart = now;
            RCLCPP_INFO(this->get_logger(),
                        "Soft-close: drive %d entered slow impedance stage after %d ms "
                        "(velocity=%.3f rad/s, torque limit=%.3f Nm)",
                        id,
                        softCloseFastDurationMs,
                        job.slowVelocityRadS,
                        softCloseSlowTorqueLimitNm);
            continue;
        }

        const auto pos = readLogicalPosition(*md);
        if (!pos.has_value())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 1000,
                                 "Soft-close: failed to read position for drive %d",
                                 id);
            finished.push_back(id);
            continue;
        }

        const char* stageName = job.stage == SoftCloseStage::Fast ? "fast" : "slow";
        RCLCPP_INFO_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             500,
                             "Soft-close drive %d: stage=%s pos=%.3f preclose=%.3f closed=%.3f "
                             "torque=%.3f Nm",
                             id,
                             stageName,
                             *pos,
                             job.preClosePos,
                             gripperClosedPositionRad,
                             static_cast<double>(md->getTorque().first));

        if (job.stage == SoftCloseStage::Fast)
            continue;

        const bool slowTimedOut =
            (now - job.slowStageStart) >
            rclcpp::Duration(std::chrono::milliseconds(SLOW_STAGE_TIMEOUT_MS));
        const auto [quickStatus, statusErr] = md->getQuickStatus();
        const bool statusReached =
            statusErr == mab::MD::Error_t::OK &&
            (now - job.slowStageStart) > rclcpp::Duration(std::chrono::milliseconds(20)) &&
            quickStatus.at(mab::MDStatus::QuickStatusBits::TargetPositionReached).isSet();

        if (statusReached ||
            std::abs(*pos - gripperClosedPositionRad) <
                softCloseClosedTolRad ||
            slowTimedOut)
        {
            if (slowTimedOut)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Soft-close slow stage timed out for drive %d",
                            id);
            }

            // Keep IMPEDANCE active so the controller holds the closed target.
            finished.push_back(id);
        }
    }

    for (u16 id : finished)
        m_softCloseJobs.erase(id);
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

        cancelSoftClose(id);
        if (!canAcceptPositionCommand(id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; open rejected",
                        id);
            rsp->success.push_back(false);
            continue;
        }
        rememberResumeCommand(id, gripperOpenPositionRad, false, gripperVelocityLimitRadS);
        if (!resetDriveErrorsIfNeeded(*md))
        {
            rsp->success.push_back(false);
            continue;
        }
        rsp->success.push_back(moveGripper(*md, gripperOpenPositionRad));
    }
}

void MdNode::cbCloseGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
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

        cancelSoftClose(id);
        if (!canAcceptPositionCommand(id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; close rejected",
                        id);
            rsp->success.push_back(false);
            continue;
        }
        rememberResumeCommand(id, gripperClosedPositionRad, false, gripperVelocityLimitRadS);
        if (!resetDriveErrorsIfNeeded(*md))
        {
            rsp->success.push_back(false);
            continue;
        }
        rsp->success.push_back(moveGripper(*md, gripperClosedPositionRad));
    }
}

void MdNode::cbSoftCloseGripper(
    const std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Request> req,
    std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    const double normalizedFastSpeed = static_cast<double>(req->fast_speed);
    const double normalizedSlowSpeed = static_cast<double>(req->slow_speed);
    const bool speedsValid =
        std::isfinite(normalizedFastSpeed) && normalizedFastSpeed >= 0.0 &&
        normalizedFastSpeed <= 1.0 && std::isfinite(normalizedSlowSpeed) &&
        normalizedSlowSpeed >= 0.0 && normalizedSlowSpeed <= 1.0;
    if (!speedsValid)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: fast_speed and slow_speed must be finite values in [0, 1]");
        rsp->success.assign(req->device_ids.size(), false);
        return;
    }

    if (req->pre_close_enabled &&
        (!std::isfinite(req->pre_close_gap_mm) || !std::isfinite(req->pre_close_offset_mm)))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: pre_close_gap_mm and pre_close_offset_mm must be finite when "
                    "pre-close is enabled");
        rsp->success.assign(req->device_ids.size(), false);
        return;
    }

    const double fastVelocityRadS = normalizedSpeedToRadS(normalizedFastSpeed);
    const double slowVelocityRadS = normalizedSpeedToRadS(normalizedSlowSpeed);
    const double effectiveGapMm =
        static_cast<double>(req->pre_close_gap_mm) +
        static_cast<double>(req->pre_close_offset_mm);
    const double fastTarget =
        req->pre_close_enabled ? fingerGapToMotorPos(effectiveGapMm) : gripperClosedPositionRad;
    const auto requestStart = this->now();

    if (req->pre_close_enabled)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Soft-close: pre-close enabled, gap=%.1f mm, offset=%+.1f mm, "
                    "effective gap=%.1f mm -> motor %.3f rad, "
                    "normalized speeds fast=%.3f (%.3f rad/s), slow=%.3f (%.3f rad/s)",
                    req->pre_close_gap_mm,
                    req->pre_close_offset_mm,
                    effectiveGapMm,
                    fastTarget,
                    normalizedFastSpeed,
                    fastVelocityRadS,
                    normalizedSlowSpeed,
                    slowVelocityRadS);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Soft-close: pre-close disabled, direct close at normalized speed %.3f "
                    "(%.3f rad/s)",
                    normalizedFastSpeed,
                    fastVelocityRadS);
    }

    for (auto id : req->device_ids)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            RCLCPP_WARN(this->get_logger(), "Drive with ID: %d is not added!", id);
            rsp->success.push_back(false);
            continue;
        }

        cancelSoftClose(id);
        if (!canAcceptPositionCommand(id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d position is invalid or recovering; soft-close rejected",
                        id);
            rsp->success.push_back(false);
            continue;
        }
        rememberResumeCommand(id, gripperClosedPositionRad, true, slowVelocityRadS);

        if (!resetDriveErrorsIfNeeded(*md) ||
            md->disable() != mab::MD::Error_t::OK ||
            !profilePidReady(*md) ||
            !configurePositionProfile(
                *md, fastVelocityRadS, softCloseFastTorqueLimitNm) ||
            md->setMotionMode(mab::MdMode_E::POSITION_PROFILE) != mab::MD::Error_t::OK ||
            !setGripperTarget(*md, fastTarget) ||
            md->enable() != mab::MD::Error_t::OK ||
            !setGripperTarget(*md, fastTarget))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Soft-close: failed to start fast stage for drive %d",
                        id);
            if (m_recoveryContexts.find(id) == m_recoveryContexts.end())
            {
                const auto [status, statusErr] = md->getQuickStatus();
                (void)status;
                if (statusErr != mab::MD::Error_t::OK)
                    beginRecovery(id);
                else
                {
                    restoreNormalGripperConfig(*md);
                    md->enable();
                }
            }
            rsp->success.push_back(false);
            continue;
        }

        if (req->pre_close_enabled)
        {
            m_softCloseJobs[id] = SoftCloseJob{
                SoftCloseStage::Fast, fastTarget, slowVelocityRadS, requestStart, requestStart};
            RCLCPP_INFO(this->get_logger(),
                        "Soft-close: drive %d entered fast profile "
                        "(velocity=%.3f rad/s, torque limit=%.3f Nm)",
                        id,
                        fastVelocityRadS,
                        softCloseFastTorqueLimitNm);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Soft-close: drive %d started single profile to closed target "
                        "(velocity=%.3f rad/s, torque limit=%.3f Nm)",
                        id,
                        fastVelocityRadS,
                        softCloseFastTorqueLimitNm);
        }
        rsp->success.push_back(true);
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
        rememberResumeCommand(
            req->device_ids[i], target, false, gripperVelocityLimitRadS);
        rsp->success[i] = setGripperTarget(*md, target);
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
        if (!canAcceptPositionCommand(req->device_ids[i]))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d state=%s; mode change rejected",
                        req->device_ids[i],
                        startupStateName(m_startupStates[req->device_ids[i]]));
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
        if (!canAcceptPositionCommand(id))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Drive %d state=%s; enable rejected",
                        id,
                        startupStateName(m_startupStates[id]));
            rsp->success.push_back(false);
            continue;
        }

        if (md->enable() == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
        else
            rsp->success.push_back(false);
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

        if (m_activeHoming.has_value() && m_activeHoming->first == id)
        {
            abortHoming(*md, "disabled by service request");
            rsp->success.push_back(true);
            continue;
        }

        if (md->disable() == mab::MD::Error_t::OK)
        {
            m_softCloseJobs.erase(id);
            rsp->success.push_back(true);
        }
        else
        {
            rsp->success.push_back(false);
        }
    }
    return;
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
