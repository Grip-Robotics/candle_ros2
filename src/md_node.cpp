#include "candle_ros2/md_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
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
      softCloseFastTorqueLimitNm(static_cast<float>(params.soft_close_fast_torque_limit_nm)),
      softCloseSlowTorqueLimitNm(static_cast<float>(params.soft_close_slow_torque_limit_nm)),
      softCloseProfileAccelerationRadS2(
          static_cast<float>(params.soft_close_profile_acceleration_rad_s2)),
      softCloseProfileDecelerationRadS2(
          static_cast<float>(params.soft_close_profile_deceleration_rad_s2)),
      softCloseClosedTolRad(params.soft_close_closed_tol_rad),
      softCloseFastDurationMs(params.soft_close_fast_duration_ms),
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
        !std::isfinite(softCloseFastTorqueLimitNm) || softCloseFastTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseSlowTorqueLimitNm) || softCloseSlowTorqueLimitNm <= 0.0f ||
        !std::isfinite(softCloseProfileAccelerationRadS2) ||
        softCloseProfileAccelerationRadS2 <= 0.0f ||
        !std::isfinite(softCloseProfileDecelerationRadS2) ||
        softCloseProfileDecelerationRadS2 <= 0.0f ||
        !std::isfinite(softCloseClosedTolRad) || softCloseClosedTolRad <= 0.0 ||
        softCloseFastDurationMs <= 0 || !std::isfinite(encoderWrapPeriodRad) ||
        encoderWrapPeriodRad <= 0.0 || !std::isfinite(positionRecoveryMaxDeltaRad) ||
        positionRecoveryMaxDeltaRad <= 0.0 ||
        positionRecoveryMaxDeltaRad >= encoderWrapPeriodRad / 2.0 ||
        positionRecoverySamples <= 0 || positionRecoveryRetryMs <= 0)
        throw std::invalid_argument(
            "invalid gripper position, gain, velocity, torque, or soft-close parameter");

    rclcpp::QoS defaultQoS(10);
    defaultQoS.reliable();

    if (params.default_qos == "BestEffort")
        defaultQoS.best_effort();

    pubJointState = this->create_publisher<sensor_msgs::msg::JointState>(
        std::string(NODE_PREFIX) + "joint_states", defaultQoS);

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

    tmrPub = this->create_wall_timer(std::chrono::milliseconds(PUB_TIMER_MS),
                                     std::bind(&MdNode::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node started.");
}

MdNode::~MdNode()
{
    RCLCPP_INFO(this->get_logger(), "Candle ROS2 MD node finished.");
}

void MdNode::publishJointStates()
{
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
            const double nan = std::numeric_limits<double>::quiet_NaN();
            msgJointStates.position.push_back(nan);
            msgJointStates.velocity.push_back(nan);
            msgJointStates.effort.push_back(nan);
            continue;
        }

        msgJointStates.position.push_back(*logicalPosition);
        msgJointStates.velocity.push_back(velocity);
        msgJointStates.effort.push_back(torque);
    }
    this->pubJointState->publish(msgJointStates);
    return;
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
        rememberResumeCommand(
            req->device_ids[i], *holdPosAfterEnable, false, gripperVelocityLimitRadS);
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

        cancelSoftClose(id);

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

        rememberResumeCommand(id, 0.0, false, gripperVelocityLimitRadS);
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
    const double rawOpen   = tracker->second.logicalToRaw(gripperOpenPositionRad);
    const double rawClosed = tracker->second.logicalToRaw(gripperClosedPositionRad);
    regs.positionLimitMin  = static_cast<float>(std::min(rawOpen, rawClosed));
    regs.positionLimitMax  = static_cast<float>(std::max(rawOpen, rawClosed));
    regs.maxTorque = static_cast<float>(torqueLimit);
    if (md.writeRegisters(regs.positionLimitMin, regs.positionLimitMax, regs.maxTorque) !=
        mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Soft-close: failed to set position/torque limits for drive %d",
                    md.m_canId);
        return false;
    }

    regs.maxVelocity     = static_cast<float>(velocityLimit);
    regs.maxAcceleration = softCloseProfileAccelerationRadS2;
    regs.maxDeceleration = softCloseProfileDecelerationRadS2;
    if (md.writeRegisters(regs.maxVelocity, regs.maxAcceleration, regs.maxDeceleration) !=
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
    auto tracker = m_positionTrackers.find(id);
    if (tracker == m_positionTrackers.end() ||
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
        !configureGripper(
            md, gripperImpedanceKp, gripperImpedanceKd, velocity, torque) ||
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
    return tracker != m_positionTrackers.end() && tracker->second.isTracking() &&
           m_recoveryContexts.find(id) == m_recoveryContexts.end();
}

void MdNode::rememberResumeCommand(u16 id, double target, bool softClose, double slowVelocity)
{
    m_resumeCommands[id] = ResumeCommand{target, softClose, slowVelocity};
    auto tracker = m_positionTrackers.find(id);
    if (tracker != m_positionTrackers.end())
        tracker->second.setLastTarget(target);
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

        if (resumeAfterRecovery(*md))
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
