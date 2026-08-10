#include "candle_ros2/md_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
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
        softCloseFastDurationMs <= 0)
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
        msgJointStates.position.push_back(md.getPosition().first);
        msgJointStates.velocity.push_back(md.getVelocity().first);
        msgJointStates.effort.push_back(md.getTorque().first);
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

        mab::MDRegisters_S mdRegisters;
        mdRegisters.targetPosition = msg.target_position[i];
        mdRegisters.targetVelocity = msg.target_velocity[i];
        mdRegisters.targetTorque   = msg.target_torque[i];

        if (md->writeRegisters(mdRegisters.targetPosition,
                               mdRegisters.targetVelocity,
                               mdRegisters.targetTorque) != mab::MD::Error_t::OK)
            RCLCPP_WARN(this->get_logger(),
                        "Failed to set Motion Command for drive with ID: %d",
                        msg.device_ids[i]);
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
        if (md.init() != mab::MD::Error_t::OK)
        {
            rsp->success.push_back(false);
            continue;
        }

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

        // A drive may still be enabled with a target latched in non-volatile
        // registers. Disable it before setting either the mode or target.
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

        const auto [currentPos, posErr] = md->getPosition();
        if (posErr != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Init devices: failed to read current position for drive %d",
                        req->device_ids[i]);
            continue;
        }

        const bool configured =
            req->mode != "IMPEDANCE" ||
            configureGripper(*md,
                             gripperImpedanceKp,
                             gripperImpedanceKd,
                             gripperVelocityLimitRadS,
                             gripperTorqueLimitNm);
        if (!configured || !setGripperTarget(*md, static_cast<double>(currentPos)))
            continue;

        auto modeReq        = std::make_shared<candle_ros2::srv::SetMode::Request>();
        auto modeRsp        = std::make_shared<candle_ros2::srv::SetMode::Response>();
        modeReq->device_ids = {req->device_ids[i]};
        modeReq->mode       = {req->mode};
        cbSetMode(modeReq, modeRsp);
        if (modeRsp->success.empty() || !modeRsp->success.front())
            continue;

        rsp->success[i] = md->enable() == mab::MD::Error_t::OK;
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

        if (md->zero() == mab::MD::Error_t::OK)
            rsp->success.push_back(true);
        else
            rsp->success.push_back(false);
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
    mab::MDRegisters_S regs;
    regs.positionLimitMin = static_cast<float>(
        std::min(gripperOpenPositionRad, gripperClosedPositionRad));
    regs.positionLimitMax = static_cast<float>(
        std::max(gripperOpenPositionRad, gripperClosedPositionRad));
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
    mab::MDRegisters_S motionRegs;
    motionRegs.targetPosition = targetPos;
    motionRegs.targetVelocity = 0.0;
    motionRegs.targetTorque   = 0.0;
    if (md.writeRegisters(motionRegs.targetPosition,
                          motionRegs.targetVelocity,
                          motionRegs.targetTorque) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(
            this->get_logger(), "Failed to set target position for drive with ID: %d", md.m_canId);
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
        return false;
    }

    if (!configureGripper(md,
                          gripperImpedanceKp,
                          gripperImpedanceKd,
                          gripperVelocityLimitRadS,
                          gripperTorqueLimitNm) ||
        !setGripperTarget(md, targetPos) ||
        md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
        md.enable() != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to start impedance move for drive with ID: %d",
                    md.m_canId);
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
                    "Failed to read quick status for drive %d; attempting error reset before "
                    "motion command",
                    md.m_canId);
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

    // Do not enable here: the caller must first configure its mode and target,
    // then enable the drive. Error recovery never calls zero().
    return true;
}

void MdNode::cancelSoftClose(u16 id)
{
    auto it = m_softCloseJobs.find(id);
    if (it == m_softCloseJobs.end())
        return;

    auto md = findMd(m_mds, id);
    if (md != m_mds.end())
    {
        const auto [position, err] = md->getPosition();
        if (err == mab::MD::Error_t::OK)
            setGripperTarget(*md, static_cast<double>(position));
        restoreNormalGripperConfig(*md);
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
                !setGripperTarget(*md, gripperClosedPositionRad) ||
                md->setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK ||
                md->enable() != mab::MD::Error_t::OK)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Soft-close: failed to start slow stage for drive %d",
                            id);
                restoreNormalGripperConfig(*md);
                md->enable();
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

        const auto [pos, posErr] = md->getPosition();
        if (posErr != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(),
                                 *this->get_clock(),
                                 1000,
                                 "Soft-close: failed to read position for drive %d",
                                 id);
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
                             static_cast<double>(pos),
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
            std::abs(static_cast<double>(pos) - gripperClosedPositionRad) <
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

        if (!resetDriveErrorsIfNeeded(*md) ||
            md->disable() != mab::MD::Error_t::OK ||
            !profilePidReady(*md) ||
            !configurePositionProfile(
                *md, fastVelocityRadS, softCloseFastTorqueLimitNm) ||
            !setGripperTarget(*md, fastTarget) ||
            md->setMotionMode(mab::MdMode_E::POSITION_PROFILE) != mab::MD::Error_t::OK ||
            md->enable() != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Soft-close: failed to start fast stage for drive %d",
                        id);
            restoreNormalGripperConfig(*md);
            md->enable();
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
