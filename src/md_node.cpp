#include "candle_ros2/md_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

MdNode::MdNode(const rclcpp::NodeOptions&   options,
               std::shared_ptr<mab::Candle> candle,
               const candleParams_S&        params)
    : Node("candle_md_node", options), m_candle(std::move(candle))
{
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
        msgJointStates.name.push_back(std::string("Joint " + std::to_string(md.m_canId)));
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
        clearErrorsIfAny(*md);
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
        mdRegisters.motorVelPidKp     = msg.position_pid[i].kp;
        mdRegisters.motorVelPidKi     = msg.position_pid[i].ki;
        mdRegisters.motorVelPidKd     = msg.position_pid[i].kd;
        mdRegisters.motorVelPidWindup = msg.position_pid[i].i_windup;
        mdRegisters.profileVelocity   = msg.position_pid[i].max_output;
        if (md->writeRegisters(mdRegisters.motorVelPidKp,
                               mdRegisters.motorVelPidKi,
                               mdRegisters.motorVelPidKd,
                               mdRegisters.motorVelPidWindup,
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
        clearErrorsIfAny(*md);
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
        clearErrorsIfAny(*md);
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
        clearErrorsIfAny(*md);
    }
    return;
}

void MdNode::cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                     std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    for (auto id : req->device_ids)
    {
        mab::MD md(id, m_candle.get());
        if (md.init() != mab::MD::Error_t::OK)
        {
            rsp->success.push_back(false);
            continue;
        }

        clearErrorsIfAny(md);
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

    auto addReq  = std::make_shared<candle_ros2::srv::AddDevices::Request>();
    auto addRsp  = std::make_shared<candle_ros2::srv::AddDevices::Response>();
    addReq->device_ids = req->device_ids;
    cbAddMd(addReq, addRsp);

    auto modeReq = std::make_shared<candle_ros2::srv::SetMode::Request>();
    auto modeRsp = std::make_shared<candle_ros2::srv::SetMode::Response>();
    modeReq->device_ids = req->device_ids;
    modeReq->mode.assign(n, req->mode);
    cbSetMode(modeReq, modeRsp);

    auto zeroReq = std::make_shared<candle_ros2::srv::Generic::Request>();
    auto zeroRsp = std::make_shared<candle_ros2::srv::Generic::Response>();
    zeroReq->device_ids = req->device_ids;
    cbZero(zeroReq, zeroRsp);

    auto enableReq = std::make_shared<candle_ros2::srv::Generic::Request>();
    auto enableRsp = std::make_shared<candle_ros2::srv::Generic::Response>();
    enableReq->device_ids = req->device_ids;
    cbEnable(enableReq, enableRsp);

    for (size_t i = 0; i < n; i++)
    {
        const bool added   = i < addRsp->success.size() && addRsp->success[i];
        const bool modeOk  = i < modeRsp->success.size() && modeRsp->success[i];
        const bool zeroed  = i < zeroRsp->success.size() && zeroRsp->success[i];
        const bool enabled = i < enableRsp->success.size() && enableRsp->success[i];
        rsp->success[i]    = added && modeOk && zeroed && enabled;
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

        const bool ok = md->zero() == mab::MD::Error_t::OK;
        clearErrorsIfAny(*md);
        rsp->success.push_back(ok);
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
        const bool ok =
            md->writeRegisters(mdRegisters.profileVelocity, mdRegisters.maxTorque) ==
            mab::MD::Error_t::OK;
        clearErrorsIfAny(*md);
        rsp->success.push_back(ok);
    }
    return;
}

bool MdNode::configureGripper(mab::MD& md)
{
    if (m_gripperConfigured.count(md.m_canId) != 0)
        return true;

    if (md.setMotionMode(mab::MdMode_E::IMPEDANCE) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set IMPEDANCE mode for drive with ID: %d",
                    md.m_canId);
        return false;
    }

    mab::MDRegisters_S regs;
    regs.motorImpPidKp = IMP_KP;
    regs.motorImpPidKd = IMP_KD;
    regs.maxTorque     = IMP_MAX_OUTPUT;
    if (md.writeRegisters(regs.motorImpPidKp, regs.motorImpPidKd, regs.maxTorque) !=
        mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set impedance gains for drive with ID: %d",
                    md.m_canId);
        return false;
    }

    m_gripperConfigured.insert(md.m_canId);
    return true;
}

bool MdNode::writeImpedanceKd(mab::MD& md, float kd)
{
    mab::MDRegisters_S regs;
    regs.motorImpPidKd = kd;
    if (md.writeRegisters(regs.motorImpPidKd) != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set impedance kd for drive with ID: %d",
                    md.m_canId);
        return false;
    }
    return true;
}

void MdNode::cancelSoftClose(u16 id)
{
    auto it = m_softCloseJobs.find(id);
    if (it == m_softCloseJobs.end())
        return;

    auto md = findMd(m_mds, id);
    if (md != m_mds.end())
        writeImpedanceKd(*md, IMP_KD);

    m_softCloseJobs.erase(it);
}

double MdNode::fingerGapToMotorPos(double gapMm)
{
    constexpr double deg2rad = M_PI / 180.0;

    const double distMin =
        AXIS_SPACING_MM - 2.0 * FINGER_LENGTH_MM * std::cos(KAT_MIN_DEG * deg2rad);
    const double distMax =
        AXIS_SPACING_MM - 2.0 * FINGER_LENGTH_MM * std::cos(KAT_MAX_DEG * deg2rad);

    double angleDeg = 0.0;
    if (gapMm < distMin)
    {
        angleDeg = KAT_MIN_DEG;
    }
    else if (gapMm > distMax)
    {
        angleDeg = KAT_MAX_DEG;
    }
    else
    {
        const double cosVal = (AXIS_SPACING_MM - gapMm) / (2.0 * FINGER_LENGTH_MM);
        const double clamped = std::clamp(cosVal, -1.0, 1.0);
        angleDeg             = std::acos(clamped) / deg2rad;
    }

    return (KAT_MAX_DEG - angleDeg) / (KAT_MAX_DEG - KAT_MIN_DEG) * CLOSED_POS;
}

bool MdNode::moveGripper(mab::MD& md, double targetPos)
{
    // Mode/gains only once — open/close should just update the target (1 CAN write).
    if (!configureGripper(md))
    {
        clearErrorsIfAny(md);
        return false;
    }

    const bool ok = md.setTargetPosition(static_cast<float>(targetPos)) == mab::MD::Error_t::OK;
    if (!ok)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to set target position for drive with ID: %d",
                    md.m_canId);
    }

    clearErrorsIfAny(md);
    return ok;
}

void MdNode::tickSoftCloseJobs()
{
    if (m_softCloseJobs.empty())
        return;

    const auto now = this->now();
    std::vector<u16> finished;

    for (auto& [id, job] : m_softCloseJobs)
    {
        auto md = findMd(m_mds, id);
        if (md == m_mds.end())
        {
            finished.push_back(id);
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

        const bool timedOut =
            (now - job.stageStart) > rclcpp::Duration(std::chrono::milliseconds(STAGE_TIMEOUT_MS));

        if (job.stage == SoftCloseStage::Fast)
        {
            if (std::abs(static_cast<double>(pos) - job.preClosePos) < PRE_CLOSE_TOL_RAD ||
                timedOut)
            {
                if (timedOut)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Soft-close fast stage timed out for drive %d — starting slow stage",
                                id);
                }

                if (!writeImpedanceKd(*md, SLOW_IMP_KD) ||
                    md->setTargetPosition(static_cast<float>(CLOSED_POS)) != mab::MD::Error_t::OK)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Soft-close: failed to start slow stage for drive %d",
                                id);
                    writeImpedanceKd(*md, IMP_KD);
                    clearErrorsIfAny(*md);
                    finished.push_back(id);
                    continue;
                }

                clearErrorsIfAny(*md);
                job.stage      = SoftCloseStage::Slow;
                job.stageStart = now;
            }
        }
        else  // SoftCloseStage::Slow
        {
            if (std::abs(static_cast<double>(pos) - CLOSED_POS) < PRE_CLOSE_TOL_RAD || timedOut)
            {
                if (timedOut)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Soft-close slow stage timed out for drive %d",
                                id);
                }

                writeImpedanceKd(*md, IMP_KD);
                clearErrorsIfAny(*md);
                finished.push_back(id);
            }
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
        rsp->success.push_back(moveGripper(*md, OPEN_POS));
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
        rsp->success.push_back(moveGripper(*md, CLOSED_POS));
    }
}

void MdNode::cbSoftCloseGripper(
    const std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Request> req,
    std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Response>      rsp)
{
    rsp->success.reserve(req->device_ids.size());

    const double preClosePos = fingerGapToMotorPos(static_cast<double>(req->pre_close_gap_mm));

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

        // Ensure default (fast) kd before the rapid approach stage.
        if (!configureGripper(*md) || !writeImpedanceKd(*md, IMP_KD) ||
            md->setTargetPosition(static_cast<float>(preClosePos)) != mab::MD::Error_t::OK)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Soft-close: failed to start fast stage for drive %d",
                        id);
            clearErrorsIfAny(*md);
            rsp->success.push_back(false);
            continue;
        }

        clearErrorsIfAny(*md);
        m_softCloseJobs[id] = SoftCloseJob{SoftCloseStage::Fast, preClosePos, this->now()};
        rsp->success.push_back(true);
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

        const bool ok = md->setMotionMode(mode) == mab::MD::Error_t::OK;
        clearErrorsIfAny(*md);
        rsp->success.push_back(ok);
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

        const bool ok = md->enable() == mab::MD::Error_t::OK;
        clearErrorsIfAny(*md);
        rsp->success.push_back(ok);
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

        const bool ok = md->disable() == mab::MD::Error_t::OK;
        if (ok)
        {
            // MD clears motion mode on disable — force reconfigure on next gripper move
            m_softCloseJobs.erase(id);
            m_gripperConfigured.erase(id);
        }
        clearErrorsIfAny(*md);
        rsp->success.push_back(ok);
    }
    return;
}

void MdNode::clearErrorsIfAny(mab::MD& md)
{
    using QS = mab::MDStatus::QuickStatusBits;

    const auto [status, err] = md.getQuickStatus();
    if (err != mab::MD::Error_t::OK)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to read quick status for drive with ID: %d",
                    md.m_canId);
        return;
    }

    const bool hasFault = status.at(QS::MainEncoderStatus) || status.at(QS::OutputEncoderStatus) ||
                          status.at(QS::CalibrationEncoderStatus) ||
                          status.at(QS::MosfetBridgeStatus) || status.at(QS::HardwareStatus) ||
                          status.at(QS::MotionStatus);

    if (!hasFault)
        return;

    RCLCPP_WARN(this->get_logger(),
                "Drive with ID: %d reported errors — clearing via MD::clearErrors()",
                md.m_canId);

    if (md.clearErrors() != mab::MD::Error_t::OK)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to clear errors on drive with ID: %d",
                     md.m_canId);
    }
}

std::vector<mab::MD>::iterator MdNode::findMd(std::vector<mab::MD>& mds, u16 id)
{
    return std::find_if(mds.begin(), mds.end(), [id](const mab::MD& m) { return m.m_canId == id; });
}
