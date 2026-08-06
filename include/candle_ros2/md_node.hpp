#pragma once
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros2/msg/impedance_cmd.hpp"
#include "candle_ros2/msg/motion_cmd.hpp"
#include "candle_ros2/msg/position_pid_cmd.hpp"
#include "candle_ros2/msg/velocity_pid_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/* Services */
#include "candle_ros2/srv/add_devices.hpp"
#include "candle_ros2/srv/generic.hpp"
#include "candle_ros2/srv/init_devices.hpp"
#include "candle_ros2/srv/set_limits.hpp"
#include "candle_ros2/srv/set_mode.hpp"
#include "candle_ros2/srv/soft_close_gripper.hpp"

/* Utils */
#include "candle_ros2/utils/candle_params.hpp"

/* CANdle-SDK */
#include "candle.hpp"
#include "MD.hpp"
#include "MDStatus.hpp"

class MdNode : public rclcpp::Node
{
  public:
    MdNode(const rclcpp::NodeOptions&   options,
           std::shared_ptr<mab::Candle> candle,
           const candleParams_S&        params);
    ~MdNode();

  private:
    enum class SoftCloseStage
    {
        Fast,
        Slow
    };

    struct SoftCloseJob
    {
        SoftCloseStage  stage;
        double          preClosePos;
        rclcpp::Time    stageStart;
    };

    std::shared_ptr<mab::Candle> m_candle;
    std::vector<mab::MD>         m_mds;

    static constexpr const char* NODE_PREFIX  = "md/";
    static constexpr int         PUB_TIMER_MS = 5;  // 200 Hz

    /* Gripper impedance defaults — tune for hardware */
    static constexpr double OPEN_POS       = 0.0;
    static constexpr double CLOSED_POS     = 0.83;
    static constexpr float  IMP_KP         = 5.0f;
    static constexpr float  IMP_KD         = 0.05f;
    static constexpr float  IMP_MAX_OUTPUT = 3.5f;  // max torque [Nm]
    static constexpr float  SLOW_IMP_KD    = 0.5f;  // higher damping for soft-close stage

    /* Finger geometry [mm] — tune for hardware */
    static constexpr double FINGER_LENGTH_MM = 80.0;
    static constexpr double AXIS_SPACING_MM  = 80.0;
    static constexpr double KAT_MIN_DEG      = 68.0;   // full close
    static constexpr double KAT_MAX_DEG      = 112.0;  // full open

    static constexpr double PRE_CLOSE_TOL_RAD = 0.02;
    static constexpr int    STAGE_TIMEOUT_MS  = 1000;

    /* Drives that already have mode/gains/limits applied for gripper use */
    std::unordered_set<u16> m_gripperConfigured;

    /* Active two-stage soft-close jobs, keyed by CAN id */
    std::unordered_map<u16, SoftCloseJob> m_softCloseJobs;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointState;

    rclcpp::Subscription<candle_ros2::msg::MotionCmd>::SharedPtr      subMotionCmd;
    rclcpp::Subscription<candle_ros2::msg::PositionPidCmd>::SharedPtr subPositionCmd;
    rclcpp::Subscription<candle_ros2::msg::VelocityPidCmd>::SharedPtr subVelocityCmd;
    rclcpp::Subscription<candle_ros2::msg::ImpedanceCmd>::SharedPtr   subImpedanceCmd;

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr       srvAddMd;
    rclcpp::Service<candle_ros2::srv::InitDevices>::SharedPtr      srvInitDevices;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr          srvZero;
    rclcpp::Service<candle_ros2::srv::SetMode>::SharedPtr          srvSetMode;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr          srvEnable;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr          srvDisable;
    rclcpp::Service<candle_ros2::srv::SetLimits>::SharedPtr        srvSetLimits;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr          srvOpen;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr          srvClose;
    rclcpp::Service<candle_ros2::srv::SoftCloseGripper>::SharedPtr srvSoftClose;

    rclcpp::TimerBase::SharedPtr tmrPub;

    void publishJointStates();
    void tickSoftCloseJobs();

    void cbMotionCmd(const candle_ros2::msg::MotionCmd& msg);
    void cbPositionCmd(const candle_ros2::msg::PositionPidCmd& msg);
    void cbVelocityCmd(const candle_ros2::msg::VelocityPidCmd& msg);
    void cbImpedanceCmd(const candle_ros2::msg::ImpedanceCmd& msg);

    void cbAddMd(const std::shared_ptr<candle_ros2::srv::AddDevices::Request> req,
                 std::shared_ptr<candle_ros2::srv::AddDevices::Response>      rsp);
    void cbInitDevices(const std::shared_ptr<candle_ros2::srv::InitDevices::Request> req,
                       std::shared_ptr<candle_ros2::srv::InitDevices::Response>      rsp);
    void cbZero(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbSetMode(const std::shared_ptr<candle_ros2::srv::SetMode::Request> req,
                   std::shared_ptr<candle_ros2::srv::SetMode::Response>      rsp);
    void cbEnable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                  std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbDisable(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                   std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbSetLimits(const std::shared_ptr<candle_ros2::srv::SetLimits::Request> req,
                     std::shared_ptr<candle_ros2::srv::SetLimits::Response>      rsp);
    void cbOpenGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                       std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbCloseGripper(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
                        std::shared_ptr<candle_ros2::srv::Generic::Response>      rsp);
    void cbSoftCloseGripper(
        const std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Request> req,
        std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Response>      rsp);

    bool configureGripper(mab::MD& md);
    bool moveGripper(mab::MD& md, double targetPos);
    bool writeImpedanceKd(mab::MD& md, float kd);
    void cancelSoftClose(u16 id);

    /** Inverse kinematics: finger gap [mm] → motor position [rad]. */
    static double fingerGapToMotorPos(double gapMm);

    /** Read quick status; if any fault category is set, call MD::clearErrors(). */
    void clearErrorsIfAny(mab::MD& md);

    std::vector<mab::MD>::iterator findMd(std::vector<mab::MD>& mds, u16 id);
};
