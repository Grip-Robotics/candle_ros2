#pragma once
#include <cmath>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros2/msg/impedance_cmd.hpp"
#include "candle_ros2/msg/motion_cmd.hpp"
#include "candle_ros2/msg/position_pid_cmd.hpp"
#include "candle_ros2/msg/velocity_pid_cmd.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/* Services */
#include "candle_ros2/srv/add_devices.hpp"
#include "candle_ros2/srv/configure_gripper.hpp"
#include "candle_ros2/srv/generic.hpp"
#include "candle_ros2/srv/init_devices.hpp"
#include "candle_ros2/srv/set_limits.hpp"
#include "candle_ros2/srv/set_gripper_targets.hpp"
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
        SoftCloseStage stage;
        double         preClosePos;
        rclcpp::Time   requestStart;
        rclcpp::Time   slowStageStart;
    };

    std::shared_ptr<mab::Candle> m_candle;
    std::vector<mab::MD>         m_mds;

    static constexpr const char* NODE_PREFIX  = "md/";
    static constexpr int         PUB_TIMER_MS = 5;  // 200 Hz

    static constexpr int SLOW_STAGE_TIMEOUT_MS = 500;

    std::unordered_map<u16, SoftCloseJob> m_softCloseJobs;

    std::string jointNamePrefix;
    double      gripperOpenPositionRad;
    double      gripperClosedPositionRad;
    double      gripperOpenGapMm;
    double      gripperClosedGapMm;
    float       gripperImpedanceKp;
    float       gripperImpedanceKd;
    float       gripperVelocityLimitRadS;
    float       gripperTorqueLimitNm;
    float       softCloseFastVelocityRadS;
    float       softCloseSlowVelocityRadS;
    float       softCloseFastTorqueLimitNm;
    float       softCloseSlowTorqueLimitNm;
    float       softCloseProfileAccelerationRadS2;
    float       softCloseProfileDecelerationRadS2;
    double      softCloseClosedTolRad;
    int         softCloseFastDurationMs;
    bool        initDevicesZero;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointState;

    rclcpp::Subscription<candle_ros2::msg::MotionCmd>::SharedPtr      subMotionCmd;
    rclcpp::Subscription<candle_ros2::msg::PositionPidCmd>::SharedPtr subPositionCmd;
    rclcpp::Subscription<candle_ros2::msg::VelocityPidCmd>::SharedPtr subVelocityCmd;
    rclcpp::Subscription<candle_ros2::msg::ImpedanceCmd>::SharedPtr   subImpedanceCmd;

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr        srvAddMd;
    rclcpp::Service<candle_ros2::srv::InitDevices>::SharedPtr       srvInitDevices;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvZero;
    rclcpp::Service<candle_ros2::srv::SetMode>::SharedPtr           srvSetMode;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvEnable;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvDisable;
    rclcpp::Service<candle_ros2::srv::SetLimits>::SharedPtr         srvSetLimits;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvOpen;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvClose;
    rclcpp::Service<candle_ros2::srv::ConfigureGripper>::SharedPtr  srvConfigureGripper;
    rclcpp::Service<candle_ros2::srv::SetGripperTargets>::SharedPtr srvSetGripperTargets;
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
    void cbSetGripperTargets(
        const std::shared_ptr<candle_ros2::srv::SetGripperTargets::Request> req,
        std::shared_ptr<candle_ros2::srv::SetGripperTargets::Response>      rsp);
    void cbConfigureGripper(const std::shared_ptr<candle_ros2::srv::ConfigureGripper::Request> req,
                            std::shared_ptr<candle_ros2::srv::ConfigureGripper::Response>      rsp);
    void cbSoftCloseGripper(
        const std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Request> req,
        std::shared_ptr<candle_ros2::srv::SoftCloseGripper::Response>      rsp);

    bool configureGripper(
        mab::MD& md, double kp, double kd, double velocityLimit, double torqueLimit);
    bool profilePidReady(mab::MD& md);
    bool configurePositionProfile(mab::MD& md, double velocityLimit, double torqueLimit);
    bool setGripperTarget(mab::MD& md, double targetPos);
    bool moveGripper(mab::MD& md, double targetPos);
    bool restoreNormalGripperConfig(mab::MD& md);
    bool resetDriveErrorsIfNeeded(mab::MD& md);
    void cancelSoftClose(u16 id);

    /** Inverse kinematics: finger gap [mm] → motor position [rad]. */
    double fingerGapToMotorPos(double gapMm) const;

    std::vector<mab::MD>::iterator findMd(std::vector<mab::MD>& mds, u16 id);
};
