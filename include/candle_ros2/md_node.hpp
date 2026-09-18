#pragma once
#include <chrono>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"

/* Messages */
#include "candle_ros2/msg/impedance_cmd.hpp"
#include "candle_ros2/msg/md_health.hpp"
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
#include "candle_ros2/position_tracker.hpp"
#include "candle_ros2/impedance_close.hpp"
#include "candle_ros2/drive_state.hpp"

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
    struct SoftCloseJob
    {
        ImpedanceClose control;
        std::chrono::steady_clock::time_point lastUpdate;
        bool opening;
    };

    struct RecoveryContext
    {
        std::chrono::steady_clock::time_point nextAttempt;
        bool                                  errorsCleared = false;
    };

    struct HomingJob
    {
        u16 id;
        ImpedanceHome control;
        std::chrono::steady_clock::time_point lastUpdate;
        std::shared_ptr<rmw_request_id_t> header; // Null for automatic power recovery.
    };
    std::optional<HomingJob> m_homingJob; // One mechanical-stop search at a time.
    std::unordered_set<u16> m_homingRequired;
    std::unordered_set<u16> m_autoHomePending;

    struct DisableRequest
    {
        std::shared_ptr<rmw_request_id_t> header;
        std::vector<u16> ids;
    };

    struct PendingDisable : candle_ros2::DisableTransition
    {
        using candle_ros2::DisableTransition::DisableTransition;
        explicit PendingDisable(candle_ros2::DisableTransition stop)
            : candle_ros2::DisableTransition(std::move(stop)) {}
        bool reported = false;
    };

    std::shared_ptr<mab::Candle> m_candle;
    std::vector<mab::MD>         m_mds;

    static constexpr const char* NODE_PREFIX = "md/";

    static constexpr double SOFT_CLOSE_MIN_SPEED_RAD_S = 0.4;
    static constexpr double SOFT_CLOSE_MAX_SPEED_RAD_S = 6.0;

    std::unordered_map<u16, SoftCloseJob> m_softCloseJobs;
    std::unordered_map<u16, PositionTracker> m_positionTrackers;
    std::unordered_set<u16> m_motionInhibited;
    std::unordered_map<u16, std::string> m_commandErrors;
    std::unordered_map<u16, RecoveryContext> m_recoveryContexts;
    std::unordered_map<u16, PendingDisable> m_pendingDisables;
    std::vector<DisableRequest> m_disableRequests;

    std::string jointNamePrefix;
    double      gripperOpenPositionRad;
    double      gripperClosedPositionRad;
    float       gripperImpedanceKp;
    float       gripperImpedanceKd;
    float       gripperVelocityLimitRadS;
    float       gripperOpenVelocityLimitRadS;
    float       gripperTorqueLimitNm;
    float       softCloseSlowTorqueLimitNm;
    ImpedanceClose::Config closeConfig;
    ImpedanceHome::Config homeConfig;
    bool homingRequired;
    int         jointStatePublishPeriodMs;
    int         healthPublishPeriodMs;
    int         mdCanResponseTimeout100us;
    int         mdHostTimeoutMs;
    double      encoderWrapPeriodRad;
    double      positionRecoveryMaxDeltaRad;
    int         positionRecoverySamples;
    int         positionRecoveryRetryMs;
    bool        initDevicesZero;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  pubJointState;
    rclcpp::Publisher<candle_ros2::msg::MdHealth>::SharedPtr    pubHealth;

    rclcpp::Subscription<candle_ros2::msg::MotionCmd>::SharedPtr      subMotionCmd;
    rclcpp::Subscription<candle_ros2::msg::PositionPidCmd>::SharedPtr subPositionCmd;
    rclcpp::Subscription<candle_ros2::msg::VelocityPidCmd>::SharedPtr subVelocityCmd;
    rclcpp::Subscription<candle_ros2::msg::ImpedanceCmd>::SharedPtr   subImpedanceCmd;

    rclcpp::Service<candle_ros2::srv::AddDevices>::SharedPtr        srvAddMd;
    rclcpp::Service<candle_ros2::srv::InitDevices>::SharedPtr       srvInitDevices;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvZero;
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvHome;
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
    rclcpp::TimerBase::SharedPtr tmrHealth;

    void publishJointStates();
    void publishHealth();
    void tickSoftClose(mab::MD& md, double position, double velocity, double torque);
    bool startImpedanceMotion(mab::MD& md, double speed, bool opening = false);
    void stopDrive(mab::MD& md, const std::string& reason);
    bool prepareDriveDisabled(mab::MD& md);
    void tickDisableJobs();
    void tickRecoveryJobs();
    bool isHoming(u16 id) const;
    bool startHoming(mab::MD& md, std::shared_ptr<rmw_request_id_t> header = nullptr);
    void tickHoming(mab::MD& md, double position, double velocity, double torque);
    void finishHoming(bool success);
    void completeHomingZero(mab::MD& md);
    void cbHome(std::shared_ptr<rmw_request_id_t> header,
                std::shared_ptr<candle_ros2::srv::Generic::Request> req);

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
    void cbDisable(std::shared_ptr<rmw_request_id_t> header,
                   std::shared_ptr<candle_ros2::srv::Generic::Request> req);
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
    bool setGripperTarget(mab::MD& md, double targetPos);
    bool setGripperCommand(mab::MD& md, const ImpedanceClose::Command& command);
    bool writeRawTarget(mab::MD& md, double rawTargetPos);
    bool moveGripper(mab::MD& md, double targetPos);
    bool resetDriveErrorsIfNeeded(mab::MD& md);
    std::optional<double> readLogicalPosition(mab::MD& md, bool triggerRecovery = true);
    void beginRecovery(u16 id);
    bool canAcceptPositionCommand(u16 id) const;
    bool cancelSoftClose(u16 id);

    static double normalizedSpeedToRadS(double normalizedSpeed);

    std::vector<mab::MD>::iterator findMd(std::vector<mab::MD>& mds, u16 id);
};
