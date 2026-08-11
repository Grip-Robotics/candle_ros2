#pragma once
#include <chrono>
#include <cmath>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>

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
#include "candle_ros2/homing_monitor.hpp"
#include "candle_ros2/position_tracker.hpp"
#include "candle_ros2/position_state_store.hpp"

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
        double         slowVelocityRadS;
        rclcpp::Time   requestStart;
        rclcpp::Time   slowStageStart;
    };

    struct ResumeCommand
    {
        double targetPosition = 0.0;
        bool   softClose      = false;
        double slowVelocity   = 0.0;
    };

    struct RecoveryContext
    {
        std::chrono::steady_clock::time_point nextAttempt;
        bool                                  errorsCleared = false;
    };

    enum class DriveStartupState
    {
        Uninitialized,
        Restoring,
        Homing,
        Tracking,
        Recovering,
        Faulted
    };

    enum class HomingStage
    {
        InitialBackoff,
        FirstSeek,
        Backoff,
        SecondSeek
    };

    struct HomingContext
    {
        HomingStage stage = HomingStage::FirstSeek;
        std::chrono::steady_clock::time_point stageStart;
        std::chrono::steady_clock::time_point nextSample;
        HomingSeekState seekState;
        double startRawPosition = 0.0;
        double firstStopRawPosition = 0.0;
        double backoffTargetRawPosition = 0.0;
        double lastTorqueCommand = 0.0;
        double lastBackoffKp = 0.0;
        std::optional<std::chrono::steady_clock::time_point> backoffSettleStart;
    };

    std::shared_ptr<mab::Candle> m_candle;
    std::vector<mab::MD>         m_mds;

    static constexpr const char* NODE_PREFIX  = "md/";
    static constexpr int         PUB_TIMER_MS = 5;  // 200 Hz

    static constexpr int SLOW_STAGE_TIMEOUT_MS = 1000;
    static constexpr double SOFT_CLOSE_MIN_SPEED_RAD_S = 0.4;
    static constexpr double SOFT_CLOSE_MAX_SPEED_RAD_S = 6.0;

    std::unordered_map<u16, SoftCloseJob> m_softCloseJobs;
    std::unordered_map<u16, PositionTracker> m_positionTrackers;
    std::unordered_map<u16, ResumeCommand> m_resumeCommands;
    std::unordered_map<u16, RecoveryContext> m_recoveryContexts;
    std::unordered_map<u16, DriveStartupState> m_startupStates;
    std::unordered_map<u16, int> m_homingDirections;
    std::unordered_map<u16, double> m_lastPersistedLogicalPositions;
    std::unordered_map<u16, std::uint64_t> m_calibrationGenerations;
    std::optional<std::pair<u16, HomingContext>> m_activeHoming;
    std::deque<u16> m_homingQueue;
    PositionStateStore m_positionStateStore;
    std::chrono::steady_clock::time_point m_lastStateWrite;

    std::string jointNamePrefix;
    double      gripperOpenPositionRad;
    double      gripperClosedPositionRad;
    float       gripperImpedanceKp;
    float       gripperImpedanceKd;
    float       gripperVelocityLimitRadS;
    float       gripperTorqueLimitNm;
    float       softCloseFastTorqueLimitNm;
    float       softCloseSlowTorqueLimitNm;
    float       softCloseProfileAccelerationRadS2;
    float       softCloseProfileDecelerationRadS2;
    double      softCloseClosedTolRad;
    int         softCloseFastDurationMs;
    int         healthPublishPeriodMs;
    double      encoderWrapPeriodRad;
    double      positionRecoveryMaxDeltaRad;
    int         positionRecoverySamples;
    int         positionRecoveryRetryMs;
    std::string startupPositionPolicy;
    int         positionStateWritePeriodMs;
    double      positionStateMinChangeRad;
    double      homingTorqueNm;
    double      homingSecondPassTorqueNm;
    double      homingStopVerificationTorqueNm;
    int         homingStopVerificationRampMs;
    int         homingTorqueRampMs;
    double      homingVelocityTripRadS;
    double      homingMinBusVoltageV;
    double      homingStallVelocityRadS;
    int         homingStallDwellMs;
    double      homingMaxTravelRad;
    int         homingTimeoutMs;
    double      homingBackoffRad;
    double      homingRepeatabilityRad;
    double      homingMinMotionRad;
    bool        initDevicesZero;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pubJointState;
    rclcpp::Publisher<candle_ros2::msg::MdHealth>::SharedPtr   pubHealth;

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
    rclcpp::Service<candle_ros2::srv::Generic>::SharedPtr           srvHome;

    rclcpp::TimerBase::SharedPtr tmrPub;
    rclcpp::TimerBase::SharedPtr tmrHealth;

    void publishJointStates();
    void publishHealth();
    void tickSoftCloseJobs();
    void tickRecoveryJobs();
    void tickHoming();
    void maybePersistPositionState(bool force = false);

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
    void cbHome(const std::shared_ptr<candle_ros2::srv::Generic::Request> req,
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
        mab::MD& md,
        double   kp,
        double   kd,
        double   velocityLimit,
        double   torqueLimit);
    bool profilePidReady(mab::MD& md);
    bool configurePositionProfile(mab::MD& md, double velocityLimit, double torqueLimit);
    bool setGripperTarget(mab::MD& md, double targetPos);
    bool writeRawTarget(mab::MD& md, double rawTargetPos);
    bool moveGripper(mab::MD& md, double targetPos);
    bool restoreNormalGripperConfig(mab::MD& md);
    bool resetDriveErrorsIfNeeded(mab::MD& md);
    std::optional<double> readLogicalPosition(mab::MD& md, bool triggerRecovery = true);
    void beginRecovery(u16 id);
    bool resumeAfterRecovery(mab::MD& md);
    bool canAcceptPositionCommand(u16 id) const;
    void rememberResumeCommand(u16 id, double target, bool softClose, double slowVelocity);
    bool tryRestorePosition(mab::MD& md);
    bool queueHoming(u16 id);
    bool startNextHoming();
    bool startTorqueSeek(mab::MD& md, HomingContext& context, HomingStage stage);
    bool writeTargetTorque(mab::MD& md, double torqueNm);
    void completeHoming(mab::MD& md, HomingContext& context, double stopRawPosition);
    void abortHoming(mab::MD& md, const std::string& reason);
    void parseHomingDirections(const std::string& setting);
    const char* startupStateName(DriveStartupState state) const;
    void cancelSoftClose(u16 id);

    /** Piecewise-linear calibration: finger gap [mm] → motor position [rad]. */
    double fingerGapToMotorPos(double gapMm) const;
    static double normalizedSpeedToRadS(double normalizedSpeed);

    std::vector<mab::MD>::iterator findMd(std::vector<mab::MD>& mds, u16 id);
};
