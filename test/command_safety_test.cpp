#include <gtest/gtest.h>
#include "candle_ros2/command_safety.hpp"

TEST(CommandSafety, ExplicitDisableSurvivesRepeatedCommunicationRecovery)
{
    candle_ros2::RecoveryHomingPolicy policy;
    policy.requestRecovery(113);
    ASSERT_TRUE(policy.pending(113));
    policy.disable(113);
    for (int i = 0; i < 5; ++i) policy.requestRecovery(113);
    EXPECT_FALSE(policy.pending(113));
    EXPECT_FALSE(policy.take(113));
    policy.requestRecovery(447);
    EXPECT_TRUE(policy.take(447));
    EXPECT_FALSE(policy.take(447)); // Taking a job never retries it by itself.
    policy.activate(113);
    policy.requestRecovery(113);
    EXPECT_TRUE(policy.take(113));
}

TEST(CommandSafety, RejectsWideIdsBeforeNarrowing)
{
    EXPECT_TRUE(candle_ros2::validDeviceId(113));
    EXPECT_TRUE(candle_ros2::validDeviceId(65535));
    EXPECT_FALSE(candle_ros2::validDeviceId(0));
    EXPECT_FALSE(candle_ros2::validDeviceId(65649));
    EXPECT_FALSE(candle_ros2::validDeviceId(0xffffffffu));
}

TEST(CommandSafety, RejectsNonfiniteNegativeAndUnrepresentableConfiguration)
{
    struct Pid { double kp = 30, ki = 5, kd = .2, i_windup = 1, max_output = 2; };
    EXPECT_TRUE(candle_ros2::validPid(Pid{}));
    for (double bad : {-1.0, static_cast<double>(NAN), static_cast<double>(INFINITY), 1e300})
    {
        auto pid = Pid{}; pid.kp = bad; EXPECT_FALSE(candle_ros2::validPid(pid));
        pid = {}; pid.ki = bad; EXPECT_FALSE(candle_ros2::validPid(pid));
        pid = {}; pid.kd = bad; EXPECT_FALSE(candle_ros2::validPid(pid));
        pid = {}; pid.i_windup = bad; EXPECT_FALSE(candle_ros2::validPid(pid));
        pid = {}; pid.max_output = bad; EXPECT_FALSE(candle_ros2::validPid(pid));
    }
    auto pid = Pid{}; pid.max_output = 0; EXPECT_FALSE(candle_ros2::validPid(pid));
    EXPECT_FALSE(candle_ros2::finiteRegisterValue(1e300));
    EXPECT_FALSE(candle_ros2::finiteRegisterValue(-1e300));
    EXPECT_TRUE(candle_ros2::finiteRegisterValue(-.5));
    EXPECT_FALSE(candle_ros2::positiveRegisterValue(1e-300));
}

TEST(CommandSafety, TemperatureReadbackMustBeValidAndBelowBothCutoffs)
{
    EXPECT_TRUE(candle_ros2::healthyTemperatures(35, 42, 90));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(90, 42, 90));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(35, 90, 95));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(35, 42, NAN));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(NAN, 42, 90));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(35, NAN, 90));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(0, 42, 90));
    EXPECT_FALSE(candle_ros2::healthyTemperatures(35, 42, 0));
}
