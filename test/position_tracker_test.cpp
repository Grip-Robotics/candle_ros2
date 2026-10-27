#include <gtest/gtest.h>

#include "candle_ros2/position_tracker.hpp"

namespace
{
constexpr double PERIOD = 0.62831853;
}

TEST(PositionTrackerTest, RecoversObservedPositiveWrapJump)
{
    PositionTracker tracker(PERIOD, 0.25, 3);
    tracker.initialize(0.452908);
    tracker.setLastTarget(0.63);
    tracker.markCommunicationLost();

    EXPECT_EQ(tracker.observeRecovery(-0.175257), PositionTracker::RecoveryResult::Pending);
    EXPECT_EQ(tracker.observeRecovery(-0.175296), PositionTracker::RecoveryResult::Pending);
    EXPECT_EQ(tracker.observeRecovery(-0.175280), PositionTracker::RecoveryResult::Recovered);
    EXPECT_NEAR(tracker.continuousPosition(), 0.453039, 0.001);
    ASSERT_TRUE(tracker.lastTarget().has_value());
    EXPECT_DOUBLE_EQ(*tracker.lastTarget(), 0.63);
}

TEST(PositionTrackerTest, RecoversObservedNegativeWrapJump)
{
    PositionTracker tracker(PERIOD, 0.25, 3);
    tracker.initialize(-0.706168);
    tracker.markCommunicationLost();

    EXPECT_EQ(tracker.observeRecovery(-0.077888), PositionTracker::RecoveryResult::Pending);
    EXPECT_EQ(tracker.observeRecovery(-0.077900), PositionTracker::RecoveryResult::Pending);
    EXPECT_EQ(tracker.observeRecovery(-0.077890), PositionTracker::RecoveryResult::Recovered);
    EXPECT_NEAR(tracker.continuousPosition(), -0.706209, 0.001);
}

TEST(PositionTrackerTest, PreservesSmallRealMotionAcrossWrap)
{
    PositionTracker tracker(PERIOD, 0.25, 2);
    tracker.initialize(0.45);
    tracker.markCommunicationLost();

    const double rawAfterWrap = 0.50 - PERIOD;
    EXPECT_EQ(tracker.observeRecovery(rawAfterWrap), PositionTracker::RecoveryResult::Pending);
    EXPECT_EQ(tracker.observeRecovery(rawAfterWrap), PositionTracker::RecoveryResult::Recovered);
    EXPECT_NEAR(tracker.continuousPosition(), 0.50, 1e-9);
}

TEST(PositionTrackerTest, ConvertsLogicalTargetsToRecoveredRawBranch)
{
    PositionTracker tracker(PERIOD, 0.25, 1);
    tracker.initialize(0.45);
    tracker.markCommunicationLost();

    const double recoveredRaw = 0.45 - PERIOD;
    EXPECT_EQ(tracker.observeRecovery(recoveredRaw), PositionTracker::RecoveryResult::Recovered);
    EXPECT_NEAR(tracker.logicalToRaw(0.63), 0.63 - PERIOD, 1e-9);
    const auto logicalAtTarget = tracker.observe(tracker.logicalToRaw(0.63));
    ASSERT_TRUE(logicalAtTarget.has_value());
    EXPECT_NEAR(*logicalAtTarget, 0.63, 1e-9);
}

TEST(PositionTrackerTest, RejectsAmbiguousRecoveryMotion)
{
    PositionTracker tracker(PERIOD, 0.25, 3);
    tracker.initialize(0.0);
    tracker.markCommunicationLost();

    EXPECT_EQ(tracker.observeRecovery(0.30), PositionTracker::RecoveryResult::Rejected);
    EXPECT_EQ(tracker.state(), PositionTracker::State::Faulted);
}
