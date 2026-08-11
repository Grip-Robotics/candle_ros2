#include <gtest/gtest.h>

#include "candle_ros2/homing_monitor.hpp"

TEST(HomingMonitorTest, RequiresMotionBeforeAcceptingAStop)
{
    HomingSeekState state;
    EXPECT_EQ(updateHomingSeek(
                  state, 0.0, 0.0, 1.0, 1000, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::Continue);
    EXPECT_FALSE(state.stallStartMs.has_value());
}

TEST(HomingMonitorTest, RequiresContinuousStallDwell)
{
    HomingSeekState state;
    EXPECT_EQ(updateHomingSeek(
                  state, -0.02, 0.01, 1.0, 1000, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::Continue);
    EXPECT_EQ(updateHomingSeek(
                  state, -0.02, 0.03, 1.0, 1100, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::Continue);
    EXPECT_FALSE(state.stallStartMs.has_value());
    EXPECT_EQ(updateHomingSeek(
                  state, -0.02, 0.0, 1.0, 1200, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::Continue);
    EXPECT_EQ(updateHomingSeek(
                  state, -0.02, 0.0, 1.0, 1450, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::StopDetected);
}

TEST(HomingMonitorTest, TripsIndependentSafetyLimits)
{
    HomingSeekState state;
    EXPECT_EQ(updateHomingSeek(
                  state, 0.0, 1.1, 0.2, 100, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::OverVelocity);
    EXPECT_EQ(updateHomingSeek(
                  state, 0.76, 0.0, 0.2, 100, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::MaxTravel);
    EXPECT_EQ(updateHomingSeek(
                  state, 0.1, 0.1, 0.2, 5001, 0.01, 0.75, 1.0, 0.02, 250, 5000),
              HomingSeekResult::Timeout);
}

TEST(HomingMonitorTest, ValidatesTwoPassRepeatability)
{
    EXPECT_TRUE(homingStopsRepeatable(-0.123, -0.130, 0.015));
    EXPECT_FALSE(homingStopsRepeatable(-0.123, -0.150, 0.015));
}
