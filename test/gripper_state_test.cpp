#include <gtest/gtest.h>

#include <limits>

#include "candle_ros2/gripper_state.hpp"

namespace
{
constexpr double OPEN_POSITION = 0.0;
constexpr double CLOSED_POSITION = 0.63;
constexpr double POSITION_TOLERANCE = 0.01;
constexpr double MOVING_VELOCITY = 0.05;

GripperOperationalState classify(bool homing, bool available, double position, double velocity)
{
    return classifyGripperState(homing,
                                available,
                                position,
                                velocity,
                                OPEN_POSITION,
                                CLOSED_POSITION,
                                POSITION_TOLERANCE,
                                MOVING_VELOCITY);
}
}  // namespace

TEST(GripperStateTest, HomingHasHighestPriority)
{
    EXPECT_EQ(classify(true, false, 0.3, 1.0), GripperOperationalState::Homing);
}

TEST(GripperStateTest, MissingOrInvalidSamplesAreUnknown)
{
    EXPECT_EQ(classify(false, false, 0.0, 0.0), GripperOperationalState::Unknown);
    EXPECT_EQ(classify(false,
                       true,
                       std::numeric_limits<double>::quiet_NaN(),
                       0.0),
              GripperOperationalState::Unknown);
}

TEST(GripperStateTest, MotionTakesPriorityOverEndpoints)
{
    EXPECT_EQ(classify(false, true, OPEN_POSITION, MOVING_VELOCITY + 0.001),
              GripperOperationalState::Moving);
    EXPECT_EQ(classify(false, true, OPEN_POSITION, MOVING_VELOCITY),
              GripperOperationalState::Open);
}

TEST(GripperStateTest, RecognizesEndpointsAndIdleMidTravel)
{
    EXPECT_EQ(classify(false, true, OPEN_POSITION + POSITION_TOLERANCE * 0.5, 0.0),
              GripperOperationalState::Open);
    EXPECT_EQ(classify(false, true, CLOSED_POSITION - POSITION_TOLERANCE * 0.5, 0.0),
              GripperOperationalState::Closed);
    EXPECT_EQ(classify(false, true, 0.3, 0.0), GripperOperationalState::Idle);
}

TEST(GripperStateTest, PublishesStableNames)
{
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Open), "OPEN");
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Closed), "CLOSED");
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Homing), "HOMING");
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Moving), "MOVING");
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Idle), "IDLE");
    EXPECT_STREQ(gripperStateName(GripperOperationalState::Unknown), "UNKNOWN");
}
