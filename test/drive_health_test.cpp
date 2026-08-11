#include <gtest/gtest.h>

#include "candle_ros2/drive_health.hpp"

TEST(DriveHealthTest, TrackingIsCommandReadyAndHealthy)
{
    const auto health = startupHealthForState("TRACKING");
    EXPECT_FALSE(health.error);
    EXPECT_TRUE(health.description.empty());
}

TEST(DriveHealthTest, NonTrackingStatesAreErrorsWithStableDescriptions)
{
    const auto recovering = startupHealthForState("RECOVERING");
    EXPECT_TRUE(recovering.error);
    EXPECT_EQ(recovering.description,
              "position: recovering after communication loss, commands rejected");

    const auto faulted = startupHealthForState("FAULTED");
    EXPECT_TRUE(faulted.error);
    EXPECT_EQ(faulted.description,
              "position: reference lost, manual /md/zero calibration required");

    EXPECT_EQ(startupHealthForState("HOMING").description,
              "position: homing in progress, commands rejected");
    EXPECT_EQ(startupHealthForState("RESTORING").description,
              "position: restoring persisted state, commands rejected");
    EXPECT_EQ(startupHealthForState("UNINITIALIZED").description,
              "position: uninitialized, commands rejected");
}

TEST(DriveHealthTest, UnknownStateFailsClosed)
{
    const auto health = startupHealthForState("UNKNOWN");
    EXPECT_TRUE(health.error);
    EXPECT_EQ(health.description, "position: state unavailable, commands rejected");
}
