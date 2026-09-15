#include <gtest/gtest.h>
#include <limits>
#include "candle_ros2/impedance_close.hpp"

TEST(ImpedanceCloseTest, LargeObjectAtStartCapsLeadThenHolds)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    for (int i = 0; i < 10; ++i)
    {
        close.update(0.0, 0.0, i == 0 ? 0.0 : 0.5, 0.05);
        EXPECT_LE(close.target, 0.04);
    }
    ASSERT_EQ(close.state, ImpedanceClose::State::Contact);
    const double hold = close.target;
    for (int i = 0; i < 200; ++i)
        close.update(0.0, 0.0, 0.5, 0.05);
    EXPECT_DOUBLE_EQ(close.target, hold);
    // Receding contact must not resume the sweep or accumulate target error.
    close.update(0.1, 0.0, 0.0, 0.05);
    EXPECT_DOUBLE_EQ(close.target, hold);
    close.update(-0.01, 0.0, 0.5, 0.05);
    EXPECT_LE(close.target - (-0.01), close.config.lead);
}

TEST(ImpedanceCloseTest, FreeMotionReachesClosedWithoutClaimingContact)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    double position = 0.0;
    for (int i = 0; i < 100 && close.state == ImpedanceClose::State::Closing; ++i)
    {
        const double previous = close.target;
        close.update(position, 0.0, 0.0, 0.05);
        EXPECT_LE(close.target - previous, 0.02000001);
        EXPECT_LE(close.target, close.closed);
        position = close.target;
    }
    EXPECT_EQ(close.state, ImpedanceClose::State::Closed);
}

TEST(ImpedanceCloseTest, ContactRequiresSustainedDirectionalTorqueAndLowVelocity)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    close.update(0.0, 0.0, 0.0, 0.05);
    close.update(0.0, 0.0, 0.5, 0.05);
    close.update(0.0, 0.1, 0.5, 0.05); // movement breaks the contact dwell
    EXPECT_DOUBLE_EQ(close.contactTime, 0.0);
    close.update(0.0, 0.0, -0.5, 0.05); // opposite torque is not closing contact
    EXPECT_DOUBLE_EQ(close.contactTime, 0.0);
    close.update(0.0, 0.0, 0.5, 0.05);
    close.update(0.0, 0.0, 0.5, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::Closing);
    close.update(0.0, 0.0, 0.5, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::Contact);
}

TEST(ImpedanceCloseTest, SupportsNegativeClosingDirection)
{
    ImpedanceClose close(0.0, -0.63, 0.4, {});
    for (int i = 0; i < 10; ++i)
    {
        close.update(0.0, 0.0, -0.5, 0.05);
        EXPECT_GE(close.target, -0.04);
        EXPECT_LE(close.target, 0.0);
    }
    EXPECT_EQ(close.state, ImpedanceClose::State::Contact);
}

TEST(ImpedanceCloseTest, DelayedTicksDoNotCatchUpAndStaleFeedbackFails)
{
    ImpedanceClose close(0.0, 0.63, 0.1, {});
    close.update(0.0, 0.0, 0.0, 0.20);
    EXPECT_NEAR(close.target, 0.005, 1e-9);
    EXPECT_EQ(close.update(0.0, 0.0, 0.0, 0.3), ImpedanceClose::State::Failed);
    EXPECT_EQ(close.update(0.0, 0.0, 0.0, 0.05), ImpedanceClose::State::Failed);
}

TEST(ImpedanceCloseTest, MissingContactTimesOutInsteadOfHoldingFullClose)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    for (int i = 0; i < 110; ++i)
        close.update(0.0, 0.0, 0.0, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::Failed);
    EXPECT_LE(close.target, 0.04);
}

TEST(ImpedanceCloseTest, RejectsInvalidConfigurationAndMeasurements)
{
    auto config = ImpedanceClose::Config{};
    config.lead = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ImpedanceClose(0.0, 0.63, 0.4, config), std::invalid_argument);
    EXPECT_THROW(ImpedanceClose(0.0, 0.63, 0.0, {}), std::invalid_argument);
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    EXPECT_EQ(close.update(0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.05),
              ImpedanceClose::State::Failed);
}
