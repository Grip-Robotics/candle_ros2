#include <gtest/gtest.h>
#include <limits>
#include <array>
#include <vector>
#include "candle_ros2/impedance_close.hpp"
#include "candle_ros2/impedance_home.hpp"

TEST(ImpedanceHome, OpeningTorqueCrossesZeroAndStopRequiresFullPositionDwell)
{
    for (double direction : {-1.0, 1.0})
    {
        ImpedanceClose::Config motion;
        motion.kp = 30; motion.kd = .2; motion.maxStep = .01;
        motion.torqueLimit = 2;
        ImpedanceHome home(direction * -.3, direction, {}, motion);
        EXPECT_DOUBLE_EQ(home.command().position, direction * -.3);
        EXPECT_DOUBLE_EQ(home.command().torque, direction * 1.0);
        EXPECT_DOUBLE_EQ(home.command().kp, 0);
        for (double q : {-.2, -.1, 0.0, .1})
        {
            EXPECT_EQ(home.update(direction * q, direction * 10, direction * 1.9, .01),
                      ImpedanceHome::State::Searching);
            EXPECT_DOUBLE_EQ(home.command().position, direction * q);
            EXPECT_DOUBLE_EQ(home.command().kp, 0); // Position target cannot pull against a stop.
            EXPECT_DOUBLE_EQ(home.command().kd, .5);
            EXPECT_DOUBLE_EQ(home.command().torque, direction * 1.0);
        }
        // Position alone identifies the stop; no minimum torque or velocity gate.
        for (int i = 0; i < 24; ++i)
            EXPECT_EQ(home.update(direction * .1, 8, i % 2 ? .3 : 0, .01), ImpedanceHome::State::Searching);
        EXPECT_EQ(home.update(direction * .1, 8, 0, .01), ImpedanceHome::State::Found);
        EXPECT_DOUBLE_EQ(home.command().torque, 0);
        EXPECT_DOUBLE_EQ(home.command().kp, 0);
        EXPECT_DOUBLE_EQ(home.command().kd, 0);
    }
}

TEST(ImpedanceHome, GuardsNeverProduceHomeAndSettingsAreBounded)
{
    EXPECT_TRUE(ImpedanceHome::unrestrictedPositionLimits(0, 0));
    EXPECT_TRUE(ImpedanceHome::unrestrictedPositionLimits(std::numeric_limits<float>::lowest(),
                                                         std::numeric_limits<float>::max()));
    EXPECT_FALSE(ImpedanceHome::unrestrictedPositionLimits(-.05, .75));
    EXPECT_FALSE(ImpedanceHome::unrestrictedPositionLimits(NAN, INFINITY));
    ImpedanceClose::Config motion;
    motion.kp = 30; motion.kd = .2; motion.maxStep = .01;
    motion.torqueLimit = 2;
    ImpedanceHome home(-.1, -1, {}, motion);
    EXPECT_DOUBLE_EQ(home.config.torque, 2);
    EXPECT_DOUBLE_EQ(home.config.seekTorque, 1);
    for (int i = 0; i < 210; ++i) home.update(-.1 - i * .001, 0, 0, .01);
    EXPECT_EQ(home.state, ImpedanceHome::State::Failed);
    for (const auto& sample : std::vector<std::array<double, 4>>{
             {-.97, 0, -1, .01}, {-.09, 0, 0, .01},
             {-.1, 0, -2.11, .01}, {-.1, 0, 2.11, .01}, {-.1, 0, -1, .3},
             {NAN, 0, 0, .01}, {-.1, NAN, 0, .01}, {-.1, 0, NAN, .01},
             {-.1, 0, 0, NAN}, {-.1, 0, 0, 0}, {-.1, 0, 0, -.01}})
    {
        ImpedanceHome guarded(-.1, -1, {}, motion);
        EXPECT_EQ(guarded.update(sample[0], sample[1], sample[2], sample[3]),
                  ImpedanceHome::State::Failed);
        EXPECT_DOUBLE_EQ(guarded.command().kp, 0);
        EXPECT_DOUBLE_EQ(guarded.command().torque, 0);
    }
    auto invalid = ImpedanceHome::Config{};
    invalid.torque = 3;
    EXPECT_THROW(ImpedanceHome(-.1, -1, invalid, motion), std::invalid_argument);
    invalid = {};
    invalid.seekTorque = 2.1;
    EXPECT_THROW(ImpedanceHome(-.1, -1, invalid, motion), std::invalid_argument);
    invalid = {};
    invalid.contactDuration = invalid.timeout;
    EXPECT_THROW(ImpedanceHome(-.1, -1, invalid, motion), std::invalid_argument);
    invalid = {};
    invalid.damping = NAN;
    EXPECT_THROW(ImpedanceHome(-.1, -1, invalid, motion), std::invalid_argument);
    EXPECT_THROW(ImpedanceHome(NAN, -1, {}, motion), std::invalid_argument);
    EXPECT_THROW(ImpedanceHome(-.1, 0, {}, motion), std::invalid_argument);
    ImpedanceHome delayed(-.1, -1, {}, motion);
    for (int i = 0; i < 4; ++i) delayed.update(-.1, 0, 0, .2);
    EXPECT_EQ(delayed.state, ImpedanceHome::State::Searching); // No catch-up dwell.
    EXPECT_NEAR(delayed.settledTime, .04, 1e-12);
}

TEST(ImpedanceHome, PositionMovementRestartsContactDwell)
{
    ImpedanceClose::Config motion;
    motion.kp = 30; motion.maxStep = .01;
    motion.torqueLimit = 2;
    ImpedanceHome home(-.1, -1, {}, motion);
    double q = -.1;
    for (int i = 0; i < 60; ++i)
    {
        q -= .001;
        EXPECT_EQ(home.update(q, 0, -1, .01), ImpedanceHome::State::Searching);
    }
    q -= .006; // Restart any partial window from the moving samples.
    home.update(q, 0, 0, .01);
    for (int i = 0; i < 24; ++i) home.update(q, 7, -1, .01);
    EXPECT_EQ(home.state, ImpedanceHome::State::Searching);
    home.update(q - .006, 0, -1, .01); // Position window exceeded: start again.
    for (int i = 0; i < 24; ++i) home.update(q - .006, 7, -1, .01);
    EXPECT_EQ(home.state, ImpedanceHome::State::Searching);
    EXPECT_EQ(home.update(q - .006, 7, -1, .01), ImpedanceHome::State::Found);
}

TEST(ImpedanceHome, SpeedAndReboundDoNotRejectHomeButReleaseNeedsFreshDwell)
{
    ImpedanceClose::Config motion;
    motion.kp = 30; motion.maxStep = .01;
    motion.torqueLimit = 2;
    ImpedanceHome::Config config;
    ImpedanceHome home(-.1, -1, config, motion);
    EXPECT_DOUBLE_EQ(home.config.seekTorque, 1);
    EXPECT_EQ(home.update(-.2, -10, -.5, .01), ImpedanceHome::State::Searching);
    for (int i = 0; i < 25; ++i) home.update(-.2, 6, -1, .01);
    ASSERT_EQ(home.state, ImpedanceHome::State::Found);
    home.beginRelease();
    EXPECT_EQ(home.updateRelease(0, 20, 0, .01), ImpedanceHome::State::Releasing);
    for (int i = 0; i < 24; ++i)
        EXPECT_EQ(home.updateRelease(0, 6, 0, .01), ImpedanceHome::State::Releasing);
    EXPECT_EQ(home.updateRelease(0, 6, .11, .01), ImpedanceHome::State::Releasing);
    for (int i = 0; i < 24; ++i)
        EXPECT_EQ(home.updateRelease(0, 6, 0, .01), ImpedanceHome::State::Releasing);
    EXPECT_EQ(home.updateRelease(0, 6, 0, .01), ImpedanceHome::State::Ready);
    EXPECT_DOUBLE_EQ(home.command().torque, 0);
    EXPECT_DOUBLE_EQ(home.command().kp, 0);
}

TEST(ImpedanceHome, ReleaseStillRejectsBadFeedbackMotionAndTorqueThatNeverSettle)
{
    ImpedanceClose::Config motion;
    motion.kp = 30; motion.maxStep = .01;
    motion.torqueLimit = 2;
    for (const auto& sample : std::vector<std::array<double, 4>>{
             {NAN, 0, 0, .01}, {-.2, NAN, 0, .01}, {-.2, 0, NAN, .01},
             {-.2, 0, 0, .3}, {-.2, 0, 0, NAN}, {-.2, 0, 0, 0}})
    {
        ImpedanceHome bad(-.2, -1, {}, motion);
        bad.beginRelease();
        EXPECT_EQ(bad.updateRelease(sample[0], sample[1], sample[2], sample[3]), ImpedanceHome::State::Failed);
        EXPECT_EQ(bad.updateRelease(-.2, 0, 0, .01), ImpedanceHome::State::Failed);
    }
    for (bool loaded : {false, true})
    {
        ImpedanceHome unsettled(-.2, -1, {}, motion);
        unsettled.beginRelease();
        for (int i = 0; i < 210; ++i)
            unsettled.updateRelease(loaded ? -.2 : -.2 + i * .001, 0, loaded ? .2 : 0, .01);
        EXPECT_EQ(unsettled.state, ImpedanceHome::State::Failed);
    }
}

TEST(ImpedanceCloseTest, AbsoluteTravelLimitsDoNotMoveWithTargetsOrSettlingTolerance)
{
    auto config = ImpedanceClose::Config{};
    EXPECT_DOUBLE_EQ(config.travelMin, -0.05);
    EXPECT_DOUBLE_EQ(config.travelMax, 0.75);
    EXPECT_DOUBLE_EQ(config.tolerance, 0.005);
    EXPECT_DOUBLE_EQ(config.closedHoldTorque, 0.25);
    for (double direction : {1.0, -1.0})
    {
        config.travelMin = direction > 0.0 ? -0.05 : -0.75;
        config.travelMax = direction > 0.0 ? 0.75 : 0.05;
        for (double position : {direction * -0.05, direction * -0.049999,
                                direction * 0.749999, direction * 0.75})
            EXPECT_TRUE(config.withinTravel(position));
        for (double position : {direction * -0.050001, direction * 0.750001,
                                direction * 0.80,
                                std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity()})
            EXPECT_FALSE(config.withinTravel(position));
        ImpedanceClose opening(direction * 0.007, 0.0, 10.0, config, true);
        for (int i = 0; i < 9; ++i)
            EXPECT_EQ(opening.update(direction * 0.007, 0.0, -direction * 0.2, 0.05),
                      ImpedanceClose::State::Closing);
        ImpedanceClose closing(direction * 0.743, direction * 0.75, 1.5, config);
        EXPECT_DOUBLE_EQ(closing.closed, direction * 0.75);
        EXPECT_FALSE(closing.config.withinTravel(direction * 0.750001));
        for (int i = 0; i < 10; ++i)
            EXPECT_EQ(closing.update(direction * 0.743, 0.0, direction * 0.1, 0.05),
                      ImpedanceClose::State::Closing);
    }
    for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()})
    {
        config = ImpedanceClose::Config{};
        config.travelMin = invalid;
        EXPECT_THROW(ImpedanceClose(0.0, 0.75, 1.5, config), std::invalid_argument);
        config = ImpedanceClose::Config{};
        config.travelMax = invalid;
        EXPECT_THROW(ImpedanceClose(0.0, 0.75, 1.5, config), std::invalid_argument);
    }
    for (double upper : {-0.05, -0.1})
    {
        config = ImpedanceClose::Config{};
        config.travelMax = upper;
        EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.5, config), std::invalid_argument);
    }
}

TEST(ImpedanceCloseTest, LargeObjectAtStartCapsLeadThenHoldsAndRecovers)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    for (int i = 0; i < 10; ++i)
    {
        close.update(0.0, 0.0, i == 0 ? 0.0 : 0.5, 0.05);
        EXPECT_LE(close.target, 0.04);
    }
    ASSERT_EQ(close.state, ImpedanceClose::State::TorqueHold);
    ASSERT_TRUE(close.hasObject);
    const double hold = close.target;
    for (int i = 0; i < 10; ++i)
        close.update(0.0, 0.0, 0.5, 0.05);
    EXPECT_DOUBLE_EQ(close.target, hold);
    // Displaced contact clears the old torque but retains the closing floor.
    close.update(0.1, 0.0, 0.0, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::Closing);
    EXPECT_DOUBLE_EQ(close.target, 0.1);
    EXPECT_DOUBLE_EQ(close.command().torque, 0.25);
    close.update(0.1, 0.0, 0.5, 0.05);
    EXPECT_LE(close.target - 0.1, close.config.lead);
}

TEST(ImpedanceCloseTest, FreeMotionReachesClosedWithoutClaimingContact)
{
    ImpedanceClose close(0.0, 0.7, 0.4, {});
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
    EXPECT_NEAR(position, 0.7, 1e-9);
}

TEST(ImpedanceCloseTest, ContactRequiresSustainedDirectionalTorqueAndLowVelocity)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    close.update(0.0, 0.0, 0.0, 0.05);
    close.update(0.0, 0.0, 0.5, 0.05);
    close.update(0.01, 0.1, 0.5, 0.05); // movement breaks the contact window
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
    EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
    EXPECT_TRUE(close.hasObject);
}

TEST(ImpedanceCloseTest, PositionProgressRejectsContactBetweenRampSteps)
{
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose motion(0.0, direction * 0.7, 0.4, {});
        double position = 0.0;
        for (int i = 0; i < 12; ++i)
        {
            position += direction * 0.002;
            motion.update(position, 0.0, direction * 0.5, 0.05);
            EXPECT_EQ(motion.state, ImpedanceClose::State::Closing);
        }
        for (int i = 0; i < 3; ++i)
            motion.update(position, 0.0, direction * 0.5, 0.05);
        EXPECT_EQ(motion.state, ImpedanceClose::State::Contact);
    }
}

TEST(ImpedanceCloseTest, OpeningRampsAtItsOwnSpeedWithLeadBoundAndNoIntegral)
{
    for (double direction : {1.0, -1.0})
        for (double speed : {0.4, 10.0})
        {
            auto config = ImpedanceClose::Config{};
            config.openLead = .14;
            config.maxStep = .01;
            ImpedanceClose motion(direction * 0.7, 0.0, speed, config, true);
            EXPECT_DOUBLE_EQ(motion.target, direction * 0.7); // safe enable hold
            motion.update(direction * 0.7, 0.0, 0.0, 0.001);
            EXPECT_NEAR(motion.target, direction * (0.7 - speed * 0.001), 1e-12);
            double position = motion.target;
            for (int i = 0; i < 250 && motion.state == ImpedanceClose::State::Closing; ++i)
            {
                const double previous = motion.target;
                motion.update(position, 0.0, 0.0, 0.01);
                EXPECT_LE(std::abs(motion.target - previous), speed * 0.01 + 1e-12);
                EXPECT_LE(std::abs(motion.target - position), config.openLead + 1e-12);
                EXPECT_GE(direction * motion.target, 0.0);
                EXPECT_DOUBLE_EQ(motion.integralTorque, 0.0);
                position = motion.target;
            }
            if (speed == .4)
            {
                // A deliberately slow sweep cannot finish before the new deadline.
                EXPECT_EQ(motion.state, ImpedanceClose::State::TorqueHold);
                EXPECT_GT(direction * position, config.tolerance);
                EXPECT_DOUBLE_EQ(motion.command().torque, 0);
                EXPECT_DOUBLE_EQ(motion.command().kp, 0);
                EXPECT_DOUBLE_EQ(motion.command().kd, 0);
                continue;
            }
            ASSERT_EQ(motion.state, ImpedanceClose::State::Closed);
            EXPECT_NEAR(motion.target, 0.0, 1e-12);
            EXPECT_DOUBLE_EQ(motion.command().torque, 0.0);
            EXPECT_DOUBLE_EQ(motion.command().kp, motion.config.kp);
            EXPECT_DOUBLE_EQ(motion.command().kd, motion.config.kd);
            motion.update(direction * 0.06, 0.0, 0.0, 0.05);
            EXPECT_LE(std::abs(motion.target - direction * 0.06), config.openLead + 1e-12);
        }
}

TEST(ImpedanceCloseTest, IndependentOpeningLeadPreservesClosingAndOpeningGuards)
{
    auto config = ImpedanceClose::Config{};
    config.lead = 0.04;
    config.openLead = 0.10;
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose close(0.0, direction * 0.75, 2.0, config);
        ImpedanceClose opening(direction * 0.7, 0.0, 10.0, config, true);
        for (int i = 0; i < 4; ++i)
        {
            close.update(0.0, 0.0, 0.0, 0.01);
            opening.update(direction * 0.7, 0.0, 0.0, 0.01);
        }
        EXPECT_NEAR(direction * close.target, config.lead, 1e-12);
        EXPECT_NEAR(direction * opening.target, 0.7 - config.openLead, 1e-12);
        EXPECT_DOUBLE_EQ(opening.integralTorque, 0.0);
        opening.update(direction * 0.7, 0.0, 0.0, 0.20);
        EXPECT_NEAR(direction * opening.target, 0.7 - config.openLead, 1e-12);
        for (int i = 0; i < 20; ++i)
            opening.update(direction * 0.7, 0.0, -direction * 0.8, 0.01);
        EXPECT_EQ(opening.state, ImpedanceClose::State::Contact);
        EXPECT_EQ(opening.update(direction * 0.7, 0.0, 0.0, 0.3),
                  ImpedanceClose::State::Failed);
    }
    for (double invalid : {0.0, -0.1, config.tolerance,
                           std::numeric_limits<double>::quiet_NaN()})
    {
        config.openLead = invalid;
        EXPECT_THROW(ImpedanceClose(0.7, 0.0, 10.0, config, true), std::invalid_argument);
        EXPECT_THROW(ImpedanceClose(0.0, 0.7, 2.0, config), std::invalid_argument);
    }
}

TEST(ImpedanceCloseTest, ClosingAtOnePointFiveStillRespectsRampAndLead)
{
    auto config = ImpedanceClose::Config{};
    config.maxStep = 0.01;
    config.lead = 0.02666666666666667;
    ImpedanceClose close(0.0, 0.7, 1.5, config);
    close.update(0.0, 0.0, 0.0, 0.01);
    EXPECT_NEAR(close.target, 0.015, 1e-12);
    close.update(0.0, 0.0, 0.0, 0.01);
    EXPECT_DOUBLE_EQ(close.target, config.lead);
    close.update(0.0, 0.0, 0.0, 0.20); // delayed tick cannot exceed lead
    EXPECT_DOUBLE_EQ(close.target, config.lead);
}

TEST(ImpedanceCloseTest, FastOpeningRetainsLeadObstructionAndStaleFeedbackChecks)
{
    ImpedanceClose opening(0.7, 0.0, 10.0, {}, true);
    opening.update(0.7, 0.0, 0.0, 0.05);
    EXPECT_NEAR(opening.target, 0.7 - opening.config.lead, 1e-12);
    for (int i = 0; i < 3; ++i)
        opening.update(0.7, 0.0, -0.8, 0.05);
    // The ROS node turns opening contact into stop/disable, not a grasp hold.
    EXPECT_EQ(opening.state, ImpedanceClose::State::Contact);
    EXPECT_DOUBLE_EQ(opening.command().torque, 0.0);
    EXPECT_EQ(opening.update(0.7, 0.0, -0.8, 0.3), ImpedanceClose::State::Failed);
}

TEST(ImpedanceCloseTest, EndpointRequiresSettledPositionForTheDwell)
{
    ImpedanceClose motion(0.66, 0.7, 0.4, {});
    motion.update(0.68, 0.0, 0.0, 0.05);
    motion.update(0.698, 0.0, 0.0, 0.05);
    EXPECT_EQ(motion.state, ImpedanceClose::State::Closing);
    for (int i = 0; i < 2; ++i)
    {
        motion.update(0.698, 0.0, 0.0, 0.05);
        EXPECT_EQ(motion.state, ImpedanceClose::State::Closing);
    }
    motion.update(0.698, 0.0, 0.0, 0.05);
    EXPECT_EQ(motion.state, ImpedanceClose::State::Closed);
}

TEST(ImpedanceCloseTest, DelayedTicksDoNotCatchUpAndStaleFeedbackFails)
{
    ImpedanceClose close(0.0, 0.63, 0.1, {});
    close.update(0.0, 0.0, 0.0, 0.20);
    EXPECT_NEAR(close.target, 0.005, 1e-9);
    EXPECT_EQ(close.update(0.0, 0.0, 0.0, 0.3), ImpedanceClose::State::Failed);
    EXPECT_EQ(close.update(0.0, 0.0, 0.0, 0.05), ImpedanceClose::State::Failed);
}

TEST(ImpedanceCloseTest, ClosingDeadlineCapturesTorqueInsteadOfChasingEndpoint)
{
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    for (int i = 0; i < 110; ++i)
        close.update(0.0, 0.0, 0.0, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
    EXPECT_DOUBLE_EQ(close.target, 0.0);
    EXPECT_DOUBLE_EQ(close.command().torque, 0.25);
}

TEST(ImpedanceCloseTest, IntegralUsesBoundedTargetErrorAndElapsedTime)
{
    auto config = ImpedanceClose::Config{};
    config.maxStep = 0.01;
    config.kp = 30.0;
    config.kd = 0.2;
    config.lead = 0.8 / config.kp;
    ImpedanceClose close(0.0, 0.7, 1.0, config);
    close.update(0.0, 0.0, 0.0, 0.01);
    EXPECT_NEAR(close.integralTorque, 2.0 * 0.01 * 0.01, 1e-12);
    // A delayed tick does not accumulate a large integral jump.
    const double before = close.integralTorque;
    close.update(0.0, 0.0, 0.0, 0.2);
    EXPECT_NEAR(close.integralTorque - before, 2.0 * 0.02 * 0.01, 1e-12);
    for (int i = 0; i < 20; ++i)
    {
        const double previous = close.integralTorque;
        close.update(0.0, 0.0, 0.0, 0.01);
        EXPECT_LE(close.integralTorque - previous, config.ki * config.lead * 0.01 + 1e-12);
        EXPECT_LE(close.command().torque, config.torqueLimit);
    }
    EXPECT_GT(close.integralTorque, 0.0);
    EXPECT_LE(close.integralTorque, config.ki * config.lead * 0.22);
}

TEST(ImpedanceCloseTest, IntegralAntiWindupAllowsOnlyUnwindingAtSaturation)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose close(0.0, direction * 0.04, 1.0, config);
        close.update(0.0, 0.0, 0.0, 0.05); // PD alone exceeds 1 Nm.
        EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
        close.integralTorque = direction * 0.9;
        // Small reversed error, still saturated by the damping term.
        close.update(direction * 0.05, -direction * 10.0, 0.0, 0.05);
        EXPECT_LT(direction * close.integralTorque, 0.9);
        EXPECT_GT(direction * close.integralTorque, 0.0);
    }
}

TEST(ImpedanceCloseTest, TwoSecondCaptureUsesMeasuredTorqueAndRemovesPdAndIntegral)
{
    for (double direction : {1.0, -1.0})
        for (double measured : {0.6, 1.4, -0.3})
        {
            ImpedanceClose close(0.0, direction * 0.7, 1.0, {});
            for (int i = 0; i < 39; ++i)
                close.update(0.0, 0.0, 0.0, 0.05);
            ASSERT_EQ(close.state, ImpedanceClose::State::Closing);
            ASSERT_GT(direction * close.integralTorque, 0.0);
            ASSERT_EQ(close.update(0.0, 0.0, direction * measured, 0.05),
                      ImpedanceClose::State::TorqueHold);
            EXPECT_NEAR(close.elapsed, 2.0, 1e-12);
            const auto command = close.command();
            EXPECT_DOUBLE_EQ(command.torque, direction * std::clamp(measured, 0.25, 1.0));
            EXPECT_DOUBLE_EQ(command.kp, 0.0);
            EXPECT_DOUBLE_EQ(command.kd, 0.0);
            EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
            close.update(direction * 0.01, 0.0, direction * 0.2, 0.05);
            EXPECT_DOUBLE_EQ(close.command().torque, command.torque);
            // Reacquisition discards the old torque and starts at the measurement.
            EXPECT_EQ(close.update(direction * 0.05, 0.0, 0.0, 0.05),
                      ImpedanceClose::State::Closing);
            EXPECT_DOUBLE_EQ(close.holdingTorque, 0.0);
            EXPECT_DOUBLE_EQ(close.target, direction * 0.05);
            EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
            EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
        }
}

TEST(ImpedanceCloseTest, SettledEndpointKeepsCappedFloorWithoutPdIntegralOrDeadlineRecapture)
{
    auto config = ImpedanceClose::Config{};
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
        for (double limit : {0.1, 2.5})
            for (double measured : {-0.1, 0.1, 1.0})
            {
                config.torqueLimit = limit;
                const double desired = std::min(0.25, limit);
                ImpedanceClose close(direction * 0.698, direction * 0.7, 1.5, config);
                for (int i = 0; i < 15; ++i)
                    close.update(direction * 0.698, 0.0, direction * measured, 0.01);
                ASSERT_EQ(close.state, ImpedanceClose::State::Closed);
                EXPECT_FALSE(close.hasObject);
                EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
                EXPECT_DOUBLE_EQ(close.command().kd, 0.0);
                EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
                EXPECT_DOUBLE_EQ(direction * close.command().torque, desired);
                const double previousTorque = close.command().torque;
                close.update(direction * 0.698, 0.0, direction * measured, 0.2);
                EXPECT_LE(std::abs(close.command().torque - previousTorque), 0.02 + 1e-12);
                for (int i = 0; i < 250; ++i)
                    close.update(direction * 0.698, 0.0, direction * measured, 0.01);
                EXPECT_EQ(close.state, ImpedanceClose::State::Closed);
                EXPECT_NEAR(close.command().torque, direction * desired, 1e-12);
            }
}

TEST(ImpedanceCloseTest, OpeningAndZeroKiDoNotIntegrateAndOpeningTimeoutReleases)
{
    ImpedanceClose opening(0.7, 0.0, 1.0, {}, true);
    auto config = ImpedanceClose::Config{};
    config.ki = 0.0;
    ImpedanceClose close(0.0, 0.7, 1.0, config);
    for (int i = 0; i < 41; ++i)
    {
        opening.update(0.7, 0.0, 0.0, 0.05);
        close.update(0.0, 0.0, 0.0, 0.05);
        EXPECT_DOUBLE_EQ(opening.integralTorque, 0.0);
        EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
    }
    EXPECT_EQ(opening.state, ImpedanceClose::State::TorqueHold);
    EXPECT_DOUBLE_EQ(opening.command().torque, 0.0);
    EXPECT_DOUBLE_EQ(opening.command().kp, 0.0);
    EXPECT_DOUBLE_EQ(opening.command().kd, 0.0);
    EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
    ImpedanceClose next(0.3, 0.7, 1.0, {});
    EXPECT_DOUBLE_EQ(next.integralTorque, 0.0);
    EXPECT_DOUBLE_EQ(next.holdingTorque, 0.0);
}

TEST(ImpedanceCloseTest, HalfSecondOpeningDeadlineIsIndependentOfClosing)
{
    auto config = ImpedanceClose::Config{};
    EXPECT_DOUBLE_EQ(config.openTimeout, .5);
    EXPECT_DOUBLE_EQ(config.timeout, 2);
    for (double timeout : {.5, .8})
    {
        config.openTimeout = timeout;
        ImpedanceClose opening(.4, 0, 10, config, true);
        ImpedanceClose closing(.4, .75, 2, config);
        opening.elapsed = closing.elapsed = timeout - .02;
        EXPECT_EQ(opening.update(.4, 0, 0, .01), ImpedanceClose::State::Closing);
        EXPECT_EQ(opening.update(.4, 0, 0, .02), ImpedanceClose::State::TorqueHold);
        EXPECT_DOUBLE_EQ(opening.command().torque, 0);
        EXPECT_DOUBLE_EQ(opening.command().kp, 0);
        EXPECT_DOUBLE_EQ(opening.command().kd, 0);
        EXPECT_EQ(closing.update(.4, 0, 0, .03), ImpedanceClose::State::Closing);
        closing.elapsed = config.timeout - .01;
        EXPECT_EQ(closing.update(.4, 0, .4, .02), ImpedanceClose::State::TorqueHold);
        EXPECT_DOUBLE_EQ(closing.command().torque, .4);
    }
    for (double invalid : std::array<double, 5>{0.0, -.5, .15, NAN, INFINITY})
    {
        config.openTimeout = invalid;
        EXPECT_THROW(ImpedanceClose(.4, 0, 10, config, true), std::invalid_argument);
    }
}

TEST(ImpedanceCloseTest, OpeningReleaseStaysPassiveButFeedbackFaultsStillFail)
{
    auto config = ImpedanceClose::Config{};
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
        for (double distance : {0.01, 0.4})
        {
            const double position = direction * distance;
            ImpedanceClose opening(position, 0.0, 10.0, config, true);
            opening.elapsed = config.openTimeout - 0.01;
            auto atDeadline = opening;
            ASSERT_EQ(opening.update(position, 0.0, -direction * 0.3, 0.02),
                      ImpedanceClose::State::TorqueHold);
            EXPECT_DOUBLE_EQ(opening.target, position);
            EXPECT_FALSE(opening.hasObject);
            for (double displacement : {0.0, 0.1, -0.05, 0.0})
            {
                // Rebound/manual movement must not re-engage PD or retry opening.
                EXPECT_EQ(opening.update(position + direction * displacement,
                                         direction * 20.0, 0.0, 0.01),
                          ImpedanceClose::State::TorqueHold);
                EXPECT_DOUBLE_EQ(opening.command().torque, 0.0);
                EXPECT_DOUBLE_EQ(opening.command().kp, 0.0);
                EXPECT_DOUBLE_EQ(opening.command().kd, 0.0);
                EXPECT_DOUBLE_EQ(opening.integralTorque, 0.0);
            }
            for (bool stale : {false, true})
                for (auto faulted : {atDeadline, opening})
                {
                    EXPECT_EQ(faulted.update(position, 0.0,
                                             stale ? 0.0 : std::numeric_limits<double>::quiet_NaN(),
                                             stale ? 0.3 : 0.02), ImpedanceClose::State::Failed);
                    EXPECT_EQ(faulted.update(position, 0.0, 0.0, 0.01),
                              ImpedanceClose::State::Failed);
                }
            ImpedanceClose next(position, direction * 0.75, 1.5, config);
            EXPECT_DOUBLE_EQ(next.command().kp, 0.0);
            EXPECT_DOUBLE_EQ(next.command().kd, 0.0);
            EXPECT_DOUBLE_EQ(next.command().torque, direction * 0.25);
        }
}

TEST(ImpedanceCloseTest, FaultsAtDeadlineOrDuringHoldDisableRatherThanCapture)
{
    for (bool holding : {false, true})
        for (bool stale : {false, true})
        {
            ImpedanceClose close(0.0, 0.7, 1.0, {});
            close.update(0.0, 0.0, 0.0, 0.05);
            close.elapsed = 1.99;
            if (holding)
                close.update(0.0, 0.0, 0.5, 0.02);
            EXPECT_EQ(close.update(0.0, 0.0,
                                   stale ? 0.5 : std::numeric_limits<double>::quiet_NaN(),
                                   stale ? 0.3 : 0.02), ImpedanceClose::State::Failed);
            EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
            EXPECT_DOUBLE_EQ(close.holdingTorque, 0.0);
            EXPECT_EQ(close.update(0.0, 0.0, 0.5, 0.02), ImpedanceClose::State::Failed);
        }
}

TEST(ImpedanceCloseTest, RejectsInvalidConfigurationAndMeasurements)
{
    auto config = ImpedanceClose::Config{};
    config.lead = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(ImpedanceClose(0.0, 0.63, 0.4, config), std::invalid_argument);
    EXPECT_THROW(ImpedanceClose(0.0, 0.63, 0.0, {}), std::invalid_argument);
    config = {};
    config.ki = -1.0;
    EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.0, config), std::invalid_argument);
    config.ki = std::numeric_limits<double>::infinity();
    EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.0, config), std::invalid_argument);
    ImpedanceClose close(0.0, 0.63, 0.4, {});
    EXPECT_EQ(close.update(0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.05),
              ImpedanceClose::State::Failed);
}

TEST(ImpedanceCloseTest, NoisyObjectContactRampsToRequestedTorqueWithoutAddingPd)
{
    auto config = ImpedanceClose::Config{};
    EXPECT_DOUBLE_EQ(config.holdTorque, 2.0);
    config.kp = 30.0;
    config.ki = 5.0;
    config.maxStep = 0.01;
    for (double limit : {1.0, 2.0, 2.5})
        for (double direction : {1.0, -1.0})
        {
            config.torqueLimit = limit;
            ImpedanceClose close(direction * 0.3, direction * 0.7, 1.5, config);
            bool sawContact = false;
            for (int i = 0; i < 150; ++i)
            {
                const bool wasHolding = close.hasObject;
                const double previousTorque = close.command().torque;
                const double jitter = i % 2 ? 1.0 : -1.0;
                close.update(direction * 0.3 + jitter * 0.0001,
                             jitter * 0.08, direction * 0.8, 0.01);
                if (close.hasObject)
                {
                    sawContact = true;
                    EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
                    EXPECT_DOUBLE_EQ(close.command().kd, 0.0);
                    EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
                    EXPECT_LE(std::abs(close.command().torque), limit);
                    if (wasHolding)
                    {
                        EXPECT_LE(std::abs(close.command().torque - previousTorque),
                                  config.holdTorqueRate * 0.01 + 1e-12);
                    }
                }
            }
            EXPECT_TRUE(sawContact);
            EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
            EXPECT_NEAR(close.command().torque,
                        direction * std::min(config.holdTorque, limit), 1e-9);
        }
}

TEST(ImpedanceCloseTest, MovementOrEndpointApproachReacquiresWithoutStaleTorque)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.torqueLimit = 2.0;
    for (double direction : {1.0, -1.0})
        for (double displacement : {-0.05, 0.05, 0.005, 0.398})
        {
            ImpedanceClose close(direction * 0.3, direction * 0.7, 1.5, config);
            for (int i = 0; i < 40; ++i)
                close.update(direction * 0.3, 0.0, direction * 0.8, 0.05);
            ASSERT_EQ(close.state, ImpedanceClose::State::TorqueHold);
            ASSERT_NEAR(close.command().torque, direction * 2.0, 1e-9);
            const double position = direction * (0.3 + displacement);
            // Even a small displacement recovers when velocity signals release.
            const double velocity = displacement == 0.005 ? direction * 2.0 : 0.0;
            ASSERT_EQ(close.update(position, velocity, direction * 2.0, 0.01),
                      ImpedanceClose::State::Closing);
            EXPECT_FALSE(close.hasObject);
            EXPECT_DOUBLE_EQ(close.target, position);
            const bool braking = direction * velocity > close.speed;
            EXPECT_DOUBLE_EQ(close.command().torque, braking ? 0.0 : direction * 0.25);
            EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
            EXPECT_DOUBLE_EQ(close.command().kp, braking ? config.kp : 0.0);
            EXPECT_DOUBLE_EQ(close.elapsed, 0.0);
            EXPECT_DOUBLE_EQ(close.contactTime, 0.0);
            // The next tick advances only the bounded ramp, never the old anchor.
            close.update(position, 0.0, direction * 0.8, 0.01);
            EXPECT_LE(std::abs(close.target - position), 1.5 * 0.01 + 1e-12);
            EXPECT_FALSE(close.hasObject); // fresh contact dwell is required
            for (int i = 0; i < 40; ++i)
                close.update(position, 0.0, direction * 0.8, 0.05);
            if (displacement == 0.398)
            {
                EXPECT_FALSE(close.hasObject); // near closed is not proof of an object
                EXPECT_EQ(close.state, ImpedanceClose::State::Closed);
                EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
            }
            else
            {
                EXPECT_TRUE(close.hasObject);
                EXPECT_NEAR(close.command().torque, direction * 2.0, 1e-9);
            }
        }
}

TEST(ImpedanceCloseTest, GradualCompressionKeepsTorqueRampAndRecoversOnReopening)
{
    auto config = ImpedanceClose::Config{};
    config.holdTorque = 2.5; // Retain coverage of an explicitly higher requested torque.
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.lead = 0.02666666666666667;
    config.maxStep = 0.01;
    config.torqueLimit = 2.5;
    for (double direction : {1.0, -1.0})
    {
        // Bottle trial: contact at 0.4282 rad, then compression exceeded the
        // old fixed anchor's lead window before torque reached 2 Nm.
        double position = direction * 0.4282;
        ImpedanceClose close(position, direction * 0.7, 1.5, config);
        for (int i = 0; i < 20; ++i)
            close.update(position, 0.0, direction * 0.9281, 0.01);
        ASSERT_TRUE(close.hasObject);
        ASSERT_EQ(close.state, ImpedanceClose::State::Contact);
        for (int i = 0; i < 100; ++i)
        {
            position += direction * 0.001; // 0.1 rad/s, including after 2.5 Nm
            const double previousTorque = close.command().torque;
            close.update(position, direction * 0.1, previousTorque, 0.01);
            ASSERT_TRUE(close.hasObject);
            ASSERT_NE(close.state, ImpedanceClose::State::Closing);
            EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
            EXPECT_DOUBLE_EQ(close.command().kd, 0.0);
            EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
            EXPECT_GE(direction * (close.command().torque - previousTorque), -1e-12);
            EXPECT_LE(std::abs(close.command().torque - previousTorque), 0.02 + 1e-12);
            EXPECT_LE(std::abs(close.command().torque), config.torqueLimit);
        }
        ASSERT_EQ(close.state, ImpedanceClose::State::TorqueHold);
        EXPECT_NEAR(close.command().torque, direction * 2.5, 1e-9);

        // The anchor follows only inward: small outward steps must accumulate
        // into recovery, rather than silently following an opening disturbance.
        const double anchor = position;
        for (int i = 1; i <= 27; ++i)
        {
            position = anchor - direction * i * 0.001;
            close.update(position, -direction * 0.1, direction * 2.5, 0.01);
            if (i <= 26)
            {
                ASSERT_TRUE(close.hasObject);
                EXPECT_DOUBLE_EQ(close.target, anchor);
            }
        }
        EXPECT_EQ(close.state, ImpedanceClose::State::Closing);
        EXPECT_FALSE(close.hasObject);
        EXPECT_DOUBLE_EQ(close.target, position);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
        EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
        EXPECT_DOUBLE_EQ(close.elapsed, 0.0);
    }
}

TEST(ImpedanceCloseTest, TorqueRampRecoversOnFastPositionChangeDespiteZeroReportedVelocity)
{
    auto config = ImpedanceClose::Config{};
    config.torqueLimit = 2.0;
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
        for (double displacement : {-0.02, 0.02})
        {
            ImpedanceClose close(direction * 0.3, direction * 0.7, 1.5, config);
            for (int i = 0; i < 20; ++i)
                close.update(direction * 0.3, 0.0, direction * 0.8, 0.01);
            ASSERT_EQ(close.state, ImpedanceClose::State::Contact);
            const double position = direction * (0.3 + displacement);
            // Less than lead, but position-derived speed is 2 rad/s.
            EXPECT_EQ(close.update(position, 0.0, direction * 0.8, 0.01),
                      ImpedanceClose::State::Closing);
            EXPECT_FALSE(close.hasObject);
            EXPECT_DOUBLE_EQ(close.target, position);
            EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        }
}

TEST(ImpedanceCloseTest, TorqueRampDoesNotCatchUpAndFaultsNeverReacquire)
{
    auto config = ImpedanceClose::Config{};
    config.torqueLimit = 2.0;
    config.maxStep = 0.01;
    for (bool stale : {false, true})
    {
        ImpedanceClose close(0.3, 0.7, 1.5, config);
        for (int i = 0; i < 20; ++i)
            close.update(0.3, 0.0, 0.8, 0.01);
        ASSERT_TRUE(close.hasObject);
        ASSERT_EQ(close.state, ImpedanceClose::State::Contact);
        const double previous = close.command().torque;
        close.update(0.3, 0.0, 0.8, 0.2);
        EXPECT_NEAR(close.command().torque - previous, config.holdTorqueRate * 0.01, 1e-12);
        EXPECT_EQ(close.update(0.35, 2.0,
                               stale ? 0.8 : std::numeric_limits<double>::quiet_NaN(),
                               stale ? 0.3 : 0.01), ImpedanceClose::State::Failed);
        EXPECT_DOUBLE_EQ(close.holdingTorque, 0.0);
        EXPECT_FALSE(close.hasObject);
        EXPECT_EQ(close.update(0.35, 0.0, 0.8, 0.01), ImpedanceClose::State::Failed);
    }
}

TEST(ImpedanceCloseTest, ObjectEvidenceRejectsNearEndpointMotionAndOppositeTorque)
{
    auto config = ImpedanceClose::Config{};
    config.torqueLimit = 2.0;
    for (int scenario = 0; scenario < 4; ++scenario)
    {
        const double start = scenario == 0 ? 0.695 : 0.3;
        ImpedanceClose close(start, 0.7, 1.5, config);
        for (int i = 0; i < 40; ++i)
        {
            const double position = scenario == 1 ? start + (i % 2 ? 0.006 : 0.0) : start;
            const double velocity = scenario == 2 ? 0.1 : 0.0;
            const double torque = scenario == 3 ? -0.8 : 0.8;
            close.update(position, velocity, torque, 0.05);
            EXPECT_FALSE(close.hasObject);
            EXPECT_LE(std::abs(close.command().torque), 0.8);
        }
    }
    config.holdTorqueRate = 0.0;
    EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.5, config), std::invalid_argument);
    config.holdTorqueRate = 2.0;
    config.objectMargin = config.tolerance / 2;
    EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.5, config), std::invalid_argument);
}

TEST(ImpedanceCloseTest, LateConfirmedObjectStillBuildsRequestedTorqueAfterDeadline)
{
    auto config = ImpedanceClose::Config{};
    config.torqueLimit = 2.0;
    ImpedanceClose close(0.3, 0.7, 1.5, config);
    close.elapsed = 1.75;
    for (int i = 0; i < 4; ++i)
        close.update(0.3, 0.0, 0.8, 0.05);
    ASSERT_TRUE(close.hasObject);
    for (int i = 0; i < 20; ++i)
        close.update(0.3, 0.0, 0.8, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
    EXPECT_NEAR(close.command().torque, 2.0, 1e-9);
}

TEST(ImpedanceCloseTest, EmptyEndpointHoldCanReacquireBeforeCaptureDeadline)
{
    ImpedanceClose close(0.698, 0.7, 1.5, {});
    for (int i = 0; i < 5; ++i)
        close.update(0.698, 0.0, 0.1, 0.05);
    ASSERT_EQ(close.state, ImpedanceClose::State::Closed);
    EXPECT_EQ(close.update(0.6, 0.0, 0.0, 0.05), ImpedanceClose::State::Closing);
    EXPECT_DOUBLE_EQ(close.target, 0.6);
    EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
    EXPECT_FALSE(close.hasObject);
}

TEST(ImpedanceCloseTest, SmallObjectHoldDropsToFloorWhenEnteringClosedMargin)
{
    auto config = ImpedanceClose::Config{};
    config.torqueLimit = 2.0;
    ImpedanceClose close(0.685, 0.7, 1.5, config);
    for (int i = 0; i < 40; ++i)
        close.update(0.685, 0.0, 0.8, 0.05);
    ASSERT_TRUE(close.hasObject);
    ASSERT_NEAR(close.command().torque, 2.0, 1e-9);
    // Movement is smaller than the recovery lead, but reaches the end region.
    EXPECT_EQ(close.update(0.695, 0.0, 2.0, 0.01), ImpedanceClose::State::Closing);
    EXPECT_DOUBLE_EQ(close.target, 0.695);
    EXPECT_DOUBLE_EQ(close.command().torque, 0.25);
    EXPECT_FALSE(close.hasObject);
}

TEST(ImpedanceCloseTest, NearClosedReboundDoesNotRestartObjectHoldOrDeadline)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.torqueLimit = 2.0;
    config.lead = 0.02666666666666667;
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
    {
        // Empty 113: contact near 0.685 rad, recovery at 0.6903,
        // then small rebounds repeatedly reclassified the fingertips as an object.
        ImpedanceClose close(direction * 0.6852, direction * 0.7, 1.5, config);
        for (int i = 0; i < 20; ++i)
            close.update(direction * 0.6852, 0.0, direction * 0.52, 0.01);
        ASSERT_TRUE(close.hasObject);
        ASSERT_EQ(close.update(direction * 0.6903, 0.0, direction * 0.6, 0.01),
                  ImpedanceClose::State::Closing);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        for (int i = 0; i < 350; ++i)
        {
            const double position = direction * (i % 40 < 30 ? 0.6868 : 0.6903);
            const double elapsed = close.elapsed;
            close.update(position, 0.0, direction * 0.45, 0.01);
            ASSERT_FALSE(close.hasObject);
            EXPECT_GE(close.elapsed, elapsed); // no repeated deadline resets
            EXPECT_NE(close.state, ImpedanceClose::State::Closed); // not at the endpoint
            EXPECT_LE(direction * (close.target - position), config.lead + 1e-12);
        }
        ASSERT_EQ(close.state, ImpedanceClose::State::TorqueHold);
        EXPECT_NEAR(close.elapsed, 2.0, 1e-9);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.45);
        EXPECT_DOUBLE_EQ(close.command().kp, 0.0);

        // Actual endpoint settling still selects the 0.25 Nm bias.
        auto settled = close;
        for (int i = 0; i < 25; ++i)
            settled.update(direction * 0.698, 0.0, direction * 0.45, 0.01);
        EXPECT_EQ(settled.state, ImpedanceClose::State::Closed);
        EXPECT_DOUBLE_EQ(settled.command().torque, direction * 0.25);
        for (bool stale : {false, true})
        {
            auto fault = close;
            EXPECT_EQ(fault.update(direction * 0.687, 0.0,
                                   stale ? 0.45 : std::numeric_limits<double>::quiet_NaN(),
                                   stale ? 0.3 : 0.01), ImpedanceClose::State::Failed);
            EXPECT_DOUBLE_EQ(fault.command().torque, 0.0);
        }

        // A genuine reopening beyond the existing margin + lead rearms inference.
        ASSERT_EQ(close.update(direction * 0.64, 0.0, 0.0, 0.05),
                  ImpedanceClose::State::Closing);
        for (int i = 0; i < 150; ++i)
            close.update(direction * 0.64, 0.0, direction * 0.8, 0.01);
        EXPECT_TRUE(close.hasObject);
        EXPECT_EQ(close.state, ImpedanceClose::State::TorqueHold);
        EXPECT_NEAR(close.command().torque, direction * 2.0, 1e-9);
    }
}

TEST(ImpedanceCloseTest, ClosedStateRequiresSettlingButFloorSurvivesTimeoutAndRecoveryNotOpening)
{
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose close(direction * 0.698, direction * 0.7, 1.5, {});
        for (int i = 0; i < 40; ++i)
        {
            close.update(direction * 0.698, direction * 0.1, direction * 0.1, 0.05);
            EXPECT_NE(close.state, ImpedanceClose::State::Closed);
        }
        ASSERT_EQ(close.state, ImpedanceClose::State::TorqueHold);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        for (int i = 0; i < 5; ++i)
            close.update(direction * 0.698, 0.0, direction * 0.1, 0.05);
        ASSERT_EQ(close.state, ImpedanceClose::State::Closed);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        EXPECT_EQ(close.update(direction * 0.64, 0.0, 0.25, 0.05),
                  ImpedanceClose::State::Closing);
        EXPECT_DOUBLE_EQ(close.target, direction * 0.64);
        EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
        EXPECT_DOUBLE_EQ(close.integralTorque, 0.0);
        ImpedanceClose opening(direction * 0.698, 0.0, 10.0, {}, true);
        opening.update(direction * 0.698, 0.0, direction * 0.25, 0.01);
        EXPECT_DOUBLE_EQ(opening.command().torque, 0.0);
        EXPECT_DOUBLE_EQ(opening.command().kp, opening.config.kp);
    }
}

TEST(ImpedanceCloseTest, ClosedBiasUsesCalibratedDirectionAtOrBeyondEndpointAndFaultsStillLatch)
{
    for (double direction : {1.0, -1.0})
        for (double position : {0.7, 0.702})
            for (bool stale : {false, true})
            {
                ImpedanceClose close(direction * position, direction * 0.7, 1.5, {}, false, direction);
                for (int i = 0; i < 10; ++i)
                    close.update(direction * position, 0.0, 0.0, 0.05);
                ASSERT_EQ(close.state, ImpedanceClose::State::Closed);
                EXPECT_DOUBLE_EQ(close.command().torque, direction * 0.25);
                EXPECT_EQ(close.update(direction * position, 0.0,
                                       stale ? 0.25 : std::numeric_limits<double>::quiet_NaN(),
                                       stale ? 0.3 : 0.05), ImpedanceClose::State::Failed);
                EXPECT_DOUBLE_EQ(close.command().torque, 0.0);
                EXPECT_EQ(close.update(direction * position, 0.0, 0.25, 0.05),
                          ImpedanceClose::State::Failed);
            }
}

TEST(ImpedanceCloseTest, ClosedBiasAllowsZeroAndRejectsInvalidSettings)
{
    auto config = ImpedanceClose::Config{};
    for (double invalid : {-0.1, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()})
    {
        config.closedHoldTorque = invalid;
        EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.5, config), std::invalid_argument);
        EXPECT_THROW(ImpedanceClose(0.0, 0.7, 1.5, {}, false, invalid), std::invalid_argument);
    }
    config.closedHoldTorque = 0.0;
    ImpedanceClose close(0.698, 0.7, 1.5, config);
    for (int i = 0; i < 50; ++i)
        close.update(0.698, 0.0, 0.1, 0.05);
    EXPECT_EQ(close.state, ImpedanceClose::State::Closed);
    EXPECT_DOUBLE_EQ(close.command().torque, 0.0);
}

TEST(ImpedanceCloseTest, ClosingFloorClampsCombinedEffortNotJustFeedforward)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.maxStep = 0.01;
    for (double direction : {1.0, -1.0})
        for (double floor : {0.0, 0.25, 2.5})
            for (double cap : {0.1, 2.0})
                for (double error : {-0.02, 0.0, 0.02, 0.04})
                    for (double velocity : {-8.0, -0.1, 0.0, 0.1, 8.0})
                    {
                        config.closedHoldTorque = floor;
                        config.torqueLimit = cap;
                        ImpedanceClose close(direction * 0.3, direction * 0.75, 2.0, config);
                        const auto initial = close.command();
                        EXPECT_DOUBLE_EQ(initial.torque, direction * std::min(floor, cap));
                        close.update(direction * 0.3, direction * velocity, 0.0, 0.01);
                        close.target = direction * (0.3 + error);
                        const auto command = close.command();
                        const double pd = config.kp * (close.target - close.previousPosition) -
                                          config.kd * direction * velocity + close.integralTorque;
                        const double total = command.kp * (command.position - close.previousPosition) -
                                             command.kd * direction * velocity + command.torque;
                        const bool braking = velocity > close.speed || error < -config.tolerance;
                        const double minimum = floor > 0.0 && !braking ? std::min(floor, cap) : -cap;
                        EXPECT_NEAR(std::clamp(direction * total, -cap, cap),
                                    std::clamp(direction * pd, minimum, cap), 1e-12);
                        EXPECT_LE(std::abs(command.torque), cap);
                        if (floor > 0.0 && !braking && direction * pd < std::min(floor, cap))
                        {
                            EXPECT_DOUBLE_EQ(command.kp, 0.0);
                            EXPECT_DOUBLE_EQ(command.kd, 0.0);
                            EXPECT_DOUBLE_EQ(command.torque, direction * std::min(floor, cap));
                        }
                        else
                        {
                            EXPECT_DOUBLE_EQ(command.kp, config.kp);
                            EXPECT_DOUBLE_EQ(command.kd, config.kd);
                            EXPECT_DOUBLE_EQ(command.torque, close.integralTorque); // no extra bias
                        }
                        EXPECT_EQ(close.update(direction * 0.3, 0.0, 0.0, 0.3),
                                  ImpedanceClose::State::Failed);
                        EXPECT_DOUBLE_EQ(close.command().torque, 0.0);
                        EXPECT_DOUBLE_EQ(close.command().kp, 0.0);
                        EXPECT_DOUBLE_EQ(close.command().kd, 0.0);
                    }
}

TEST(ImpedanceCloseTest, ClosingFloorPreventsLowerBoundWindupAndFloorsAllTorqueHolds)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.torqueLimit = 2.0;
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose close(direction * 0.3, direction * 0.75, 2.0, config);
        close.update(direction * 0.35, 0.0, 0.0, 0.01); // overshoot makes PD + I negative
        EXPECT_LT(direction * close.integralTorque, 0.0); // braking is not floor-limited
        EXPECT_DOUBLE_EQ(close.command().kp, config.kp);
        close.update(direction * 0.35, 0.0, 0.0, 0.01);
        close.update(direction * 0.35, 0.0, 0.0, 0.01);
        close.update(direction * 0.35, 0.0, 0.0, 0.01);
        EXPECT_GT(direction * close.integralTorque, 0.0); // can integrate out of the floor

        ImpedanceClose floorBound(direction * 0.3, direction * 0.75, 2.0, config);
        floorBound.update(direction * 0.322, 0.0, 0.0, 0.01);
        EXPECT_DOUBLE_EQ(floorBound.integralTorque, 0.0); // no windup while floor is active
        EXPECT_DOUBLE_EQ(floorBound.command().torque, direction * 0.25);
        floorBound.update(direction * 0.322, 0.0, 0.0, 0.01);
        EXPECT_GT(direction * floorBound.integralTorque, 0.0);

        config.holdTorque = 0.1; // the closing floor also bounds a lower object-hold request
        ImpedanceClose object(direction * 0.3, direction * 0.75, 2.0, config);
        for (int i = 0; i < 100; ++i)
        {
            object.update(direction * 0.3, 0.0, direction * 0.8, 0.01);
            if (object.hasObject)
                EXPECT_GE(direction * object.command().torque, 0.25);
        }
        ASSERT_TRUE(object.hasObject);
        EXPECT_EQ(object.state, ImpedanceClose::State::TorqueHold);
        EXPECT_DOUBLE_EQ(object.command().torque, direction * 0.25);

        config.closedHoldTorque = 0.0; // explicit opt-out preserves the old zero capture
        ImpedanceClose legacy(direction * 0.3, direction * 0.75, 2.0, config);
        legacy.elapsed = 1.99;
        legacy.update(direction * 0.3, 0.0, 0.0, 0.02);
        EXPECT_EQ(legacy.state, ImpedanceClose::State::TorqueHold);
        EXPECT_DOUBLE_EQ(legacy.command().torque, 0.0);
        config.closedHoldTorque = 0.25;
    }
}

TEST(ImpedanceCloseTest, ClosingFloorMustNotSuppressBraking)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.torqueLimit = 2.0;
    for (double direction : {1.0, -1.0})
    {
        ImpedanceClose close(0.0, direction * 0.75, 2.0, config);
        // Unloaded 111: the old floor replaced this braking command with
        // +0.25 Nm, accelerating to 7.5 rad/s and past the travel boundary.
        close.previousPosition = direction * 0.046211;
        close.previousVelocity = direction * 2.482;
        close.target = direction * 0.06;
        auto command = close.command();
        EXPECT_DOUBLE_EQ(command.kp, config.kp);
        EXPECT_DOUBLE_EQ(command.kd, config.kd);
        EXPECT_LT(direction * (command.kp * (command.position - close.previousPosition) -
                              command.kd * close.previousVelocity + command.torque), 0.0);
        // Position progress also restores braking if velocity reads zero.
        close.previousPosition = direction * 0.09;
        close.previousVelocity = 0.0;
        command = close.command();
        EXPECT_DOUBLE_EQ(command.kp, config.kp);
        EXPECT_LT(direction * command.kp * (command.position - close.previousPosition), 0.0);
    }
}

TEST(ImpedanceCloseTest, UnloadedClosingPlantKeepsBrakingAndSettlesWithoutRunaway)
{
    auto config = ImpedanceClose::Config{};
    config.kp = 30.0;
    config.kd = 0.2;
    config.ki = 5.0;
    config.torqueLimit = 2.0;
    for (double direction : {1.0, -1.0})
        for (double inertia : {0.003, 0.006, 0.01})
        {
            // Simple light-jaw plant with compliant fingertip contact. This is
            // a runaway regression, not a hardware speed/force certification.
            ImpedanceClose close(0.0, direction * 0.75, 2.0, config);
            double position = 0.0, velocity = 0.0, torque = 0.0;
            double peakSpeed = 0.0, peakPosition = 0.0;
            for (int tick = 0; tick < 150; ++tick)
            {
                close.update(direction * position, direction * velocity,
                             direction * torque, 0.01);
                const auto command = close.command();
                for (int step = 0; step < 100; ++step)
                {
                    torque = std::clamp(direction * (command.kp *
                        (command.position - direction * position) -
                        command.kd * direction * velocity + command.torque), -2.0, 2.0);
                    const double contact = position > 0.733 ?
                        std::max(0.0, 80.0 * (position - 0.733) + 0.5 * velocity) : 0.0;
                    velocity += (torque - 0.02 * velocity - contact) / inertia * 0.0001;
                    position += velocity * 0.0001;
                    peakSpeed = std::max(peakSpeed, velocity);
                    peakPosition = std::max(peakPosition, position);
                }
            }
            EXPECT_LT(peakSpeed, 3.5);
            EXPECT_LT(peakPosition, 0.75);
            EXPECT_GT(position, 0.73);
            EXPECT_LT(std::abs(velocity), 0.05);
        }
}
