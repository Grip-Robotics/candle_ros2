#include <gtest/gtest.h>
#include <array>
#include <limits>

#include "candle_ros2/gripper_homing.hpp"

namespace
{
ImpedanceClose::Config motion(double closingDirection = 1.0)
{
    ImpedanceClose::Config result;
    result.torqueLimit = 2.0;
    result.maxStep = .01;
    result.travelMin = closingDirection > 0.0 ? -.05 : -.8;
    result.travelMax = closingDirection > 0.0 ? .8 : .05;
    return result;
}

void dwell(GripperHoming& home, double position, double torque = 0.0)
{
    for (int i = 0; i < 26; ++i) home.update(position, 0.0, torque, .01);
}

void findStop(GripperHoming& home, double position)
{
    home.update(position, 0.0, home.control.direction, .01);
    dwell(home, position, home.control.direction);
    ASSERT_EQ(home.control.state, ImpedanceHome::State::Found);
}

void release(GripperHoming& home, double position)
{
    ASSERT_TRUE(home.beginRelease());
    EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
    EXPECT_DOUBLE_EQ(home.control.command().kp, 0.0);
    EXPECT_DOUBLE_EQ(home.control.command().kd, 0.0);
    home.update(position, 0.0, 0.0, .01);
    dwell(home, position);
    ASSERT_EQ(home.control.state, ImpedanceHome::State::Ready);
}

void openAndZero(GripperHoming& home)
{
    const double stop = home.control.start + home.control.direction * .2;
    findStop(home, stop);
    release(home, stop - home.control.direction * .01);
    ASSERT_EQ(home.phase, GripperHoming::Phase::Opening);
    EXPECT_FALSE(home.calibration());
    ASSERT_TRUE(home.startClosing(0.0));
}

void closedAndReleased(GripperHoming& home, double closedPosition)
{
    findStop(home, closedPosition);
    EXPECT_FALSE(home.calibration());
    release(home, closedPosition - home.control.direction * .02);
    ASSERT_EQ(home.phase, GripperHoming::Phase::Closing);
    EXPECT_FALSE(home.calibration());
}
}

TEST(GripperHoming, CalibratesBothDirectionsAfterUnloadedReopening)
{
    for (double direction : {-1.0, 1.0})
    {
        GripperHoming home(12.0, -direction, {}, motion(direction), .5);
        openAndZero(home);
        EXPECT_DOUBLE_EQ(home.control.command().torque, direction);
        const double loadedStop = direction * .77;
        closedAndReleased(home, loadedStop);
        EXPECT_FALSE(home.calibration());
        ASSERT_TRUE(home.startReopening(direction * .75));
        EXPECT_DOUBLE_EQ(home.control.command().torque, -direction);
        findStop(home, direction * -.003);
        EXPECT_FALSE(home.calibration());
        release(home, direction * .002);
        ASSERT_EQ(home.phase, GripperHoming::Phase::Ready);
        const auto calibration = home.calibration();
        ASSERT_TRUE(calibration);
        EXPECT_DOUBLE_EQ(calibration->closedPosition, loadedStop);
        EXPECT_DOUBLE_EQ(calibration->openPosition, direction * -.003);
        EXPECT_DOUBLE_EQ(direction > 0.0 ? calibration->motion.travelMax : calibration->motion.travelMin,
                         direction * 1.27);
        EXPECT_DOUBLE_EQ(direction > 0.0 ? calibration->motion.travelMin : calibration->motion.travelMax,
                         direction * -.05);
        EXPECT_TRUE(calibration->motion.withinTravel(direction * 1.27));
        EXPECT_FALSE(calibration->motion.withinTravel(direction * 1.27001));
        EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
    }
}

TEST(GripperHoming, DifferentDrivesKeepIndependentLoadedStopsAndMargins)
{
    GripperHoming first(-4, -1, {}, motion(), .5);
    GripperHoming second(8, -1, {}, motion(), 0.0);
    for (auto [home, stop] : {std::pair{&first, .65}, std::pair{&second, .82}})
    {
        openAndZero(*home);
        closedAndReleased(*home, stop);
        ASSERT_TRUE(home->startReopening(stop - .02));
        findStop(*home, 0.0);
        release(*home, 0.0);
    }
    ASSERT_TRUE(first.calibration());
    ASSERT_TRUE(second.calibration());
    EXPECT_DOUBLE_EQ(first.calibration()->closedPosition, .65);
    EXPECT_DOUBLE_EQ(first.calibration()->motion.travelMax, 1.15);
    EXPECT_DOUBLE_EQ(second.calibration()->closedPosition, .82);
    EXPECT_DOUBLE_EQ(second.calibration()->motion.travelMax, .82);
}

TEST(GripperHoming, RejectsInvalidMarginAndBounds)
{
    for (double margin : std::array<double, 3>{-0.1, NAN, INFINITY})
        EXPECT_THROW(GripperHoming(0, -1, {}, motion(), margin), std::invalid_argument);
    auto invalid = motion();
    invalid.travelMin = .01;
    EXPECT_THROW(GripperHoming(0, -1, {}, invalid, .5), std::invalid_argument);
    invalid = motion();
    invalid.travelMax = NAN;
    EXPECT_THROW(GripperHoming(0, -1, {}, invalid, .5), std::invalid_argument);
}

TEST(GripperHoming, RejectsOverflowInDerivedTravelBound)
{
    ImpedanceHome::Config config;
    GripperHoming home(.2, -1, config, motion(), 1e308);
    openAndZero(home);
    dwell(home, 1e308, 1.0);
    dwell(home, 1e308, 1.0);
    EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
    EXPECT_FALSE(home.calibration());
}

TEST(GripperHoming, InvalidTransitionsCannotProduceCalibrationOrRestart)
{
    for (int transition = 0; transition < 3; ++transition)
    {
        GripperHoming home(.2, -1, {}, motion(), .5);
        const bool accepted = transition == 0 ? home.beginRelease() :
            transition == 1 ? home.startClosing(0) : home.startReopening(0);
        EXPECT_FALSE(accepted);
        EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
        EXPECT_FALSE(home.calibration());
        EXPECT_FALSE(home.startClosing(0));
        EXPECT_EQ(home.update(0, 0, 0, .01), ImpedanceHome::State::Failed);
        EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
    }
    GripperHoming invalidZero(.2, -1, {}, motion(), .5);
    findStop(invalidZero, 0);
    release(invalidZero, 0);
    EXPECT_FALSE(invalidZero.startClosing(.006));
    EXPECT_EQ(invalidZero.phase, GripperHoming::Phase::Failed);
}

TEST(GripperHoming, RejectsMissingShortAndWrongDirectionClosedTravel)
{
    for (double position : {0.0, .005, .01, -.004})
    {
        GripperHoming home(.2, -1, {}, motion(), .5);
        openAndZero(home);
        dwell(home, position, 1.0);
        dwell(home, position, 1.0);
        EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
        EXPECT_FALSE(home.calibration());
        EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
    }
}

TEST(GripperHoming, SearchGuardsApplyDuringClosingAndReopening)
{
    for (bool reopening : {false, true})
    {
        for (const auto& sample : std::array<std::array<double, 4>, 7>{{
                 {NAN, 0, 0, .01}, {.1, NAN, 0, .01}, {.1, 0, NAN, .01},
                 {.1, 0, 0, .3}, {.1, 0, 0, 0}, {.1, 0, 2.11, .01},
                 {.1, 0, 0, NAN}}})
        {
            GripperHoming home(.2, -1, {}, motion(), .5);
            openAndZero(home);
            if (reopening)
            {
                closedAndReleased(home, .75);
                ASSERT_TRUE(home.startReopening(.73));
            }
            EXPECT_EQ(home.update(sample[0], sample[1], sample[2], sample[3]),
                      ImpedanceHome::State::Failed);
            EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
            EXPECT_FALSE(home.calibration());
        }
    }
}

TEST(GripperHoming, ClosingSearchStillTimesOutWithoutAStop)
{
    GripperHoming home(.2, -1, {}, motion(), .5);
    openAndZero(home);
    for (int i = 1; i <= 210; ++i) home.update(i * .001, .1, 1, .01);
    EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
    EXPECT_FALSE(home.calibration());
}

TEST(GripperHoming, ClosedReleaseAndReopeningNeedFreshUnloadedDwell)
{
    for (bool reopening : {false, true})
    {
        GripperHoming home(.2, -1, {}, motion(), .5);
        openAndZero(home);
        if (reopening)
        {
            closedAndReleased(home, .75);
            ASSERT_TRUE(home.startReopening(.73));
            findStop(home, 0.0);
        }
        else findStop(home, .75);
        ASSERT_TRUE(home.beginRelease());
        const double position = reopening ? 0 : .75;
        for (int i = 0; i < 310; ++i) home.update(position, 0, .11, .01);
        EXPECT_EQ(home.phase, GripperHoming::Phase::Failed);
        EXPECT_FALSE(home.calibration());
    }
}

TEST(GripperHoming, InvalidReleaseFeedbackNeverPublishesCalibration)
{
    for (bool reopening : {false, true})
    {
        for (const auto& sample : std::array<std::array<double, 4>, 5>{{
                 {NAN, 0, 0, .01}, {.1, NAN, 0, .01}, {.1, 0, NAN, .01},
                 {.1, 0, 0, .3}, {.1, 0, 0, NAN}}})
        {
            GripperHoming home(.2, -1, {}, motion(), .5);
            openAndZero(home);
            if (reopening)
            {
                closedAndReleased(home, .75);
                ASSERT_TRUE(home.startReopening(.73));
                findStop(home, 0.0);
            }
            else findStop(home, .75);
            ASSERT_TRUE(home.beginRelease());
            EXPECT_EQ(home.update(sample[0], sample[1], sample[2], sample[3]),
                      ImpedanceHome::State::Failed);
            EXPECT_FALSE(home.calibration());
            EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
        }
    }
}

TEST(GripperHoming, ReopeningCannotSkipClosedReleaseOrUseInvalidStart)
{
    for (double position : std::array<double, 3>{NAN, INFINITY, -INFINITY})
    {
        GripperHoming home(.2, -1, {}, motion(), .5);
        openAndZero(home);
        closedAndReleased(home, .75);
        EXPECT_FALSE(home.startReopening(position));
        EXPECT_FALSE(home.calibration());
    }
    GripperHoming early(.2, -1, {}, motion(), .5);
    openAndZero(early);
    findStop(early, .75);
    EXPECT_FALSE(early.startReopening(.75));
    EXPECT_EQ(early.phase, GripperHoming::Phase::Failed);
    EXPECT_FALSE(early.calibration());
}

TEST(GripperHoming, LearnsLongerStrokeAndAllowsOpenStopBeyondFallbackBound)
{
    for (double direction : {-1.0, 1.0})
    {
        GripperHoming home(.2, -direction, {}, motion(direction), .5);
        openAndZero(home);
        closedAndReleased(home, direction * 1.2);
        ASSERT_TRUE(home.startReopening(direction * 1.18));
        // The loaded open stop can lie beyond the preset -0.05 rad bound.
        // Readiness still requires a complete unloaded settling window.
        findStop(home, direction * -.08);
        ASSERT_TRUE(home.beginRelease());
        EXPECT_EQ(home.update(direction * -.06, 0, 0, .01), ImpedanceHome::State::Releasing);
        EXPECT_FALSE(home.calibration());
        dwell(home, 0.0);
        ASSERT_EQ(home.phase, GripperHoming::Phase::Ready);
        ASSERT_TRUE(home.calibration());
        EXPECT_DOUBLE_EQ(home.calibration()->closedPosition, direction * 1.2);
        EXPECT_DOUBLE_EQ(home.calibration()->openPosition, direction * -.08);
        EXPECT_DOUBLE_EQ(direction > 0 ? home.calibration()->motion.travelMin :
                                         home.calibration()->motion.travelMax, direction * -.085);
        EXPECT_DOUBLE_EQ(direction > 0 ? home.calibration()->motion.travelMax :
                                         home.calibration()->motion.travelMin, direction * 1.7);
    }
}

TEST(GripperHoming, NoPartiallyLearnedGuardConstrainsClosedReleaseOrReopeningStart)
{
    for (double position : {-0.08, 1.4})
    {
        GripperHoming home(.2, -1, {}, motion(), .5);
        openAndZero(home);
        findStop(home, .75);
        release(home, position);
        EXPECT_FALSE(home.calibration());
        ASSERT_TRUE(home.startReopening(position));
        EXPECT_EQ(home.phase, GripperHoming::Phase::Reopening);
        EXPECT_FALSE(home.calibration());
    }
}

TEST(GripperHoming, FinalReopeningAcceptsReleasedOffsetWithoutChangingMeasuredStops)
{
    // Drive 345 settled at +0.01963 rad after its powered reopening stop.
    // That absolute offset must not reject otherwise complete calibration.
    for (double direction : {-1.0, 1.0})
    {
        for (double offset : {.01963, -.00633})
        {
            GripperHoming home(.2, -direction, {}, motion(direction), .5);
            openAndZero(home);
            closedAndReleased(home, direction * .81182);
            ASSERT_TRUE(home.startReopening(direction * .73999));
            findStop(home, direction * -.02707);
            release(home, direction * offset);
            ASSERT_EQ(home.phase, GripperHoming::Phase::Ready);
            ASSERT_TRUE(home.calibration());
            EXPECT_DOUBLE_EQ(home.calibration()->closedPosition, direction * .81182);
            EXPECT_DOUBLE_EQ(home.calibration()->openPosition, direction * -.02707);
            EXPECT_DOUBLE_EQ(home.control.previousPosition, direction * offset);
            EXPECT_DOUBLE_EQ(home.control.command().torque, 0.0);
            EXPECT_LT(home.control.elapsed, home.control.config.releaseTimeout);
        }
    }
}

TEST(GripperHoming, CalibrationPreservesDisabledTravelGuardAndLearnedEndpoints)
{
    for (double direction : {-1.0, 1.0})
    {
        auto settings = motion(direction);
        settings.travelLimitsEnabled = false;
        GripperHoming home(.2, -direction, {}, settings, .5);
        openAndZero(home);
        closedAndReleased(home, direction * .77);
        ASSERT_TRUE(home.startReopening(direction * .75));
        findStop(home, direction * -.105768);
        release(home, direction * -.11);
        const auto learned = home.calibration();
        ASSERT_TRUE(learned);
        EXPECT_FALSE(learned->motion.travelLimitsEnabled);
        EXPECT_DOUBLE_EQ(learned->closedPosition, direction * .77);
        EXPECT_DOUBLE_EQ(learned->openPosition, direction * -.105768);
        EXPECT_TRUE(learned->motion.withinTravel(direction * -1.0));
        EXPECT_TRUE(learned->motion.withinTravel(direction * 2.0));
    }
}
