#include <gtest/gtest.h>
#include <array>

#include "candle_ros2/impedance_home.hpp"

namespace
{
ImpedanceClose::Config homeMotion()
{
    ImpedanceClose::Config motion;
    motion.torqueLimit = 2.0;
    motion.maxStep = .01;
    return motion;
}
}

TEST(ImpedanceHome, SearchesPastFormerTravelBoundInEitherDirection)
{
    for (double direction : {-1.0, 1.0})
    {
        ImpedanceHome home(.83134, direction, {}, homeMotion());
        for (double travel : {.2, .6, .85144, 1.2})
        {
            EXPECT_EQ(home.update(.83134 + direction * travel, direction, direction, .01),
                      ImpedanceHome::State::Searching);
            EXPECT_DOUBLE_EQ(home.command().torque, direction);
        }
        for (int i = 0; i < 26; ++i)
            home.update(.83134 + direction * 1.2, 0, direction, .01);
        EXPECT_EQ(home.state, ImpedanceHome::State::Found);
        EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
    }
}

TEST(ImpedanceHome, RemovalOfTravelCapRetainsFeedbackTorqueAndDirectionGuards)
{
    for (const auto& sample : std::array<std::array<double, 4>, 8>{{
             {NAN, 0, 0, .01}, {0, NAN, 0, .01}, {0, 0, NAN, .01},
             {0, 0, 0, .3}, {0, 0, 0, 0}, {0, 0, 2.11, .01},
             {0, 0, 0, NAN}, {-.006, 0, 0, .01}}})
    {
        ImpedanceHome home(0, 1, {}, homeMotion());
        EXPECT_EQ(home.update(sample[0], sample[1], sample[2], sample[3]),
                  ImpedanceHome::State::Failed);
        EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
        EXPECT_EQ(home.update(0, 0, 0, .01), ImpedanceHome::State::Failed);
    }
    ImpedanceHome timedOut(0, 1, {}, homeMotion());
    for (int i = 1; i <= 210; ++i) timedOut.update(i * .02, 2, 1, .01);
    EXPECT_EQ(timedOut.state, ImpedanceHome::State::Failed);
    EXPECT_DOUBLE_EQ(timedOut.command().torque, 0.0);
}

TEST(ImpedanceHome, DisabledReleaseIgnoresPresetPositionBoundsButStillRequiresUnloadedDwell)
{
    ImpedanceHome home(0, -1, {}, homeMotion());
    home.update(-.1, 0, -1, .01);
    for (int i = 0; i < 26; ++i) home.update(-.1, 0, -1, .01);
    ASSERT_EQ(home.state, ImpedanceHome::State::Found);
    home.beginRelease();
    for (int i = 0; i < 26; ++i)
        EXPECT_EQ(home.updateRelease(-.08, 0, .11, .01), ImpedanceHome::State::Releasing);
    for (int i = 0; i < 26; ++i) home.updateRelease(-.08, 0, 0, .01);
    EXPECT_EQ(home.state, ImpedanceHome::State::Ready);
    EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
}

TEST(ImpedanceHome, StationaryIdleOrWrongDirectionEffortCannotEstablishAStop)
{
    for (double direction : {-1.0, 1.0})
    {
        for (double signedTorque : {0.004, 0.499, -1.0})
        {
            ImpedanceHome home(0, direction, {}, homeMotion());
            for (int i = 0; i < 100; ++i)
                EXPECT_EQ(home.update(0, 0, direction * signedTorque, .01),
                          ImpedanceHome::State::Searching);
            for (int i = 0; i < 110; ++i) home.update(0, 0, direction * signedTorque, .01);
            EXPECT_EQ(home.state, ImpedanceHome::State::Failed);
            EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
        }
    }
}

TEST(ImpedanceHome, LossOfMeasuredEffortRestartsEntireStopDwell)
{
    for (double direction : {-1.0, 1.0})
    {
        auto config = ImpedanceHome::Config{};
        config.seekTorque = .8;
        ImpedanceHome home(0, direction, config, homeMotion());
        for (int i = 0; i < 24; ++i)
            EXPECT_EQ(home.update(0, 8, direction * .4, .01), ImpedanceHome::State::Searching);
        EXPECT_EQ(home.update(0, 8, direction * .399, .01), ImpedanceHome::State::Searching);
        for (int i = 0; i < 24; ++i)
            EXPECT_EQ(home.update(0, 8, direction * .4, .01), ImpedanceHome::State::Searching);
        EXPECT_EQ(home.update(0, 8, direction * .4, .01), ImpedanceHome::State::Found);
    }
}

TEST(ImpedanceHome, PassiveReleaseDeadlineIsSeparateAndMustAllowTheDwell)
{
    for (double invalid : std::array<double, 5>{0, -.1, NAN, INFINITY, .25})
    {
        auto config = ImpedanceHome::Config{};
        config.releaseTimeout = invalid;
        EXPECT_THROW(ImpedanceHome(0, -1, config, homeMotion()), std::invalid_argument);
    }
    ImpedanceHome home(0, -1, {}, homeMotion());
    EXPECT_DOUBLE_EQ(home.config.timeout, 2.0);
    EXPECT_DOUBLE_EQ(home.config.releaseTimeout, 3.0);
    home.beginRelease();
    for (int i = 0; i < 299; ++i)
    {
        EXPECT_EQ(home.updateRelease(-.00633, 0, .11, .01), ImpedanceHome::State::Releasing);
        EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
    }
    for (int i = 0; i < 2; ++i) home.updateRelease(-.00633, 0, .11, .01);
    EXPECT_EQ(home.state, ImpedanceHome::State::Failed);
    EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
}

TEST(ImpedanceHome, UnloadedDwellIsIndependentOfAbsolutePosition)
{
    for (double position : {-.00633, .01963, .2})
    {
        ImpedanceHome home(position, -1, {}, homeMotion());
        home.beginRelease();
        for (int i = 0; i < 24; ++i)
            EXPECT_EQ(home.updateRelease(position, 0, 0, .01), ImpedanceHome::State::Releasing);
        EXPECT_EQ(home.updateRelease(position, 0, 0, .01), ImpedanceHome::State::Ready);
        EXPECT_DOUBLE_EQ(home.previousPosition, position);
        EXPECT_DOUBLE_EQ(home.command().torque, 0.0);
    }
}
