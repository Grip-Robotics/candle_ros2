#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <utility>
#include <vector>
#include "candle_ros2/drive_state.hpp"

namespace
{
struct Drive
{
    enum class Error_t { OK, Failed };
    struct Register { unsigned value = 0; };
    struct { Register state, motionModeStatus; } m_mdRegisters;
    std::vector<std::pair<unsigned, unsigned>> feedback{{0x27, 4}};
    Error_t writeResult = Error_t::OK, readResult = Error_t::OK;
    unsigned enables = 0, disables = 0, reads = 0;
    std::vector<double> positions{0.0};
    std::pair<double, Error_t> getPosition()
    {
        return {positions[std::min<std::size_t>(reads++, positions.size()-1)], readResult};
    }
    Error_t enable() { ++enables; return writeResult; }
    Error_t disable() { ++disables; return writeResult; }
    Error_t readRegisters(Register& state, Register& mode)
    {
        const auto sample = feedback[std::min<std::size_t>(reads++, feedback.size() - 1)];
        state.value = sample.first;
        mode.value = sample.second;
        return readResult;
    }
};

struct LimitDrive
{
    enum class Error_t { OK, Failed };
    struct Register
    {
        float value = 0;
        Register& operator=(float next) { value = next; return *this; }
    };
    struct { Register profileVelocity, maxVelocity, maxTorque; } m_mdRegisters;
    std::array<float, 3> hardware{10, 10, 2};
    std::optional<std::array<float, 3>> forcedReadback;
    Error_t writeResult = Error_t::OK, readResult = Error_t::OK;
    unsigned writes = 0, reads = 0;
    Error_t writeRegisters(Register& profile, Register& ceiling, Register& torque)
    {
        ++writes;
        if (writeResult == Error_t::OK) hardware = {profile.value, ceiling.value, torque.value};
        return writeResult;
    }
    Error_t readRegisters(Register& profile, Register& ceiling, Register& torque)
    {
        ++reads;
        const auto values = forcedReadback.value_or(hardware);
        profile = values[0]; ceiling = values[1]; torque = values[2];
        return readResult;
    }
};
}

TEST(DriveStateTest, GripperVelocityCeilingDoesNotFollowOpeningClosingOrHomingRamp)
{
    LimitDrive md;
    // The shared configuration path serves opening, closing, homing and init.
    // Reconfiguring a slower target must not reinstate the old 10 rad/s trip.
    for (const auto [ramp, torque] : {std::pair{10.f, 2.f}, {2.f, 2.f},
                                      {10.f, 1.f}, {2.f, 2.f}})
    {
        ASSERT_TRUE(candle_ros2::configureGripperLimits(md, ramp, torque));
        EXPECT_FLOAT_EQ(md.hardware[0], ramp);
        EXPECT_FLOAT_EQ(md.hardware[1], 1500.f);
        EXPECT_FLOAT_EQ(md.hardware[2], torque);
    }
    EXPECT_EQ(md.writes, 4u);
    EXPECT_EQ(md.reads, 4u);
}

TEST(DriveStateTest, GripperLimitsRequireSuccessfulWriteAndReadback)
{
    LimitDrive md;
    md.writeResult = LimitDrive::Error_t::Failed;
    EXPECT_FALSE(candle_ros2::configureGripperLimits(md, 10, 2));
    EXPECT_EQ(md.reads, 0u);
    EXPECT_FLOAT_EQ(md.hardware[1], 10.f);
    md.writeResult = LimitDrive::Error_t::OK;
    md.readResult = LimitDrive::Error_t::Failed;
    EXPECT_FALSE(candle_ros2::configureGripperLimits(md, 10, 2));
    EXPECT_EQ(md.reads, 1u);
}

TEST(DriveStateTest, GripperLimitsRejectClampedStaleAndNonfiniteReadback)
{
    for (const auto readback : {std::array<float, 3>{10, 10, 2},
                               {10, 1500, 3}, {2, 1500, 2},
                               {NAN, 1500, 2}, {10, NAN, 2}, {10, 1500, NAN},
                               {10, INFINITY, 2}})
    {
        LimitDrive md;
        md.forcedReadback = readback;
        EXPECT_FALSE(candle_ros2::configureGripperLimits(md, 10, 2));
        EXPECT_EQ(md.writes, 1u);
        EXPECT_EQ(md.reads, 1u);
    }
}

TEST(DriveStateTest, StateFlagsDoNotHideDisabledQuickStopOrFault)
{
    for (unsigned flags : {0u, 0x10u, 0x400u, 0x800u})
    {
        EXPECT_TRUE(candle_ros2::driveEnabled(0x27 | flags));
        EXPECT_TRUE(candle_ros2::driveDisabled(0x40 | flags));
        for (unsigned state : {0u, 0x21u, 0x23u, 0x40u, 0x07u, 0x08u, 0x0fu})
            EXPECT_FALSE(candle_ros2::driveEnabled(state | flags));
        EXPECT_FALSE(candle_ros2::driveDisabled(0x27 | flags));
    }
}

TEST(DriveStateTest, ZeroReadbackWaitsButRejectsErrorsEvenWhenSdkReturnsZero)
{
    Drive md;
    md.positions = {-.2, -.2, .0001};
    ASSERT_TRUE(candle_ros2::readZeroPosition(md, .005));
    EXPECT_EQ(md.reads, 3u);
    md.positions = {0.0}; md.readResult = Drive::Error_t::Failed;
    EXPECT_FALSE(candle_ros2::readZeroPosition(md, .005));
    md.readResult = Drive::Error_t::OK; md.positions = {NAN};
    EXPECT_FALSE(candle_ros2::readZeroPosition(md, .005));
    md.positions = {-.2};
    EXPECT_FALSE(candle_ros2::readZeroPosition(md, .005));
}

TEST(DriveStateTest, WaitsForAcknowledgedEnableWithoutReissuingIt)
{
    Drive md;
    md.feedback = {{0x40, 4}, {0x23, 4}, {0x27, 4}};
    ASSERT_TRUE(candle_ros2::enableDrive(md, 4));
    EXPECT_EQ(md.enables, 1u);
    EXPECT_EQ(md.reads, 3u);
}

TEST(DriveStateTest, AcknowledgedButInactiveWrongModeOrFaultedIsNotSuccess)
{
    for (auto state : {std::pair{0x40u, 4u}, {0x27u, 0u}, {0x27u, 1u}, {0x08u, 4u}})
    {
        Drive md;
        md.feedback = {state};
        EXPECT_FALSE(candle_ros2::enableDrive(md, 4));
        EXPECT_EQ(md.enables, 1u);
    }
    Drive md;
    md.readResult = Drive::Error_t::Failed;
    EXPECT_FALSE(candle_ros2::enableDrive(md, 4));
    md.writeResult = Drive::Error_t::Failed;
    md.reads = 0;
    EXPECT_FALSE(candle_ros2::enableDrive(md, 4));
    EXPECT_EQ(md.reads, 0u);
}

TEST(DriveStateTest, SlowQuickStopIsPendingNotCommunicationLossOrSuccess)
{
    using Stop = candle_ros2::DisableTransition;
    using Status = Stop::Status;
    using namespace std::chrono_literals;
    Drive md;
    md.feedback = {{0x27, 8}, {0x07, 8}, {0x07, 8}, {0x40, 8}};
    const auto start = Stop::Clock::now();
    Stop stop(md, start);
    EXPECT_EQ(stop.poll(md, start), Status::Pending);
    EXPECT_EQ(stop.poll(md, start + 11ms), Status::Pending);
    EXPECT_EQ(stop.poll(md, start + 100ms), Status::Pending);
    EXPECT_FALSE(stop.communicationLost);
    EXPECT_EQ(stop.poll(md, start + 120ms), Status::Disabled);
    EXPECT_EQ(md.disables, 1u);
    EXPECT_EQ(md.reads, 4u); // one read per tick; no sleeps/spins/extra writes
    EXPECT_EQ(stop.poll(md, start + 130ms), Status::Disabled);
    EXPECT_EQ(md.reads, 4u);
}

TEST(DriveStateTest, DisableTimeoutFaultAndTransferFailureNeverConfirm)
{
    using Stop = candle_ros2::DisableTransition;
    using Status = Stop::Status;
    const auto start = Stop::Clock::now();
    for (unsigned state : {0x07u, 0x27u, 0x23u})
    {
        Drive md;
        md.feedback = {{state, 8}};
        Stop stop(md, start);
        EXPECT_EQ(stop.poll(md, start + Stop::timeout), Status::Failed);
        EXPECT_FALSE(stop.communicationLost); // a live, refused stop is NOT a lost encoder
        md.feedback = {{0x40, 0}};
        EXPECT_EQ(stop.poll(md), Status::Failed); // terminal result is latched
    }
    for (unsigned state : {0x08u, 0x0fu})
    {
        Drive md;
        md.feedback = {{state, 8}};
        Stop stop(md, start);
        EXPECT_EQ(stop.poll(md, start), Status::Failed);
        EXPECT_FALSE(stop.communicationLost);
    }
    Drive md;
    md.readResult = Drive::Error_t::Failed;
    Stop unreadable(md, start);
    EXPECT_EQ(unreadable.poll(md, start), Status::Failed);
    EXPECT_TRUE(unreadable.communicationLost);
    md.reads = 0;
    md.writeResult = Drive::Error_t::Failed;
    Stop unwritten(md, start);
    EXPECT_EQ(unwritten.poll(md, start), Status::Failed);
    EXPECT_TRUE(unwritten.communicationLost);
    EXPECT_EQ(md.reads, 0u);
}

#include "candle_ros2/startup_reboot.hpp"
namespace {
struct RebootDrive {
    enum class Error_t { OK, Failed };
    struct Register { unsigned value=0; };
    struct { Register state{8}, motionModeStatus, quickStatus{1}, targetTorque, canWatchdog{100}; } m_mdRegisters;
    unsigned resets=0, disables=0;
    bool readFails=false, resetFails=false, staysFaulted=false, disableFails=false, remainsEnabled=false;
    unsigned bootReadFailures=0;
    Error_t readRegister(Register&) {
        if (readFails) return Error_t::Failed;
        if (resets && bootReadFailures) { --bootReadFailures; return Error_t::Failed; }
        return Error_t::OK;
    }
    Error_t disable() {
        ++disables;
        if (disableFails) return Error_t::Failed;
        if (!remainsEnabled) m_mdRegisters.state.value=64;
        return Error_t::OK;
    }
    Error_t reset() {
        ++resets;
        if (!staysFaulted) { m_mdRegisters.state.value=64; m_mdRegisters.quickStatus.value=0; }
        return resetFails?Error_t::Failed:Error_t::OK;
    }
};
const auto noWait=[](auto) {};
}
TEST(StartupReboot, FaultEightRebootsOnceAndVerifiesDisabled) {
    RebootDrive md;
    EXPECT_TRUE(candle_ros2::prepareStartupReboot(md,noWait));
    EXPECT_TRUE(candle_ros2::rebootStartupDrive(md,noWait));
    EXPECT_EQ(md.disables,0u); EXPECT_EQ(md.resets,1u);
}
TEST(StartupReboot, EnabledDriveMustBeStoppedBeforeReset) {
    RebootDrive md; md.m_mdRegisters.state.value=39;
    EXPECT_FALSE(candle_ros2::rebootStartupDrive(md,noWait)); EXPECT_EQ(md.resets,0u);
    EXPECT_TRUE(candle_ros2::prepareStartupReboot(md,noWait)); EXPECT_EQ(md.disables,1u);
    EXPECT_TRUE(candle_ros2::rebootStartupDrive(md,noWait));
}
TEST(StartupReboot, FailedStopOrUnreadableStatePreventsReset) {
    RebootDrive md;md.readFails=true;
    EXPECT_FALSE(candle_ros2::prepareStartupReboot(md,noWait));
    EXPECT_FALSE(candle_ros2::rebootStartupDrive(md,noWait)); EXPECT_EQ(md.resets,0u);
    md.readFails=false;md.m_mdRegisters.state.value=39;md.remainsEnabled=true;
    EXPECT_FALSE(candle_ros2::prepareStartupReboot(md,noWait)); EXPECT_EQ(md.resets,0u);
}
TEST(StartupReboot, PersistentFaultAndLostAcknowledgementNeverSucceedOrRepeatReset) {
    for (bool fault : {true,false}) {
        RebootDrive md;md.staysFaulted=fault;md.resetFails=!fault;
        EXPECT_FALSE(candle_ros2::rebootStartupDrive(md,noWait)); EXPECT_EQ(md.resets,1u);
    }
}
TEST(StartupReboot, BootDelayIsBoundedAndRequiresValidModeTorqueAndWatchdog) {
    RebootDrive md;md.bootReadFailures=2;
    EXPECT_TRUE(candle_ros2::rebootStartupDrive(md,noWait));
    for (int failure=0;failure<5;++failure) {
        RebootDrive bad;
        if (failure==0) bad.m_mdRegisters.targetTorque.value=1;
        if (failure==1) bad.m_mdRegisters.motionModeStatus.value=4;
        if (failure==2) bad.m_mdRegisters.canWatchdog.value=0;
        if (failure==3) bad.m_mdRegisters.canWatchdog.value=500;
        if (failure==4) bad.bootReadFailures=100;
        EXPECT_FALSE(candle_ros2::rebootStartupDrive(bad,noWait)); EXPECT_EQ(bad.resets,1u);
    }
}
