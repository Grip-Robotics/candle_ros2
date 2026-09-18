#include <gtest/gtest.h>
#include <algorithm>
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
