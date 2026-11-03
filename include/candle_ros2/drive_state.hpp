#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <thread>

namespace candle_ros2
{
// MD's documented maxVelocity range is [0.0001, 1500] rad/s; there is no
// documented off value. Keep target ramps separate from this firmware ceiling.
// This is a volatile limit, never a speed command or a flash-save operation.
inline constexpr float gripperFirmwareVelocityCeilingRadS = 1500.0f;

template <class Drive>
bool configureGripperLimits(Drive& md, float profileVelocity, float torqueLimit)
{
    using Error = typename Drive::Error_t;
    auto& regs = md.m_mdRegisters;
    regs.profileVelocity = profileVelocity;
    regs.maxVelocity = gripperFirmwareVelocityCeilingRadS;
    regs.maxTorque = torqueLimit;
    if (md.writeRegisters(regs.profileVelocity, regs.maxVelocity, regs.maxTorque) != Error::OK ||
        md.readRegisters(regs.profileVelocity, regs.maxVelocity, regs.maxTorque) != Error::OK)
        return false;
    return std::isfinite(regs.profileVelocity.value) &&
           std::isfinite(regs.maxVelocity.value) && std::isfinite(regs.maxTorque.value) &&
           std::abs(regs.profileVelocity.value - profileVelocity) <= 1e-6f &&
           std::abs(regs.maxVelocity.value - gripperFirmwareVelocityCeilingRadS) <= 1e-6f &&
           std::abs(regs.maxTorque.value - torqueLimit) <= 1e-6f;
}

// Call only after zero has been acknowledged with PWM disabled. Some firmware
// updates the encoder register on its next control tick, not in the command ACK.
template <class Drive>
std::optional<double> readZeroPosition(Drive& md, double tolerance)
{
    using Error = typename Drive::Error_t;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
    do
    {
        const auto [raw, error] = md.getPosition();
        if (error != Error::OK || !std::isfinite(raw)) return std::nullopt;
        if (std::abs(raw) <= tolerance) return raw;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    return std::nullopt;
}

// MD protocol state register uses the CiA402 state bits. Ignore flags such
// as target reached, but never mistake switched-on/quick-stop/fault for enabled.
inline bool driveEnabled(std::uint16_t state)
{
    return (state & 0x006f) == 0x0027;
}

inline bool driveDisabled(std::uint16_t state)
{
    return (state & 0x004f) == 0x0040;
}

// Disabling enters quick-stop before removing power. Poll without sleeping so
// the node can keep observing position and servicing the other drives. Only a
// failed transfer loses communication; normal braking (or a refused transition)
// must not invalidate the encoder reference. The deadline never means success.
struct DisableTransition
{
    using Clock = std::chrono::steady_clock;
    enum class Status { Pending, Disabled, Failed };
    static constexpr auto timeout = std::chrono::milliseconds(500);
    Clock::time_point started;
    Status status = Status::Pending;
    bool communicationLost = false;

    template <class Drive>
    explicit DisableTransition(Drive& md, Clock::time_point now = Clock::now()) : started(now)
    {
        using Error = typename Drive::Error_t;
        if (md.disable() != Error::OK)
        {
            communicationLost = true;
            status = Status::Failed;
        }
    }

    template <class Drive>
    Status poll(Drive& md, Clock::time_point now = Clock::now())
    {
        if (status != Status::Pending)
            return status;
        using Error = typename Drive::Error_t;
        auto& regs = md.m_mdRegisters;
        if (md.readRegisters(regs.state, regs.motionModeStatus) != Error::OK)
        {
            communicationLost = true;
            status = Status::Failed;
        }
        else if (driveDisabled(regs.state.value))
            status = Status::Disabled;
        else if ((regs.state.value & 0x0008) != 0 || now - started >= timeout)
            status = Status::Failed;
        return status;
    }
};

// Enable is a short transition, unlike quick-stop. Never reissue the write or
// clear faults while waiting for enabled state AND the requested mode.
template <class Drive>
bool enableDrive(Drive& md, std::uint8_t expectedMode)
{
    using Error = typename Drive::Error_t;
    if (md.enable() != Error::OK)
        return false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(10);
    do
    {
        auto& regs = md.m_mdRegisters;
        if (md.readRegisters(regs.state, regs.motionModeStatus) != Error::OK)
            return false;
        if (driveEnabled(regs.state.value) && regs.motionModeStatus.value == expectedMode)
            return true;
        if ((regs.state.value & 0x0008) != 0) // fault/fault reaction: no retry
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
}
} // namespace candle_ros2
