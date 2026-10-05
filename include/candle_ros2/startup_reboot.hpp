#pragma once
#include "candle_ros2/drive_state.hpp"

namespace candle_ros2
{
// Startup only, before any MD is registered or controlled. No enable, zero,
// homing, flash-save or repeated reset. Sleep injection keeps tests hardware-free.
template<class Drive, class Sleep>
bool prepareStartupReboot(Drive& md, Sleep sleep)
{
    using Error = typename Drive::Error_t;
    auto& state = md.m_mdRegisters.state;
    if (md.readRegister(state) != Error::OK) return false;
    const auto stopped = [&] {
        return driveDisabled(state.value) || (state.value & 0x004f) == 0x0008;
    };
    if (stopped()) return true; // A latched FAULT cannot complete normal disable.
    if (md.disable() != Error::OK) return false;
    for (int i = 0; i < 50; ++i)
    {
        if (md.readRegister(state) != Error::OK) return false;
        if (stopped()) return true;
        sleep(std::chrono::milliseconds(10));
    }
    return false;
}

template<class Drive, class Sleep>
bool rebootStartupDrive(Drive& md, Sleep sleep)
{
    using Error = typename Drive::Error_t;
    auto& r = md.m_mdRegisters;
    // Recheck immediately before the only reset write.
    if (md.readRegister(r.state) != Error::OK ||
        !(driveDisabled(r.state.value) || (r.state.value & 0x004f) == 0x0008)) return false;
    const bool acknowledged = md.reset() == Error::OK;
    sleep(std::chrono::milliseconds(3000));
    for (int i = 0; i < 50; ++i)
    {
        // Single-register reads avoid oversized SDK frames. A boot-time read
        // failure never counts as a zero register or as successful recovery.
        if (md.readRegister(r.state) == Error::OK &&
            md.readRegister(r.motionModeStatus) == Error::OK &&
            md.readRegister(r.quickStatus) == Error::OK &&
            md.readRegister(r.targetTorque) == Error::OK &&
            md.readRegister(r.canWatchdog) == Error::OK &&
            driveDisabled(r.state.value) && r.motionModeStatus.value == 0 &&
            r.quickStatus.value == 0 && std::isfinite(r.targetTorque.value) &&
            r.targetTorque.value == 0 && r.canWatchdog.value > 0 && r.canWatchdog.value <= 250)
            return acknowledged;
        sleep(std::chrono::milliseconds(100));
    }
    return false;
}
} // namespace candle_ros2
