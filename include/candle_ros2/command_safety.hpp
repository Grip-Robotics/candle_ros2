#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace candle_ros2
{
inline bool validDeviceId(std::uint32_t id)
{
    return id > 0 && id <= std::numeric_limits<std::uint16_t>::max();
}

inline bool finiteRegisterValue(double value)
{
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

inline bool nonnegativeRegisterValue(double value)
{
    return finiteRegisterValue(value) && value >= 0.0;
}

inline bool positiveRegisterValue(double value)
{
    return finiteRegisterValue(value) && value > 0.0 && static_cast<float>(value) > 0.0f;
}

template <class Pid>
bool validPid(const Pid& value)
{
    return nonnegativeRegisterValue(value.kp) && nonnegativeRegisterValue(value.ki) &&
           nonnegativeRegisterValue(value.kd) && nonnegativeRegisterValue(value.i_windup) &&
           positiveRegisterValue(value.max_output);
}

inline bool healthyTemperatures(double motor, double bridge, double cutoff)
{
    return std::isfinite(motor) && std::isfinite(bridge) && std::isfinite(cutoff) &&
           cutoff > 0.0 && motor > 0.0 && motor < cutoff && bridge > 0.0 && bridge < 90.0;
}

// A new communication loss is not permission to undo an operator's stop.
// Explicit home/enable/init releases the latch only after successful setup.
class RecoveryHomingPolicy
{
  public:
    void disable(std::uint16_t id) { disabled_.insert(id); pending_.erase(id); }
    void activate(std::uint16_t id) { disabled_.erase(id); pending_.erase(id); }
    void requestRecovery(std::uint16_t id)
    {
        if (!disabled_.contains(id)) pending_.insert(id);
    }
    bool pending(std::uint16_t id) const { return pending_.contains(id); }
    bool take(std::uint16_t id) { return pending_.erase(id) != 0; }

  private:
    std::unordered_set<std::uint16_t> disabled_, pending_;
};
}
