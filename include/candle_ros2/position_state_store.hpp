#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

struct PersistedPositionState
{
    double       logicalPosition = 0.0;
    double       rawPosition     = 0.0;
    double       logicalTarget   = 0.0;
    double       wrapPeriodRad   = 0.0;
    double       travelRad       = 0.0;
    std::uint64_t calibrationGeneration = 0;
    std::int64_t  timestampUnixMs       = 0;
    bool          homed                 = false;
};

inline bool isPersistedPositionStateCompatible(const PersistedPositionState& state,
                                               double expectedWrapPeriodRad,
                                               double expectedTravelRad)
{
    return state.homed && std::isfinite(state.logicalPosition) &&
           std::isfinite(state.rawPosition) && std::isfinite(state.logicalTarget) &&
           std::abs(state.wrapPeriodRad - expectedWrapPeriodRad) <= 1e-6 &&
           std::abs(state.travelRad - expectedTravelRad) <= 1e-6;
}

class PositionStateStore
{
  public:
    explicit PositionStateStore(std::string path);

    bool load(std::string* error = nullptr);
    bool saveAtomic(std::string* error = nullptr) const;

    void set(std::uint16_t id, const PersistedPositionState& state);
    std::optional<PersistedPositionState> get(std::uint16_t id) const;
    void erase(std::uint16_t id);

    const std::string& path() const;

  private:
    static constexpr int FORMAT_VERSION = 1;

    std::string expandUserPath(std::string path) const;

    std::string m_path;
    std::unordered_map<std::uint16_t, PersistedPositionState> m_states;
};
