#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

enum class HomingSeekResult
{
    Continue,
    StopDetected,
    OverVelocity,
    MaxTravel,
    Timeout
};

struct HomingSeekState
{
    bool                       moved = false;
    std::optional<std::int64_t> verificationStartMs;
    std::optional<std::int64_t> stallStartMs;
    std::optional<double>       stallStartPositionDelta;
};

inline double updateHomingStopVerification(HomingSeekState& state,
                                           double           positionDelta,
                                           double           baseTorqueFraction,
                                           std::int64_t     elapsedMs,
                                           double           minMotionRad,
                                           std::int64_t     verificationRampMs)
{
    if (std::abs(positionDelta) >= minMotionRad)
        state.moved = true;

    if (!state.moved || baseTorqueFraction < 1.0)
    {
        state.verificationStartMs.reset();
        return 0.0;
    }

    if (!state.verificationStartMs.has_value())
        state.verificationStartMs = elapsedMs;

    return std::fmin(
        1.0,
        static_cast<double>(elapsedMs - *state.verificationStartMs) /
            static_cast<double>(verificationRampMs));
}

inline bool homingStopsRepeatable(double firstStop, double secondStop, double tolerance)
{
    return std::isfinite(firstStop) && std::isfinite(secondStop) &&
           std::isfinite(tolerance) && tolerance > 0.0 &&
           std::abs(secondStop - firstStop) <= tolerance;
}

inline HomingSeekResult updateHomingSeek(HomingSeekState& state,
                                         double           positionDelta,
                                         double           velocity,
                                         double           torqueFraction,
                                         std::int64_t     elapsedMs,
                                         double           minMotionRad,
                                         double           maxTravelRad,
                                         double           velocityTripRadS,
                                         double           stallVelocityRadS,
                                         std::int64_t     stallDwellMs,
                                         std::int64_t     timeoutMs)
{
    if (elapsedMs > timeoutMs)
        return HomingSeekResult::Timeout;
    if (std::abs(positionDelta) > maxTravelRad)
        return HomingSeekResult::MaxTravel;
    if (std::abs(velocity) > velocityTripRadS)
        return HomingSeekResult::OverVelocity;

    if (std::abs(positionDelta) >= minMotionRad)
        state.moved = true;

    const bool stalled =
        state.moved && torqueFraction >= 0.8 &&
        std::abs(velocity) <= stallVelocityRadS;
    if (!stalled)
    {
        state.stallStartMs.reset();
        state.stallStartPositionDelta.reset();
        return HomingSeekResult::Continue;
    }

    if (!state.stallStartMs.has_value())
    {
        state.stallStartMs = elapsedMs;
        state.stallStartPositionDelta = positionDelta;
        return HomingSeekResult::Continue;
    }

    const double stallPositionToleranceRad = std::fmax(0.005, minMotionRad * 0.25);
    if (!state.stallStartPositionDelta.has_value() ||
        std::abs(positionDelta - *state.stallStartPositionDelta) >
            stallPositionToleranceRad)
    {
        state.stallStartMs = elapsedMs;
        state.stallStartPositionDelta = positionDelta;
        return HomingSeekResult::Continue;
    }

    return elapsedMs - *state.stallStartMs >= stallDwellMs
               ? HomingSeekResult::StopDetected
               : HomingSeekResult::Continue;
}
