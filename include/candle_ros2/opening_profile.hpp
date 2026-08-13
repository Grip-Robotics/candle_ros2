#pragma once

#include <cmath>

enum class GripperResumeMode
{
    Impedance,
    PositionProfile
};

inline GripperResumeMode resumeModeForProfileOpening(bool profileOpening)
{
    return profileOpening ? GripperResumeMode::PositionProfile
                          : GripperResumeMode::Impedance;
}

inline bool isOpeningTargetDirectionValid(double currentRawPosition,
                                          double targetRawPosition,
                                          double openLogicalPosition,
                                          double closedLogicalPosition)
{
    if (!std::isfinite(currentRawPosition) || !std::isfinite(targetRawPosition) ||
        !std::isfinite(openLogicalPosition) || !std::isfinite(closedLogicalPosition))
        return false;

    const double openingDirection = openLogicalPosition - closedLogicalPosition;
    if (openingDirection == 0.0)
        return false;

    constexpr double OPENING_DIRECTION_TOLERANCE_RAD = 0.005;
    const double rawMovement = targetRawPosition - currentRawPosition;
    return std::abs(rawMovement) <= OPENING_DIRECTION_TOLERANCE_RAD ||
           rawMovement * openingDirection >= 0.0;
}
