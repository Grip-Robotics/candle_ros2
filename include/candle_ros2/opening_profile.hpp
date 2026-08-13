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

    const double rawMovement = targetRawPosition - currentRawPosition;
    return rawMovement * openingDirection >= 0.0;
}
