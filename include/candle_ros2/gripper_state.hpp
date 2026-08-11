#pragma once

#include <cmath>

enum class GripperOperationalState
{
    Open,
    Closed,
    Homing,
    Moving,
    Idle,
    Unknown
};

inline GripperOperationalState classifyGripperState(bool   homing,
                                                    bool   sampleAvailable,
                                                    double position,
                                                    double velocity,
                                                    double openPosition,
                                                    double closedPosition,
                                                    double positionTolerance,
                                                    double movingVelocityThreshold)
{
    if (homing)
        return GripperOperationalState::Homing;
    if (!sampleAvailable || !std::isfinite(position) || !std::isfinite(velocity))
        return GripperOperationalState::Unknown;
    if (std::abs(velocity) > movingVelocityThreshold)
        return GripperOperationalState::Moving;
    if (std::abs(position - openPosition) <= positionTolerance)
        return GripperOperationalState::Open;
    if (std::abs(position - closedPosition) <= positionTolerance)
        return GripperOperationalState::Closed;
    return GripperOperationalState::Idle;
}

inline const char* gripperStateName(GripperOperationalState state)
{
    switch (state)
    {
        case GripperOperationalState::Open:
            return "OPEN";
        case GripperOperationalState::Closed:
            return "CLOSED";
        case GripperOperationalState::Homing:
            return "HOMING";
        case GripperOperationalState::Moving:
            return "MOVING";
        case GripperOperationalState::Idle:
            return "IDLE";
        case GripperOperationalState::Unknown:
            return "UNKNOWN";
    }
    return "UNKNOWN";
}
