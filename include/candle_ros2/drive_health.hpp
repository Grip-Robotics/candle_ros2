#pragma once

#include <string>
#include <string_view>

struct StartupHealth
{
    bool        error = false;
    std::string description;
};

inline StartupHealth startupHealthForState(std::string_view state)
{
    if (state == "TRACKING")
        return {};
    if (state == "RECOVERING")
        return {true, "position: recovering after communication loss, commands rejected"};
    if (state == "FAULTED")
        return {true, "position: reference lost, manual /md/zero calibration required"};
    if (state == "HOMING")
        return {true, "position: homing in progress, commands rejected"};
    if (state == "RESTORING")
        return {true, "position: restoring persisted state, commands rejected"};
    if (state == "UNINITIALIZED")
        return {true, "position: uninitialized, commands rejected"};
    return {true, "position: state unavailable, commands rejected"};
}
