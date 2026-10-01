#pragma once

#include "candle_ros2/impedance_close.hpp"
#include <limits>

// Directional stop search without a preset travel range. A stop is only inferred: an obstruction can
// look identical. The caller must provide a clear path and a repeatable stop.
struct ImpedanceHome
{
    struct Config
    {
        double seekTorque = 1.0; // Opening effort at rest; damping softens arrival at the stop.
        double damping = 0.5;    // Nm/(rad/s), runs in the MD impedance loop, Kp=0.
        double torque = 2.0;
        double contactDuration = 0.25;
        double timeout = 2.0;
        double releaseTimeout = 3.0; // Passive rebound can outlast the powered search.
    };
    enum class State { Searching, Found, Releasing, Ready, Failed };
    Config config;
    ImpedanceClose::Config motion;
    double start, direction, previousPosition;
    State state = State::Searching;
    double elapsed = 0.0, settledTime = 0.0;
    double settledMin, settledMax;

    static bool unrestrictedPositionLimits(float minimum, float maximum)
    {
        return (minimum == 0.0f && maximum == 0.0f) ||
               (minimum == std::numeric_limits<float>::lowest() &&
                maximum == std::numeric_limits<float>::max());
    }

    ImpedanceHome(double position, double openingDirection, Config home, ImpedanceClose::Config settings)
        : config(home), motion(settings), start(position), direction(openingDirection),
          previousPosition(position), settledMin(position), settledMax(position)
    {
        for (double v : {home.seekTorque, home.damping, home.torque, home.contactDuration,
                         home.timeout, home.releaseTimeout, motion.tolerance, motion.maxStep,
                         motion.feedbackTimeout, motion.torqueLimit})
            if (!std::isfinite(v) || v <= 0.0)
                throw std::invalid_argument("homing settings must be finite and positive");
        if (!std::isfinite(position) || std::abs(direction) != 1.0 ||
            home.contactDuration >= home.timeout || home.contactDuration >= home.releaseTimeout ||
            home.seekTorque > home.torque ||
            motion.maxStep > motion.feedbackTimeout || home.torque > motion.torqueLimit)
            throw std::invalid_argument("invalid homing torque/gain/timing bounds");
    }

    State update(double position, double velocity, double torque, double dt)
    {
        if (state != State::Searching)
            return state;
        const double travel = direction * (position - start);
        if (!fresh(position, velocity, torque, dt) ||
            !std::isfinite(travel) || travel < -motion.tolerance ||
            std::abs(torque) > config.torque + 0.1)
            return state = State::Failed;
        previousPosition = position;
        // Stationary feedback alone is also produced by an unpowered drive.
        // Require measured effort in the search direction for the entire dwell;
        // half the requested effort tolerates control ripple without accepting
        // the millinewton-metre idle readings seen from a disabled output.
        if (settled(position, dt, direction * torque >= 0.5 * config.seekTorque))
            state = State::Found;
        return state;
    }

    ImpedanceClose::Command command() const
    {
        // No spring, integral or endpoint. Firmware caps feedforward + damping.
        return state == State::Searching
            ? ImpedanceClose::Command{previousPosition, direction * config.seekTorque, 0.0, config.damping}
            : ImpedanceClose::Command{previousPosition, 0.0, 0.0, 0.0};
    }

    void beginRelease()
    {
        state = State::Releasing;
        settledMin = settledMax = previousPosition;
        elapsed = settledTime = 0.0;
    }

    State updateRelease(double position, double velocity, double torque, double dt)
    {
        if (state != State::Releasing) return state;
        if (!fresh(position, velocity, torque, dt))
            return state = State::Failed;
        previousPosition = position;
        // Passive rebound need only settle unloaded; its absolute offset from
        // the first open zero is telemetry, not a readiness condition.
        if (settled(position, dt, std::abs(torque) <= .1)) state = State::Ready;
        return state;
    }

  private:
    bool fresh(double position, double velocity, double torque, double dt)
    {
        elapsed += dt;
        return std::isfinite(position) && std::isfinite(velocity) && std::isfinite(torque) &&
               std::isfinite(dt) && dt > 0.0 && dt <= motion.feedbackTimeout &&
               elapsed < (state == State::Releasing ? config.releaseTimeout : config.timeout);
    }

    bool settled(double position, double dt, bool unloaded = true)
    {
        settledMin = std::min(settledMin, position);
        settledMax = std::max(settledMax, position);
        settledTime += std::min(dt, motion.maxStep);
        if (!unloaded || settledMax - settledMin > motion.tolerance)
        {
            settledMin = settledMax = position;
            settledTime = 0.0;
        }
        return settledTime >= config.contactDuration;
    }
};
