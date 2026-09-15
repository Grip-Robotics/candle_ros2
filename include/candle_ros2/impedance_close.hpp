#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Pure command shaping; the MD runs the impedance PD and torque limiter.
struct ImpedanceClose
{
    enum class State { Closing, Contact, Closed, Failed };

    struct Config
    {
        double lead = 0.04;             // rad: maximum closing target error
        double tolerance = 0.005;       // rad
        double contactTorque = 0.35;   // Nm, signed in the closing direction
        double contactVelocity = 0.03; // rad/s
        double contactDuration = 0.15; // s of consecutive contact evidence
        double timeout = 5.0;          // s to contact or closed endpoint
        double maxStep = 0.05;         // s: never catch up a delayed control tick
        double feedbackTimeout = 0.25; // s
    };

    Config config;
    double target;
    double closed;
    double speed;
    double direction;
    double elapsed = 0.0;
    double contactTime = 0.0;
    State state = State::Closing;

    ImpedanceClose(double position, double closedPosition, double velocity, Config settings)
        : config(settings), target(position), closed(closedPosition), speed(velocity),
          direction(closedPosition >= position ? 1.0 : -1.0)
    {
        for (double value : {config.lead, config.tolerance, config.contactTorque,
                             config.contactVelocity, config.contactDuration, config.timeout,
                             config.maxStep, config.feedbackTimeout, speed})
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("impedance close settings must be finite and positive");
        if (!std::isfinite(position) || !std::isfinite(closed) ||
            config.tolerance >= config.lead || config.contactDuration >= config.timeout ||
            config.maxStep > config.feedbackTimeout)
            throw std::invalid_argument("invalid impedance close position or timing bounds");
    }

    State update(double position, double velocity, double torque, double dt)
    {
        if (state == State::Failed)
            return state;
        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(torque) ||
            !std::isfinite(dt) || dt <= 0.0 || dt > config.feedbackTimeout)
            return state = State::Failed;

        // Even while holding, retreat the target if the object pushes back.
        // A frozen target must never regain a large closing error.
        const double bounded = std::min(direction * target,
                                         direction * position + config.lead);
        target = direction * bounded;
        if (state != State::Closing)
            return state;

        elapsed += dt;
        if (elapsed >= config.timeout)
            return state = State::Failed;
        if (std::abs(closed - position) <= config.tolerance &&
            std::abs(velocity) <= config.contactVelocity)
            return state = State::Closed;

        // Check the previously applied target, not the target about to be sent.
        const bool contact = direction * torque >= config.contactTorque &&
                             std::abs(velocity) <= config.contactVelocity &&
                             direction * (target - position) > config.tolerance;
        contactTime = contact ? contactTime + std::min(dt, config.maxStep) : 0.0;
        if (contactTime >= config.contactDuration)
            return state = State::Contact;

        target = direction * std::min({direction * closed,
                                        direction * target + speed * std::min(dt, config.maxStep),
                                        direction * position + config.lead});
        return state;
    }
};
