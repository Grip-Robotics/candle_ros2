#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

// Shared open/close command shaping; "closed" is the requested endpoint.
// The MD runs bounded PD + I, or object/closed/captured torque holding.
struct ImpedanceClose
{
    enum class State { Closing, Contact, Closed, TorqueHold, Failed };

    struct Config
    {
        double lead = 0.04;             // rad: maximum closing target error
        double openLead = 0.04;         // rad: independent opening target error
        double tolerance = 0.005;       // rad: settled endpoint
        double travelMin = -0.05;      // rad: absolute guards, independent of targets
        double travelMax = 0.75;
        double contactTorque = 0.35;   // Nm, signed in the commanded direction
        double contactVelocity = 0.03; // rad/s
        double contactDuration = 0.15; // s of consecutive contact/endpoint evidence
        double timeout = 2.0;          // s until close torque capture
        double openTimeout = 0.5;      // s until opening zero-torque release
        double maxStep = 0.05;         // s: never catch up a delayed control tick
        double feedbackTimeout = 0.25; // s
        double kp = 12.5;
        double kd = 0.05;
        double ki = 2.0;               // Nm/(rad*s), closing only; 0 disables integral
        double torqueLimit = 1.0;      // MD limit applies to the TOTAL PD + I torque
        double holdTorque = 2.0;       // Nm for inferred objects, clipped to torqueLimit
        double closedHoldTorque = 0.25; // Nm closing floor (braking takes priority) and closed hold
        double holdTorqueRate = 2.0;   // Nm/s, never catch up delayed samples
        double objectMargin = 0.01;   // rad before closed endpoint: no object boost here

        bool withinTravel(double position) const
        {
            return std::isfinite(position) && position >= travelMin && position <= travelMax;
        }
    };

    struct Command
    {
        double position;
        double torque;
        double kp;
        double kd;
    };

    Config config;
    double target;
    double closed;
    double speed;
    double direction;
    double elapsed = 0.0;
    double contactTime = 0.0;
    double endpointTime = 0.0;
    double previousPosition;
    double previousVelocity = 0.0;
    double integralTorque = 0.0;
    double holdingTorque = 0.0;
    bool hasObject = false;
    bool opening;
    State state = State::Closing;

    ImpedanceClose(double position, double closedPosition, double velocity, Config settings,
                   bool isOpening = false, double motionDirection = 0.0)
        : config(settings), target(position), closed(closedPosition), speed(velocity),
          direction(motionDirection == 0.0 ? (closedPosition >= position ? 1.0 : -1.0)
                                          : motionDirection), previousPosition(position),
          opening(isOpening)
    {
        for (double value : {config.lead, config.openLead, config.tolerance, config.contactTorque,
                             config.contactVelocity, config.contactDuration, config.timeout, config.openTimeout,
                             config.maxStep, config.feedbackTimeout, config.torqueLimit,
                             config.holdTorque, config.holdTorqueRate, config.objectMargin, speed})
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("impedance close settings must be finite and positive");
        for (double value : {config.kp, config.kd, config.ki, config.closedHoldTorque})
            if (!std::isfinite(value) || value < 0.0)
                throw std::invalid_argument("impedance gains and closed bias must be finite and non-negative");
        if (!std::isfinite(position) || !std::isfinite(closed) ||
            !std::isfinite(config.travelMin) || !std::isfinite(config.travelMax) ||
            config.travelMin >= config.travelMax ||
            std::abs(direction) != 1.0 ||
            config.tolerance >= std::min(config.lead, config.openLead) ||
            config.contactDuration >= std::min(config.timeout, config.openTimeout) ||
            config.maxStep > config.feedbackTimeout || config.objectMargin < config.tolerance)
            throw std::invalid_argument("invalid impedance close position or timing bounds");
    }

    Command command() const
    {
        if (state == State::Failed)
            return {target, 0.0, 0.0, 0.0};
        if (state == State::TorqueHold ||
            (!opening && (state == State::Contact || state == State::Closed)))
            return {target, holdingTorque, 0.0, 0.0};
        const double minimum = closingFloor();
        const double output = config.kp * (target - previousPosition) -
                              config.kd * previousVelocity + integralTorque;
        // Clamp the combined closing effort, not just feedforward that PD could
        // cancel. Use pure torque while the floor binds; otherwise retain MD PD.
        // This bounds each fresh command, not instantaneous measured motor torque.
        if (!opening && minimum > 0.0 && direction * output < minimum)
            return {target, direction * minimum, 0.0, 0.0};
        return {target, integralTorque, config.kp, config.kd};
    }

    State update(double position, double velocity, double torque, double dt)
    {
        if (state == State::Failed)
            return state;
        if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(torque) ||
            !std::isfinite(dt) || dt <= 0.0 || dt > config.feedbackTimeout)
            return fail();
        // Opening timeout is a passive release, not a hold to reacquire after rebound.
        // Keep the job alive so feedback and the node's travel/fault guards still run.
        if (opening && state == State::TorqueHold)
            return state;

        const double step = std::min(dt, config.maxStep);
        const double lastPosition = previousPosition;
        const double positionSpeed = std::abs(position - lastPosition) / dt;
        previousPosition = position;
        previousVelocity = velocity;
        if (!opening)
        {
            // Hysteresis: fingertip rebound must not repeatedly rearm object
            // torque and restart the closing deadline at the same boundary.
            const double remaining = direction * (closed - position);
            if (remaining <= config.objectMargin)
                nearClosed = true;
            else if (remaining > config.objectMargin + config.lead)
                nearClosed = false;
        }
        const bool stopped = std::abs(velocity) <= config.contactVelocity &&
                             positionSpeed <= config.contactVelocity;
        endpointTime = std::abs(closed - position) <= config.tolerance && stopped
                           ? endpointTime + step : 0.0;
        // Also detect settling after an unconfirmed timeout torque capture.
        // A thin foil cannot be distinguished from empty closure here.
        if (!opening && state != State::Closed && endpointTime >= config.contactDuration)
        {
            beginHold(position, direction * std::min(direction * torque, config.closedHoldTorque), false);
            state = State::Closed;
        }
        const bool closedHold = !opening && state == State::Closed;
        if (state == State::TorqueHold || (!opening && state == State::Contact) || closedHold)
        {
            // Object compression is expected in the closing direction. Only
            // reopening from the most-closed anchor counts as displacement;
            // unconfirmed deadline holds retain their fixed-window guard.
            const double displacement = hasObject ? direction * (target - position)
                                                   : std::abs(position - target);
            if (displacement > config.lead ||
                std::abs(velocity) > speed || positionSpeed > speed ||
                (hasObject && nearClosed))
                return recover(position);
            if (hasObject || closedHold)
            {
                // Kp is zero here: follow compression without dropping torque.
                // ponytail: slow object loss can resemble compression; use
                // contact/force sensing if that distinction must be reliable.
                if (hasObject)
                    target = direction * std::max(direction * target, direction * position);
                const double desired = direction * std::clamp(
                    closedHold ? config.closedHoldTorque : config.holdTorque,
                    std::min(config.closedHoldTorque, config.torqueLimit), config.torqueLimit);
                const double increment = config.holdTorqueRate * step;
                holdingTorque += std::clamp(desired - holdingTorque, -increment, increment);
                if (!closedHold && std::abs(desired - holdingTorque) < 1e-9)
                    state = State::TorqueHold;
            }
            return state;
        }

        // Opening can be tuned without changing closing I/contact/recovery behavior.
        const double lead = opening ? config.openLead : config.lead;
        // Both directions stay lead-limited, including when holding a target.
        target = direction * std::min(direction * target,
                                       direction * position + lead);
        if (opening && state != State::Closing)
            return state;

        elapsed += dt;
        if (elapsed >= (opening ? config.openTimeout : config.timeout))
        {
            if (opening)
                return beginHold(position, 0.0, false);
            // No confirmed object: capture measured torque subject to the closing
            // floor/cap, without promoting it to the object-grip setpoint.
            return beginHold(position, torque, false);
        }

        if (state == State::Closing)
        {
            // Check the previously applied target, not the next ramp step.
            const bool contact = direction * torque >= config.contactTorque &&
                                 direction * (target - position) > config.tolerance;
            if (opening)
                contactTime = contact && stopped ? contactTime + step : 0.0;
            else
                updateContactEvidence(position, lastPosition, velocity, step,
                    contact && (opening || !nearClosed));
            if (endpointTime >= config.contactDuration)
                state = State::Closed;
            else if (contactTime >= config.contactDuration)
            {
                if (!opening)
                    return beginHold(position, torque, true);
                state = State::Contact; // opening obstruction; the node disables
            }
            else
                target = direction * std::min({direction * closed,
                                                direction * target + speed * step,
                                                direction * position + lead});
        }

        if (!opening)
        {
            const double error = target - position;
            // Bound both I magnitude and its rate (Ki * lead); never integrate
            // the remaining sweep to the unreachable full-close endpoint.
            const double increment = config.ki * std::clamp(error, -config.lead, config.lead) * step;
            const double candidate = std::clamp(integralTorque + increment,
                                                 -config.torqueLimit, config.torqueLimit);
            const double output = config.kp * error - config.kd * velocity + candidate;
            // Conditional anti-windup at both the hardware cap and closing floor.
            const double minimum = closingFloor();
            if ((std::abs(output) <= config.torqueLimit ||
                 (output > config.torqueLimit && increment < 0.0) ||
                 (output < -config.torqueLimit && increment > 0.0)) &&
                (minimum == 0.0 || direction * output >= minimum || direction * increment > 0.0))
                integralTorque = candidate;
        }
        return state;
    }

  private:
    double closingFloor() const
    {
        // Positive net torque cannot brake an unloaded jaw. Restore native PD
        // when it outruns the speed ramp or gets ahead of its target; otherwise
        // a pure-torque floor can accelerate straight through the travel limit.
        if (opening || direction * previousVelocity > speed ||
            direction * (previousPosition - target) > config.tolerance)
            return 0.0;
        return std::min(config.closedHoldTorque, config.torqueLimit);
    }

    bool nearClosed = false;
    double contactMinPosition = 0.0;
    double contactMaxPosition = 0.0;
    double contactVelocitySum = 0.0;

    void updateContactEvidence(double position, double lastPosition, double velocity,
                               double step, bool candidate)
    {
        if (!candidate || contactTime == 0.0)
        {
            contactTime = 0.0;
            contactVelocitySum = 0.0;
            contactMinPosition = std::min(position, lastPosition);
            contactMaxPosition = std::max(position, lastPosition);
        }
        if (!candidate)
            return;
        contactMinPosition = std::min(contactMinPosition, position);
        contactMaxPosition = std::max(contactMaxPosition, position);
        contactVelocitySum += velocity * step;
        contactTime += step;
        // Use movement across the dwell window, not noisy single-tick
        // derivatives. The range also rejects back-and-forth motion.
        if (contactMaxPosition - contactMinPosition > config.contactVelocity * config.contactDuration ||
            (contactTime >= config.contactDuration &&
             std::abs(contactVelocitySum / contactTime) > config.contactVelocity))
            contactTime = 0.0;
    }

    State beginHold(double position, double torque, bool object)
    {
        hasObject = object;
        const double minimum = opening ? 0.0 : std::min(config.closedHoldTorque, config.torqueLimit);
        holdingTorque = direction * std::clamp(direction * torque, minimum, config.torqueLimit);
        integralTorque = 0.0;
        target = position;
        return state = object ? State::Contact : State::TorqueHold;
    }

    State recover(double position)
    {
        // Preserve nearClosed until position actually leaves its hysteresis band.
        target = previousPosition = position;
        integralTorque = holdingTorque = 0.0;
        elapsed = contactTime = endpointTime = 0.0;
        hasObject = false;
        return state = State::Closing;
    }

    State fail()
    {
        integralTorque = 0.0;
        holdingTorque = 0.0;
        hasObject = false;
        return state = State::Failed;
    }
};
