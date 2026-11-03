#pragma once

#include "candle_ros2/impedance_home.hpp"
#include <optional>

// Hardware operations (disable, zero, enable) remain the caller's responsibility.
// A complete calibration includes both mechanical stops and a return to open.
struct GripperHoming
{
    enum class Phase { Opening, Closing, Reopening, Ready, Failed };

    struct Calibration
    {
        double closedPosition;
        ImpedanceClose::Config motion;
        double openPosition;
    };

    ImpedanceHome control;
    Phase phase = Phase::Opening;

    GripperHoming(double raw, double openingDirection, ImpedanceHome::Config home,
                  ImpedanceClose::Config motion, double margin)
        : control(raw, openingDirection, home, motion), home_(home), motion_(motion),
          openingDirection_(openingDirection), margin_(margin)
    {
        if (!std::isfinite(margin) || margin < 0.0 ||
            !std::isfinite(motion.travelMin) || !std::isfinite(motion.travelMax) ||
            motion.travelMin >= motion.travelMax || !motion.withinTravel(0.0))
            throw std::invalid_argument("invalid homing travel margin or open-side bounds");
    }

    ImpedanceHome::State update(double position, double velocity, double torque, double dt)
    {
        if (phase == Phase::Failed || phase == Phase::Ready)
            return control.state;
        // Searches learn the travel range; preset and partially learned
        // position bounds must not constrain any search or disabled release.
        const auto state = control.state == ImpedanceHome::State::Releasing
            ? control.updateRelease(position, velocity, torque, dt)
            : control.update(position, velocity, torque, dt);
        if (state == ImpedanceHome::State::Failed)
            return fail();
        if (phase == Phase::Closing && state == ImpedanceHome::State::Found)
        {
            // Record the powered stop before release can relax the mechanism.
            if (!validClosedStop(position)) return fail();
            closedPosition_ = position;
        }
        if (phase == Phase::Reopening && state == ImpedanceHome::State::Found)
            openPosition_ = position;
        if (phase == Phase::Reopening && state == ImpedanceHome::State::Ready)
        {
            if (!closedPosition_ || !validOpenStop())
                return fail();
            phase = Phase::Ready;
        }
        return state;
    }

    bool beginRelease()
    {
        if (phase == Phase::Ready || phase == Phase::Failed ||
            control.state != ImpedanceHome::State::Found)
            return rejectTransition();
        if (phase == Phase::Closing)
        {
            if (!validClosedStop(control.previousPosition)) return rejectTransition();
            closedPosition_ = control.previousPosition;
        }
        if (phase == Phase::Reopening)
            openPosition_ = control.previousPosition;
        control.beginRelease();
        return true;
    }

    // Call only after the opening release is complete and hardware zero has
    // been acknowledged and read back. The closing search never changes zero.
    bool startClosing(double nearZero)
    {
        if (phase != Phase::Opening || control.state != ImpedanceHome::State::Ready ||
            !std::isfinite(nearZero) || std::abs(nearZero) > motion_.tolerance)
            return rejectTransition();
        control = ImpedanceHome(nearZero, -openingDirection_, home_, motion_);
        phase = Phase::Closing;
        return true;
    }

    bool startReopening(double releasedPosition)
    {
        if (phase != Phase::Closing || control.state != ImpedanceHome::State::Ready ||
            !closedPosition_ || !std::isfinite(releasedPosition))
            return rejectTransition();
        control = ImpedanceHome(releasedPosition, openingDirection_, home_, motion_);
        phase = Phase::Reopening;
        return true;
    }

    std::optional<Calibration> calibration() const
    {
        if (phase != Phase::Ready || !closedPosition_ || !openPosition_) return std::nullopt;
        return Calibration{*closedPosition_, calibratedMotion(), *openPosition_};
    }

  private:
    ImpedanceHome::Config home_;
    ImpedanceClose::Config motion_;
    double openingDirection_;
    double margin_;
    std::optional<double> closedPosition_;
    std::optional<double> openPosition_;

    bool validOpenStop() const
    {
        if (!openPosition_ || !closedPosition_) return false;
        const double stroke = -openingDirection_ * (*closedPosition_ - *openPosition_);
        const double bound = *openPosition_ + openingDirection_ * motion_.tolerance;
        return std::isfinite(*openPosition_) && std::isfinite(stroke) &&
               stroke > 2.0 * motion_.tolerance && std::isfinite(bound);
    }

    bool validClosedStop(double position) const
    {
        const double closingDirection = -openingDirection_;
        const double stroke = closingDirection * position;
        const double bound = position + closingDirection * margin_;
        return std::isfinite(position) && std::isfinite(stroke) &&
               stroke > 2.0 * motion_.tolerance && std::isfinite(bound);
    }

    ImpedanceClose::Config calibratedMotion() const
    {
        auto result = motion_;
        if (openingDirection_ < 0.0)
        {
            result.travelMax = *closedPosition_ + margin_;
            result.travelMin = std::min(result.travelMin, *openPosition_ - motion_.tolerance);
        }
        else
        {
            result.travelMin = *closedPosition_ - margin_;
            result.travelMax = std::max(result.travelMax, *openPosition_ + motion_.tolerance);
        }
        return result;
    }

    ImpedanceHome::State fail()
    {
        phase = Phase::Failed;
        control.state = ImpedanceHome::State::Failed;
        closedPosition_.reset();
        openPosition_.reset();
        return control.state;
    }

    bool rejectTransition()
    {
        fail();
        return false;
    }
};
