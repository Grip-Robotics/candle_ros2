#pragma once

#include <cmath>
#include <cstddef>
#include <optional>

class PositionTracker
{
  public:
    enum class State
    {
        Uninitialized,
        Tracking,
        Recovering,
        Faulted
    };

    enum class RecoveryResult
    {
        Pending,
        Recovered,
        Rejected
    };

    PositionTracker() = default;

    PositionTracker(double wrapPeriodRad, double maxRecoveryDeltaRad, std::size_t recoverySamples)
        : m_wrapPeriodRad(wrapPeriodRad),
          m_maxRecoveryDeltaRad(maxRecoveryDeltaRad),
          m_requiredRecoverySamples(recoverySamples)
    {
    }

    void initialize(double rawPosition)
    {
        m_lastRawPosition    = rawPosition;
        m_continuousPosition = rawPosition;
        m_rawOffset          = 0.0;
        m_recoveryCandidate.reset();
        m_recoverySampleCount = 0;
        m_state               = State::Tracking;
    }

    bool restore(double rawPosition, double logicalPosition, std::optional<double> logicalTarget)
    {
        if (!std::isfinite(rawPosition) || !std::isfinite(logicalPosition) ||
            (logicalTarget.has_value() && !std::isfinite(*logicalTarget)))
            return false;

        m_lastRawPosition    = rawPosition;
        m_continuousPosition = logicalPosition;
        m_rawOffset          = logicalPosition - rawPosition;
        m_lastTarget         = logicalTarget;
        m_recoveryCandidate.reset();
        m_recoverySampleCount = 0;
        m_state               = State::Tracking;
        return true;
    }

    void resetAtZero(double rawPosition = 0.0)
    {
        m_lastRawPosition    = rawPosition;
        m_continuousPosition = 0.0;
        m_rawOffset          = -rawPosition;
        m_lastTarget         = 0.0;
        m_recoveryCandidate.reset();
        m_recoverySampleCount = 0;
        m_state               = State::Tracking;
    }

    std::optional<double> observe(double rawPosition)
    {
        if (!std::isfinite(rawPosition))
            return std::nullopt;

        if (m_state == State::Uninitialized)
        {
            initialize(rawPosition);
            return m_continuousPosition;
        }

        if (m_state != State::Tracking)
            return std::nullopt;

        const double delta = std::remainder(rawPosition - m_lastRawPosition, m_wrapPeriodRad);
        m_continuousPosition += delta;
        m_lastRawPosition = rawPosition;
        m_rawOffset       = m_continuousPosition - rawPosition;
        return m_continuousPosition;
    }

    void markCommunicationLost()
    {
        if (m_state == State::Tracking)
        {
            m_state = State::Recovering;
            m_recoveryCandidate.reset();
            m_recoverySampleCount = 0;
        }
    }

    RecoveryResult observeRecovery(double rawPosition)
    {
        if (m_state != State::Recovering || !std::isfinite(rawPosition))
            return RecoveryResult::Rejected;

        const double candidate =
            nearestEquivalent(rawPosition, m_continuousPosition, m_wrapPeriodRad);
        if (std::abs(candidate - m_continuousPosition) > m_maxRecoveryDeltaRad)
        {
            markFaulted();
            return RecoveryResult::Rejected;
        }

        if (m_recoveryCandidate.has_value() &&
            std::abs(candidate - *m_recoveryCandidate) > RECOVERY_SAMPLE_TOLERANCE_RAD)
        {
            m_recoveryCandidate = candidate;
            m_recoverySampleCount = 1;
            return RecoveryResult::Pending;
        }

        m_recoveryCandidate = candidate;
        ++m_recoverySampleCount;
        if (m_recoverySampleCount < m_requiredRecoverySamples)
            return RecoveryResult::Pending;

        m_continuousPosition = candidate;
        m_lastRawPosition    = rawPosition;
        m_rawOffset          = m_continuousPosition - rawPosition;
        m_recoveryCandidate.reset();
        m_recoverySampleCount = 0;
        m_state               = State::Tracking;
        return RecoveryResult::Recovered;
    }

    void markFaulted()
    {
        m_state = State::Faulted;
        m_recoveryCandidate.reset();
        m_recoverySampleCount = 0;
    }

    static double nearestEquivalent(double rawPosition,
                                    double referencePosition,
                                    double wrapPeriodRad)
    {
        return rawPosition +
               std::round((referencePosition - rawPosition) / wrapPeriodRad) * wrapPeriodRad;
    }

    double logicalToRaw(double logicalPosition) const
    {
        return logicalPosition - m_rawOffset;
    }

    void setLastTarget(double logicalTarget)
    {
        m_lastTarget = logicalTarget;
    }

    State state() const
    {
        return m_state;
    }

    bool isTracking() const
    {
        return m_state == State::Tracking;
    }

    double continuousPosition() const
    {
        return m_continuousPosition;
    }

    double rawOffset() const
    {
        return m_rawOffset;
    }

    double rawPosition() const
    {
        return m_lastRawPosition;
    }

    std::optional<double> lastTarget() const
    {
        return m_lastTarget;
    }

  private:
    static constexpr double RECOVERY_SAMPLE_TOLERANCE_RAD = 0.02;

    double      m_wrapPeriodRad          = 0.62831853;
    double      m_maxRecoveryDeltaRad    = 0.25;
    std::size_t m_requiredRecoverySamples = 3;

    State                 m_state = State::Uninitialized;
    double                m_lastRawPosition = 0.0;
    double                m_continuousPosition = 0.0;
    double                m_rawOffset = 0.0;
    std::optional<double> m_lastTarget;
    std::optional<double> m_recoveryCandidate;
    std::size_t           m_recoverySampleCount = 0;
};
