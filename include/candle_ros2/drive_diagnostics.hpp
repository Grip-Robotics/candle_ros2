#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <optional>
#include <sstream>
#include <string>

namespace candle_ros2
{
// JSON emitted through the existing ROS logger: no new middleware dependency.
// Non-finite/unavailable numbers are null, never a misleading zero or invalid JSON.
inline std::string diagnosticQuote(const std::string& value)
{
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value)
    {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
        else out << c;
    }
    out << '"';
    return out.str();
}

struct DiagnosticJson
{
    std::ostringstream out;
    bool first = true;
    DiagnosticJson() { out.imbue(std::locale::classic()); out << std::setprecision(17) << '{'; }
    void key(const char* name) { if (!first) out << ','; first = false; out << diagnosticQuote(name) << ':'; }
    void number(const char* name, double value) { key(name); if (std::isfinite(value)) out << value; else out << "null"; }
    void integer(const char* name, int64_t value) { key(name); out << value; }
    void boolean(const char* name, bool value) { key(name); out << (value ? "true" : "false"); }
    void string(const char* name, const std::string& value) { key(name); out << diagnosticQuote(value); }
    void raw(const char* name, const std::string& value) { key(name); out << value; }
    std::string str() const { return out.str() + '}'; }
};

struct DriveDiagnosticSample
{
    int64_t rosNs = 0, steadyNs = 0, commandRosNs = 0;
    bool valid = false, commandValid = false;
    bool travelLimitsEnabled = false;
    double rawPosition = NAN, position = NAN, velocity = NAN, torque = NAN;
    unsigned state = 0, mode = 0;
    double desiredPosition = NAN, desiredTorque = NAN, kp = NAN, kd = NAN;
    // 0 idle, 1 homing, 2 reserved (old stop-search open), 3 close, 4 normal open.
    int operation = 0, phase = -1, controlState = -1;
    double elapsed = NAN, settled = NAN, travelMin = NAN, travelMax = NAN;
    double timeout = NAN, feedbackTimeout = NAN, torqueLimit = NAN;
    double feedbackIntervalMs = NAN;
    std::string json() const
    {
        DiagnosticJson j;
        j.integer("ros_ns", rosNs); j.integer("steady_ns", steadyNs);
        j.boolean("feedback_valid", valid);
        j.number("raw_position_rad", rawPosition); j.number("position_rad", position);
        j.number("velocity_rad_s", velocity); j.number("torque_nm", torque);
        if (valid) { j.integer("state", state); j.integer("mode", mode); }
        else { j.raw("state", "null"); j.raw("mode", "null"); }
        j.boolean("last_command_write_ok", commandValid); j.integer("command_ros_ns", commandRosNs);
        j.number("desired_position_rad", desiredPosition); j.number("desired_torque_nm", desiredTorque);
        j.number("kp", kp); j.number("kd", kd);
        j.integer("operation", operation); j.integer("phase", phase); j.integer("control_state", controlState);
        j.number("elapsed_s", elapsed); j.number("settled_s", settled);
        j.number("travel_min_rad", travelMin); j.number("travel_max_rad", travelMax);
        j.boolean("travel_limits_enabled", travelLimitsEnabled);
        j.number("timeout_s", timeout); j.number("feedback_timeout_s", feedbackTimeout);
        j.number("torque_limit_nm", torqueLimit);
        j.number("feedback_interval_ms", feedbackIntervalMs);
        return j.str();
    }
};

struct DriveDiagnosticHistory
{
    static constexpr size_t capacity = 64;
    static constexpr size_t registerGroupCount = 6;
    static constexpr int64_t samplePeriodNs = 50'000'000;
    std::array<DriveDiagnosticSample, capacity> samples{};
    size_t count = 0, next = 0;
    DriveDiagnosticSample latest;
    DriveDiagnosticSample lastCommand;
    std::array<std::string, registerGroupCount> registers{{"null", "null", "null", "null", "null", "null"}};
    struct Fault
    {
        uint64_t sequence;
        int64_t rosNs, steadyNs;
        std::string reason;
        DriveDiagnosticSample atStop;
        std::array<DriveDiagnosticSample, capacity> samples;
        size_t count, next;
        std::array<std::string, registerGroupCount> registers;
        int stateAtCapture = -1, phaseAtCapture = -1;
        double elapsedAtCapture = NAN, settledAtCapture = NAN;
    };
    std::optional<Fault> pending;
    uint64_t suppressed = 0, faultSequence = 0;
    int refreshGroup = -1;
    int64_t lastFaultNs = 0;

    void observe(DriveDiagnosticSample sample)
    {
        if (latest.steadyNs) sample.feedbackIntervalMs = (sample.steadyNs - latest.steadyNs) / 1e6;
        latest = sample; // Always retain the offending tick, including a failed read.
        if (count && sample.steadyNs - samples[(next + capacity - 1) % capacity].steadyNs < samplePeriodNs)
            return;
        samples[next] = sample;
        next = (next + 1) % capacity;
        if (count < capacity) ++count;
    }
    bool capture(uint64_t sequence, int64_t rosNs, int64_t steadyNs, const std::string& reason)
    {
        if (pending || (lastFaultNs && steadyNs - lastFaultNs < 1'000'000'000))
        { ++suppressed; return false; }
        pending.emplace(Fault{sequence, rosNs, steadyNs, reason, latest, samples, count, next, registers});
        lastFaultNs = steadyNs;
        faultSequence = sequence;
        refreshGroup = 0;
        return true;
    }
};
} // namespace candle_ros2
