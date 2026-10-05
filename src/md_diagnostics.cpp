#include "candle_ros2/md_node.hpp"

namespace
{
int64_t steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

void MdNode::recordDiagnosticFeedback(mab::MD& md, bool valid, double logicalPosition)
{
    auto& history = m_diagnostics[md.m_canId];
    auto sample = history.lastCommand;
    sample.rosNs = get_clock()->now().nanoseconds();
    sample.steadyNs = steadyNs();
    sample.valid = valid;
    if (valid)
    {
        sample.rawPosition = md.m_mdRegisters.mainEncoderPosition.value;
        sample.position = logicalPosition;
        sample.velocity = md.m_mdRegisters.mainEncoderVelocity.value;
        sample.torque = md.m_mdRegisters.motorTorque.value;
        sample.state = md.m_mdRegisters.state.value;
        sample.mode = md.m_mdRegisters.motionModeStatus.value;
    }
    const auto calibration = m_gripperCalibrations.find(md.m_canId);
    const auto& limits = calibration == m_gripperCalibrations.end() ? closeConfig : calibration->second.motion;
    sample.travelMin = limits.travelMin;
    sample.travelMax = limits.travelMax;
    sample.travelLimitsEnabled = limits.travelLimitsEnabled;
    sample.feedbackTimeout = limits.feedbackTimeout;
    sample.torqueLimit = limits.torqueLimit;
    const ImpedanceHome* home = nullptr;
    if (isHoming(md.m_canId))
    {
        sample.operation = 1;
        sample.phase = static_cast<int>(m_homingJob->sequence.phase);
        home = &m_homingJob->sequence.control;
        // Mechanical searches intentionally do not enforce learned travel.
        sample.travelMin = sample.travelMax = NAN;
        sample.travelLimitsEnabled = false;
    }
    else if (const auto closing = m_softCloseJobs.find(md.m_canId); closing != m_softCloseJobs.end())
    {
        const auto& c = closing->second.control;
        sample.operation = closing->second.opening ? 4 : 3;
        sample.phase = sample.controlState = static_cast<int>(c.state);
        sample.elapsed = c.elapsed;
        sample.settled = c.contactTime;
        sample.timeout = c.opening ? c.config.openTimeout : c.config.timeout;
    }
    if (home)
    {
        sample.controlState = static_cast<int>(home->state);
        sample.elapsed = home->elapsed;
        sample.settled = home->settledTime;
        sample.timeout = home->state == ImpedanceHome::State::Releasing ?
                         home->config.releaseTimeout : home->config.timeout;
        sample.torqueLimit = home->config.torque;
    }
    history.observe(sample);
}

void MdNode::captureDiagnosticFault(mab::MD& md, const std::string& reason)
{
    auto& history = m_diagnostics[md.m_canId];
    // Feedback timestamps remain those of the actual read. Only command fields
    // are refreshed here, including an attempted write which just failed.
    const auto& command = history.lastCommand;
    history.latest.commandRosNs = command.commandRosNs;
    history.latest.commandValid = command.commandValid;
    history.latest.desiredPosition = command.desiredPosition;
    history.latest.desiredTorque = command.desiredTorque;
    history.latest.kp = command.kp;
    history.latest.kd = command.kd;
    if (!history.capture(++m_diagnosticSequence, get_clock()->now().nanoseconds(), steadyNs(), reason)) return;
    auto& fault = *history.pending;
    const ImpedanceHome* home = nullptr;
    if (isHoming(md.m_canId))
    {
        home = &m_homingJob->sequence.control;
        fault.phaseAtCapture = static_cast<int>(m_homingJob->sequence.phase);
    }
    else if (const auto closing = m_softCloseJobs.find(md.m_canId); closing != m_softCloseJobs.end())
    {
        const auto& c = closing->second.control;
        fault.phaseAtCapture = fault.stateAtCapture = static_cast<int>(c.state);
        fault.elapsedAtCapture = c.elapsed;
        fault.settledAtCapture = c.contactTime;
    }
    if (home)
    {
        fault.stateAtCapture = static_cast<int>(home->state);
        fault.elapsedAtCapture = home->elapsed;
        fault.settledAtCapture = home->settledTime;
    }
}

std::string MdNode::readDiagnosticRegisters(mab::MD& md, int group, uint64_t faultSequence)
{
    // Local entries only: a diagnostic read must not mutate the control cache.
    // Each batch is at most 32 MD bytes including its two-byte header.
    // The synchronous CANdle USB envelope adds five bytes (at most 37 total).
    mab::MDRegisters_S r;
    const auto started = steadyNs();
    candle_ros2::DiagnosticJson j, values;
    j.string("event", "register_snapshot");
    j.integer("schema", 1);
    j.integer("drive_id", md.m_canId);
    j.integer("fault_sequence", faultSequence);
    j.boolean("after_fault_stop_request", faultSequence != 0);
    j.integer("read_start_ros_ns", get_clock()->now().nanoseconds());
    j.integer("read_start_steady_ns", started);
    j.boolean("motion_inhibited", m_motionInhibited.contains(md.m_canId));
    j.boolean("disable_pending", m_pendingDisables.contains(md.m_canId));
    if (m_diagnosticPollingDisabled)
    {
        j.boolean("read_ok", false);
        j.boolean("read_skipped", true);
        j.string("skip_reason", "optional polling disabled until node restart");
        j.raw("values", "null");
        return j.str();
    }
    mab::MD::Error_t error;
    if (group == 0)
    {
        j.string("group", "status_core");
        error = md.readRegisters(r.quickStatus, r.mainEncoderStatus, r.auxEncoderStatus,
            r.calibrationStatus, r.bridgeStatus);
        if (error == mab::MD::Error_t::OK)
        {
            values.integer("quick_status", r.quickStatus.value);
            values.integer("main_encoder_status", r.mainEncoderStatus.value);
            values.integer("aux_encoder_status", r.auxEncoderStatus.value);
            values.integer("calibration_status", r.calibrationStatus.value);
            values.integer("bridge_status", r.bridgeStatus.value);
        }
    }
    else if (group == 1)
    {
        j.string("group", "status_protection");
        // miscStatus (0x813) timed out even as a single disabled-drive read on
        // the deployed firmware. Never include it in optional traffic.
        error = md.readRegisters(r.hardwareStatus, r.communicationStatus, r.motionStatus,
            r.homingStatus);
        if (error == mab::MD::Error_t::OK)
        {
            values.integer("hardware_status", r.hardwareStatus.value);
            values.integer("communication_status", r.communicationStatus.value);
            values.boolean("communication_can_watchdog_warning", (r.communicationStatus.value & (uint32_t{1} << 30)) != 0);
            values.integer("motion_status", r.motionStatus.value);
            values.integer("homing_status", r.homingStatus.value);
            values.raw("misc_status", "null");
            values.boolean("misc_status_queried", false);
        }
    }
    else if (group == 2)
    {
        j.string("group", "command");
        error = md.readRegisters(r.state, r.motionModeStatus, r.canWatchdog,
            r.motorImpPidKp, r.motorImpPidKd, r.maxTorque);
        if (error == mab::MD::Error_t::OK)
        {
            values.integer("state", r.state.value);
            values.integer("mode", r.motionModeStatus.value);
            values.integer("can_watchdog_ms", r.canWatchdog.value);
            values.number("kp", r.motorImpPidKp.value);
            values.number("kd", r.motorImpPidKd.value);
            values.number("max_torque_nm", r.maxTorque.value);
        }
    }
    else if (group == 3)
    {
        j.string("group", "targets");
        error = md.readRegisters(r.targetPosition, r.targetVelocity, r.targetTorque,
            r.maxVelocity, r.profileVelocity);
        if (error == mab::MD::Error_t::OK)
        {
            values.number("target_raw_position_rad", r.targetPosition.value);
            values.number("target_velocity_rad_s", r.targetVelocity.value);
            values.number("target_torque_nm", r.targetTorque.value);
            values.number("max_velocity_rad_s", r.maxVelocity.value);
            values.number("profile_velocity_rad_s", r.profileVelocity.value);
        }
    }
    else if (group == 4)
    {
        j.string("group", "limits_supply");
        error = md.readRegisters(r.positionLimitMin, r.positionLimitMax, r.dcBusVoltage, r.motorIMax, r.motorKt);
        if (error == mab::MD::Error_t::OK)
        {
            values.number("position_limit_min_rad", r.positionLimitMin.value);
            values.number("position_limit_max_rad", r.positionLimitMax.value);
            values.number("dc_bus_voltage_v", r.dcBusVoltage.value);
            values.number("motor_current_limit_a", r.motorIMax.value);
            values.number("motor_torque_constant_nm_a", r.motorKt.value);
        }
    }
    else
    {
        j.string("group", "thermal");
        error = md.readRegisters(r.motorTemperature, r.mosfetTemperature, r.motorShutdownTemp);
        if (error == mab::MD::Error_t::OK)
        {
            values.number("motor_temperature_c", r.motorTemperature.value);
            values.number("mosfet_temperature_c", r.mosfetTemperature.value);
            values.integer("motor_shutdown_temperature_c", r.motorShutdownTemp.value);
        }
    }
    j.integer("read_end_ros_ns", get_clock()->now().nanoseconds());
    j.integer("read_end_steady_ns", steadyNs());
    j.number("read_duration_ms", (steadyNs() - started) / 1e6);
    j.boolean("read_ok", error == mab::MD::Error_t::OK);
    j.integer("read_error", static_cast<int>(error));
    j.raw("values", error == mab::MD::Error_t::OK ? values.str() : "null");
    if (error != mab::MD::Error_t::OK)
    {
        // One failed optional transaction is enough: never retry on the shared
        // control transport, including for later fault snapshots.
        m_diagnosticPollingDisabled = true;
        candle_ros2::DiagnosticJson disabled;
        disabled.string("event", "diagnostic_polling_disabled");
        disabled.integer("schema", 1);
        disabled.integer("drive_id", md.m_canId);
        disabled.integer("group_index", group);
        disabled.integer("ros_ns", get_clock()->now().nanoseconds());
        disabled.integer("steady_ns", steadyNs());
        disabled.integer("read_error", static_cast<int>(error));
        disabled.string("reason", "first optional register read failed; disabled until node restart");
        RCLCPP_WARN(get_logger(), "CANDLE_DIAG %s", disabled.str().c_str());
    }
    return j.str();
}

void MdNode::flushDiagnosticFaults()
{
    // Pure logging of memory already captured: no transport access or control changes.
    for (auto& [id, history] : m_diagnostics)
    {
        if (!history.pending) continue;
        const auto& fault = *history.pending;
        candle_ros2::DiagnosticJson j;
        j.string("event", "drive_fault"); j.integer("schema", 1);
        j.integer("drive_id", id); j.integer("fault_sequence", fault.sequence);
        j.integer("capture_ros_ns", fault.rosNs); j.integer("capture_steady_ns", fault.steadyNs);
        j.integer("emit_ros_ns", get_clock()->now().nanoseconds());
        j.string("reason", fault.reason); j.integer("suppressed_faults", history.suppressed);
        j.string("capture_stage", "before_stop_requested");
        j.integer("controller_state_at_capture", fault.stateAtCapture);
        j.integer("phase_at_capture", fault.phaseAtCapture);
        j.number("controller_elapsed_s_at_capture", fault.elapsedAtCapture);
        j.number("controller_settled_s_at_capture", fault.settledAtCapture);
        j.number("feedback_age_ms", fault.atStop.rosNs ? (fault.steadyNs - fault.atStop.steadyNs) / 1e6 : NAN);
        j.raw("at_stop", fault.atStop.json());
        j.raw("cached_status", fault.registers[0]);
        j.raw("cached_protection", fault.registers[1]);
        j.raw("cached_command", fault.registers[2]);
        j.raw("cached_targets", fault.registers[3]);
        j.raw("cached_limits_supply", fault.registers[4]);
        j.raw("cached_thermal", fault.registers[5]);
        j.integer("history_samples", fault.count);
        RCLCPP_WARN(get_logger(), "CANDLE_DIAG %s", j.str().c_str());
        // Separate bounded records avoid Docker's long-line fragmentation.
        for (size_t offset = 0; offset < fault.count; offset += 8)
        {
            candle_ros2::DiagnosticJson chunk;
            chunk.string("event", "fault_history"); chunk.integer("schema", 1);
            chunk.integer("drive_id", id); chunk.integer("fault_sequence", fault.sequence);
            chunk.integer("offset", offset);
            std::string samples = "[";
            for (size_t i = offset; i < std::min(offset + 8, fault.count); ++i)
            {
                if (i != offset) samples += ',';
                samples += fault.samples[(fault.next + candle_ros2::DriveDiagnosticHistory::capacity - fault.count + i) %
                    candle_ros2::DriveDiagnosticHistory::capacity].json();
            }
            chunk.raw("samples", samples + ']');
            RCLCPP_WARN(get_logger(), "CANDLE_DIAG %s", chunk.str().c_str());
        }
        history.pending.reset();
        history.suppressed = 0;
    }
}

void MdNode::publishDiagnostics()
{
    flushDiagnosticFaults();
    if (m_mds.empty() || m_diagnosticPollingDisabled || !m_pendingDisables.empty()) return;
    if (m_homingJob)
    {
        // One bounded read per 200ms, for the active search only. Previously
        // startup faults had no command/limit/protection snapshots at all.
        const auto now = std::chrono::steady_clock::now();
        if (now - m_lastDiagnosticRead < std::chrono::milliseconds(200)) return;
        if (m_homingJob->sequence.control.state != ImpedanceHome::State::Searching) return;
        m_lastDiagnosticRead = now;
        auto md = findMd(m_mds, m_homingJob->id);
        if (md == m_mds.end()) return;
        const auto group = m_homingJob->diagnosticGroup++ % candle_ros2::DriveDiagnosticHistory::registerGroupCount;
        auto record = readDiagnosticRegisters(*md, group, 0);
        m_diagnostics[md->m_canId].registers[group] = record;
        RCLCPP_INFO(get_logger(), "CANDLE_DIAG %s", record.c_str());
        return;
    }
    if (!m_diagnosticStartupReady)
    {
        // Keep optional background reads out of quiet inter-drive startup
        // gaps. The active homing search is sampled separately above.
        if (homingRequired && (!m_homingRequired.empty() ||
            std::any_of(m_mds.begin(), m_mds.end(), [&](const auto& md) {
                return !m_gripperCalibrations.contains(md.m_canId);
            }))) return;
        m_diagnosticStartupReady = true;
    }
    const auto now = std::chrono::steady_clock::now();
    // At most ONE extra transaction globally per 200 ms, never six bursts.
    if (now - m_lastDiagnosticRead < std::chrono::milliseconds(200)) return;
    m_lastDiagnosticRead = now;
    for (auto& md : m_mds)
    {
        auto& history = m_diagnostics[md.m_canId];
        if (history.refreshGroup < 0) continue;
        const auto group = history.refreshGroup++;
        auto record = readDiagnosticRegisters(md, group, history.faultSequence);
        history.registers[group] = record;
        RCLCPP_INFO(get_logger(), "CANDLE_DIAG %s", record.c_str());
        if (history.refreshGroup == static_cast<int>(candle_ros2::DriveDiagnosticHistory::registerGroupCount)) history.refreshGroup = -1;
        return;
    }
    const auto index = m_diagnosticCursor++ % (m_mds.size() * candle_ros2::DriveDiagnosticHistory::registerGroupCount);
    auto& md = m_mds[index / candle_ros2::DriveDiagnosticHistory::registerGroupCount];
    const auto group = index % candle_ros2::DriveDiagnosticHistory::registerGroupCount;
    auto record = readDiagnosticRegisters(md, group, 0);
    m_diagnostics[md.m_canId].registers[group] = record;
    RCLCPP_INFO(get_logger(), "CANDLE_DIAG %s", record.c_str());
}
