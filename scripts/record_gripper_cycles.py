#!/usr/bin/env python3
"""Guarded MD20 commissioning cycles; never zero, change limits, or retry faults."""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time
from datetime import datetime, timezone
from statistics import fmean

DISABLED = "disabled by request; explicit enable required"
TRAVEL_MIN_RAD = -0.05
TRAVEL_MAX_RAD = 0.75


def within_travel(position):
    return math.isfinite(position) and TRAVEL_MIN_RAD <= position <= TRAVEL_MAX_RAD


def stop_reason(p, v, torque, motor, bridge, limit, responsive, error, description,
                disabled=False, target_torque=2.0):
    if not all(math.isfinite(x) for x in (p, v, torque, motor, bridge, limit)):
        return "invalid telemetry"
    if limit != 90.0 or motor <= 0.0 or bridge <= 0.0:
        return "invalid thermistor reading or changed cutoff"
    if not responsive:
        return "drive unresponsive"
    if not within_travel(p) or abs(torque) > target_torque + 0.1:
        return "travel/torque bound exceeded"
    if motor >= limit:
        return "motor threshold reached"
    if "Error Motor Temperature" in description:
        return "motor temperature fault"
    if error and not (disabled and description == DISABLED):
        return "drive fault: " + description
    if bridge >= 90.0:
        return "controller temperature guard reached"
    return None


def guard_self_test():
    good = [0.47, 0.0, 2.0, 45.0, 50.0, 90.0, True, False, ""]
    assert stop_reason(*good) is None
    for index, value, expected in [
            (3, 89.99, None), (3, 90.0, "motor threshold reached"),
            (3, float("nan"), "invalid telemetry"),
            (3, 0.0, "invalid thermistor reading or changed cutoff"),
            (5, 100.0, "invalid thermistor reading or changed cutoff"),
            (0, -0.05, None), (0, 0.75, None),
            (0, -0.050001, "travel/torque bound exceeded"),
            (0, 0.750001, "travel/torque bound exceeded"),
            (2, 2.2, "travel/torque bound exceeded"),
            (4, 90.0, "controller temperature guard reached"),
            (6, False, "drive unresponsive")]:
        sample = good.copy()
        sample[index] = value
        assert stop_reason(*sample) == expected
    assert stop_reason(*good[:7], True, "hardware: Error Motor Temperature") == "motor temperature fault"
    assert stop_reason(*good[:7], True, DISABLED, disabled=True) is None
    assert stop_reason(*good[:7], True, DISABLED).startswith("drive fault")
    for target in (2.0, 2.5):
        sample = good.copy()
        sample[2] = target + 0.05
        assert stop_reason(*sample, target_torque=target) is None
        sample[2] = target + 0.11
        assert stop_reason(*sample, target_torque=target) == "travel/torque bound exceeded"
        sample[2] = -(target + 0.11)
        assert stop_reason(*sample, target_torque=target) == "travel/torque bound exceeded"
        sample[2], sample[3] = target, 90.0
        assert stop_reason(*sample, target_torque=target) == "motor threshold reached"
    print("Thermal guard self-test passed; no ROS initialization or hardware access.")




# Deliberately limited to the calibrated MD20 commissioning envelope.
# Gains/ramp/lead values are recorded, but these safeguards must match before init.
REQUIRED_PARAMETERS = {
    "init_devices_zero": False, "joint_name_prefix": "md_",
    "gripper_open_position_rad": 0.0, "gripper_closed_position_rad": 0.75,
    "gripper_travel_min_rad": -0.05, "gripper_travel_max_rad": 0.75,
    "gripper_torque_limit_nm": 2.0, "soft_close_slow_torque_limit_nm": 2.0,
    "soft_close_closed_tol_rad": 0.005, "close_timeout_s": 2.0,
    "open_timeout_s": 0.5,
    "close_feedback_timeout_s": 0.25,
}


def validate_ids(ids):
    if not ids or len(set(ids)) != len(ids) or any(i not in (111, 113, 447) for i in ids):
        raise ValueError("select unique calibrated IDs from 111, 113, 447")
    return tuple(ids)


def hold_duration(value):
    seconds = float(value)
    if not 1.0 <= seconds <= 30.0:
        raise argparse.ArgumentTypeError("hold duration must be between 1 and 30 seconds")
    return seconds


def validate_parameters(parameters):
    for key, expected in REQUIRED_PARAMETERS.items():
        if key not in parameters or type(parameters[key]) is not type(expected) or parameters[key] != expected:
            raise ValueError(f"unsafe/mismatched parameter {key}: {parameters.get(key)!r}; expected {expected!r}")


def window(samples, phase, drive, seconds=0.5):
    rows = [r for r in samples if r[1] == phase and r[2] == drive]
    if not rows:
        return None
    rows = [r for r in rows if r[0] >= rows[-1][0] - seconds]
    if not all(math.isfinite(v) for r in rows for v in r[3:]):
        return None
    return {"samples": len(rows), "position_rad": fmean(r[3] for r in rows),
            "position_range_rad": max(r[3] for r in rows) - min(r[3] for r in rows),
            "torque_nm": fmean(r[5] for r in rows),
            "peak_abs_torque_nm": max(abs(r[5]) for r in rows)}


def close_finished(message):
    return "close:" in message and any(text in message for text in (
        "object torque hold", "timeout torque hold", "closed endpoint"))


def open_finished(message):
    return "open:" in message and any(text in message for text in (
        "open endpoint", "timeout released"))


def close_response_confirmed(start_position, measured):
    # Match the recorder's endpoint tolerance and near-zero torque threshold.
    # An acknowledged timeout at the unchanged open position is not a close.
    return measured is not None and (
        abs(measured["position_rad"] - start_position) > 0.005 or
        abs(measured["position_rad"] - 0.75) <= 0.005 or
        abs(measured["torque_nm"]) >= 0.1)


def self_test():
    guard_self_test()
    validate_ids((111, 113, 447))
    assert hold_duration("10") == 10.0
    assert hold_duration(30) == 30.0
    for duration in (0, -1, 30.01, float("nan"), float("inf")):
        try:
            hold_duration(duration)
        except argparse.ArgumentTypeError:
            pass
        else:
            raise AssertionError("unbounded hold accepted")
    validate_parameters(dict(REQUIRED_PARAMETERS))
    for ids in ((), (111, 111), (999,)):
        try:
            validate_ids(ids)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid target IDs accepted")
    for key, value in (("init_devices_zero", True), ("gripper_torque_limit_nm", 2.5),
                       ("gripper_travel_max_rad", float("nan"))):
        bad = dict(REQUIRED_PARAMETERS, **{key: value})
        try:
            validate_parameters(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe profile accepted")
    rows = [(0.0, "initial", 111, 0.02, 0.0, 0.0),
            (0.8, "initial", 111, 0.04, 0.0, -0.02),
            (1.0, "initial", 111, 0.06, 0.0, 0.02),
            (1.0, "initial", 113, 0.5, 0.0, 2.0),
            (1.1, "opening", 111, 0.0, 0.0, 1.0)]
    measured = window(rows, "initial", 111)
    assert measured["samples"] == 2
    assert abs(measured["position_rad"] - 0.05) < 1e-12
    assert measured["torque_nm"] == 0.0
    assert measured["peak_abs_torque_nm"] == 0.02
    assert window(rows, "missing", 111) is None
    assert close_finished("Drive 111 close: object torque hold")
    assert close_finished("Drive 113 close: closed endpoint")
    assert close_finished("Drive 447 close: timeout torque hold")
    assert not close_finished("Drive 111 close: contact inferred")
    assert not close_finished("Drive 111 close: recovering from measured position")
    assert not close_finished("Drive 111 open: open endpoint")
    assert open_finished("Drive 111 open: open endpoint")
    assert open_finished("Drive 113 open: timeout released; endpoint unconfirmed")
    assert not open_finished("Drive 111 close: timeout torque hold")
    assert not open_finished("Drive 111 open: sustained resistance")
    assert not open_finished("")
    # Cycle 8: 113 never moved and measured only torque-sensor noise.
    assert not close_response_confirmed(0.0258744, {
        "position_rad": 0.0258694, "torque_nm": -0.0060369})
    assert not close_response_confirmed(0.0259, None)
    assert close_response_confirmed(0.0259, {"position_rad": 0.736, "torque_nm": 0.55})
    assert close_response_confirmed(0.0259, {"position_rad": 0.0259, "torque_nm": 2.0})
    assert close_response_confirmed(0.75, {"position_rad": 0.75, "torque_nm": 0.0})
    print("Cycle recorder self-test passed; no hardware access.")


def run(path, ids, hold_seconds=10.0):
    hold_seconds = hold_duration(hold_seconds)
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from candle_ros2.msg import MdHealth
    from candle_ros2.srv import Generic, InitDevices
    from rcl_interfaces.msg import Log
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState

    ids = validate_ids(ids)
    if Path(path + ".json").exists():
        raise FileExistsError(path + ".json")
    # Exclusive creation protects earlier measurements before any ROS/motor work.
    log = open(path, "x", newline="")
    writer = csv.writer(log)
    writer.writerow(["utc", "elapsed_s", "phase", "drive", "position_rad",
                     "velocity_rad_s", "torque_nm", "motor_c", "mosfet_c",
                     "health_age_s", "health_description"])
    rclpy.init()
    node = rclpy.create_node("guarded_gripper_cycle")
    origin = time.monotonic()
    phase = "initial"
    joints, health, outcomes = {}, {}, {}
    active, samples, events = set(), [], []
    abort = None
    result = {"start_utc": datetime.now(timezone.utc).isoformat(),
              "close_order": ids, "wait_after_last_close_s": hold_seconds,
              "open_target_rad": 0.0, "closed_target_rad": 0.75,
              "comparison_tolerance_rad": 0.005,
              "travel_min_rad": TRAVEL_MIN_RAD, "travel_max_rad": TRAVEL_MAX_RAD,
              "sequence_completed": False}

    def event(message):
        entry = {"elapsed_s": time.monotonic() - origin, "message": message}
        events.append(entry)
        print("EVENT", json.dumps(entry), flush=True)

    def on_joints(msg):
        nonlocal abort
        if any(len(values) != len(msg.name) for values in (msg.position, msg.velocity, msg.effort)):
            abort = "malformed joint telemetry"
            return
        now = time.monotonic()
        utc = datetime.now(timezone.utc).isoformat()
        for drive in ids:
            if f"md_{drive}" not in msg.name:
                abort = f"{drive}: missing from joint message"
                continue
            index = msg.name.index(f"md_{drive}")
            p, v, torque = msg.position[index], msg.velocity[index], msg.effort[index]
            joints[drive] = (now, p, v, torque)
            samples.append((now - origin, phase, drive, p, v, torque))
            h = health.get(drive)
            writer.writerow([utc, now - origin, phase, drive, p, v, torque,
                             h[1] if h else "", h[2] if h else "",
                             now - h[0] if h else "", h[-1] if h else ""])
            if (not all(math.isfinite(x) for x in (p, v, torque)) or
                    not within_travel(p) or abs(torque) > 2.1):
                abort = f"{drive}: invalid feedback or travel/torque bound exceeded"
        log.flush()

    def on_health(msg):
        nonlocal abort
        fields = [msg.motor_temperature_c, msg.mosfet_temperature_c,
                  msg.motor_shutdown_temperature_c, msg.responsive, msg.error, msg.active_errors]
        if any(len(field) != len(msg.device_ids) for field in fields):
            abort = "malformed health telemetry"
            return
        for drive, *values in zip(msg.device_ids, *fields):
            health[drive] = (time.monotonic(), *values)

    def on_log(msg):
        for drive in ids:
            if f"Drive {drive} close:" in msg.msg or f"Drive {drive} open:" in msg.msg:
                outcomes[drive] = msg.msg
                event(msg.msg)

    subscriptions = [node.create_subscription(JointState, "/md/joint_states", on_joints, 10),
                     node.create_subscription(MdHealth, "/md/health", on_health, 10),
                     node.create_subscription(Log, "/rosout", on_log, 100)]

    def check():
        if abort:
            raise RuntimeError(abort)
        now = time.monotonic()
        for drive in ids:
            if drive not in joints or drive not in health:
                raise RuntimeError(f"{drive}: missing telemetry")
            joint, h = joints[drive], health[drive]
            if now - joint[0] > 0.25 or now - h[0] > 0.6:
                raise RuntimeError(f"{drive}: stale telemetry")
            reason = stop_reason(*joint[1:], *h[1:], disabled=drive not in active)
            if reason:
                raise RuntimeError(f"{drive}: {reason}")

    def observe(seconds, guarded=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if guarded:
                check()

    def call(kind, service, devices, **fields):
        client = node.create_client(kind, "/md/" + service)
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError(service + " unavailable")
            event(f"REQUEST {service} {devices}")
            future = client.call_async(kind.Request(device_ids=list(devices), **fields))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if not future.done() or list(future.result().success) != [True] * len(devices):
                raise RuntimeError(service + " unconfirmed/rejected")
            event(f"ACK {service} {devices}")
        finally:
            node.destroy_client(client)

    def interrupted(signum, frame):
        raise KeyboardInterrupt("trial interrupted")

    signal.signal(signal.SIGTERM, interrupted)
    try:
        client = node.create_client(GetParameters, "/candle_config/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError("configuration service unavailable")
            names = list(REQUIRED_PARAMETERS) + [
                "gripper_impedance_kp", "gripper_impedance_kd", "gripper_impedance_ki",
                "gripper_velocity_limit_rad_s", "gripper_open_velocity_limit_rad_s",
                "close_target_lead_rad", "open_target_lead_rad",
                "gripper_hold_torque_nm", "gripper_closed_hold_torque_nm",
                "joint_state_publish_period_ms"]
            future = client.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            if not future.done() or future.result() is None:
                raise RuntimeError("configuration read unconfirmed")
            parameters = dict(zip(names, map(parameter_value_to_python, future.result().values)))
            validate_parameters(parameters)
            result["parameters"] = parameters
        finally:
            node.destroy_client(client)
        observe(1.0, guarded=False)
        check()
        if any(health[i][-1] != DISABLED for i in ids):
            raise RuntimeError("all selected drives must be disabled before trial")
        if any(health[i][1] >= 70.0 or health[i][2] >= 70.0 for i in ids):
            raise RuntimeError("cool further before starting")
        result["initial"] = {i: window(samples, "initial", i) for i in ids}
        result["starting_motor_c"] = {i: health[i][1] for i in ids}
        event("BASELINE " + json.dumps(result["initial"]))
        for drive in ids:
            phase = f"initializing_{drive}"
            call(InitDevices, "init_devices", [drive], mode="IMPEDANCE")
            observe(0.4)
            active.add(drive)
            check()
            phase = f"closing_{drive}"
            outcomes.pop(drive, None)
            close_start_position = joints[drive][1]
            call(Generic, "close_gripper", [drive])
            deadline = time.monotonic() + 4.0
            while not close_finished(outcomes.get(drive, "")):
                observe(0.01)
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"{drive}: no completed close/hold state within 4 s")
            observe(0.4)
            measured = window(samples, phase, drive)
            if not close_response_confirmed(close_start_position, measured):
                raise RuntimeError(f"{drive}: closing acknowledged but no movement or holding torque")
            event(f"CLOSE_RESULT {drive}: {joints[drive][1:]} {outcomes[drive]}")
        phase = "holding_all"
        event(f"Selected close sequences finished; starting {hold_seconds:g} s wait")
        observe(hold_seconds)
        result["held_before_open"] = {i: window(samples, phase, i) for i in ids}
        result["close_outcomes"] = dict(outcomes)
        phase = "opening_all"
        outcomes.clear()
        call(Generic, "open_gripper", ids)
        observe(3.0)
        if any(not open_finished(outcomes.get(i, "")) for i in ids):
            raise RuntimeError("not all drives reported an open endpoint or timeout release")
        result["powered_open"] = {i: window(samples, phase, i) for i in ids}
        result["open_outcomes"] = dict(outcomes)
        result["open_endpoint_confirmed"] = {
            i: "open endpoint" in outcomes[i] for i in ids}
        result["all_open_endpoints_confirmed"] = all(result["open_endpoint_confirmed"].values())
        result["timeout_release_zero_torque_verified"] = {
            i: result["powered_open"][i]["peak_abs_torque_nm"] < 0.1
            for i in ids if "timeout released" in outcomes[i]}
        if not all(result["timeout_release_zero_torque_verified"].values()):
            raise RuntimeError("opening timeout release still producing measured torque")
        result["sequence_completed"] = True
        result["stop_reason"] = ("sequence completed" if result["all_open_endpoints_confirmed"]
                                 else "sequence completed with opening timeout release")
    except BaseException as error:
        result["stop_reason"] = str(error) or type(error).__name__
        result["stopped_during"] = phase
    finally:
        phase = "disabled_final"
        try:
            call(Generic, "disable", ids)
            result["disable_ack"] = True
            active.clear()
            observe(1.0, guarded=False)
            result["disabled_final"] = {i: window(samples, phase, i) for i in ids}
            result["disabled_verified"] = all(
                i in health and health[i][-1] == DISABLED and
                time.monotonic() - health[i][0] <= 0.6 and
                i in joints and time.monotonic() - joints[i][0] <= 0.25 and
                abs(joints[i][3]) < 0.1 and
                stop_reason(*joints[i][1:], *health[i][1:], disabled=True) is None
                for i in ids)
        except BaseException as error:
            result["disable_ack"] = False
            result["disable_error"] = str(error)
        result["last_outcomes"] = dict(outcomes)
        result["final_health"] = {i: h[1:] for i, h in health.items() if i in ids}
        result["peak_abs_torque_nm"] = {
            i: max((abs(r[5]) for r in samples if r[2] == i and math.isfinite(r[5])), default=0.0)
            for i in ids}
        result["return_comparison"] = {}
        for i in ids:
            baseline = result.get("initial", {}).get(i)
            comparison = {}
            for key in ("powered_open", "disabled_final"):
                measured = result.get(key, {}).get(i)
                if baseline and measured:
                    delta = measured["position_rad"] - baseline["position_rad"]
                    comparison[key] = {"delta_from_initial_rad": delta,
                                       "position_from_zero_rad": measured["position_rad"],
                                       "within_0_005_rad_of_initial": abs(delta) <= 0.005}
            result["return_comparison"][i] = comparison
        result["events"] = events
        result["end_utc"] = datetime.now(timezone.utc).isoformat()
        with open(path + ".json", "x") as summary:
            json.dump(result, summary, indent=2)
        print("RESULT", json.dumps(result), flush=True)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--self-test", action="store_true")
    action.add_argument("--run", metavar="NEW_CSV_PATH")
    parser.add_argument("--ids", type=int, nargs="+", help="explicit target device IDs")
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--hold-seconds", type=hold_duration, default=10.0,
                        help="guarded hold before automatic opening, 1..30 seconds (default 10)")
    parser.add_argument("--clear", action="store_true", help="operator confirms the scene is safe")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        if not args.clear or not args.ids or not 1 <= args.cycles <= 100:
            parser.error("motion requires --clear, explicit --ids, and 1..100 cycles")
        ids = validate_ids(args.ids)
        path = Path(args.run)
        for cycle in range(1, args.cycles + 1):
            output = path if args.cycles == 1 else path.with_name(
                f"{path.stem}_cycle{cycle:02d}{path.suffix}")
            result = run(str(output), ids, args.hold_seconds)
            if not result["sequence_completed"] or not result.get("disabled_verified"):
                raise SystemExit("Stopped without retry: " + result["stop_reason"])
