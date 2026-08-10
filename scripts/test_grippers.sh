#!/usr/bin/env bash

set -Eeuo pipefail

readonly DEVICE_IDS=(342 343 345)
readonly DEVICE_IDS_YAML="[342, 343, 345]"
readonly SOFT_CLOSE_GAPS=(40 30 20)

MOVE_WAIT_SECONDS="${MOVE_WAIT_SECONDS:-2}"
SOFT_CLOSE_WAIT_SECONDS="${SOFT_CLOSE_WAIT_SECONDS:-7}"
SERVICE_TIMEOUT_SECONDS="${SERVICE_TIMEOUT_SECONDS:-15}"
ASSUME_YES=false
ZERO_DRIVES=false

usage()
{
    cat <<'EOF'
Usage: test_grippers.sh [--zero] [--yes]

Runs the calibrated service test for drives 342, 343, and 345:
  1. Initialize all drives while preserving their encoder zeros.
  2. Apply calibrated position and velocity PID gains.
  3. Close/open each gripper individually.
  4. Soft-close each gripper individually with transition gaps 40, 30, and 20 mm.

Options:
  --zero    Explicitly zero all drives at their current mechanical positions.
  --yes     Skip the interactive motion/zero confirmation.
  -h        Show this help.

Environment:
  MOVE_WAIT_SECONDS        Delay after normal open/close (default: 2).
  SOFT_CLOSE_WAIT_SECONDS  Delay after soft close (default: 7).
  SERVICE_TIMEOUT_SECONDS  ROS service timeout (default: 15).
EOF
}

while (($# > 0)); do
    case "$1" in
        --yes)
            ASSUME_YES=true
            ;;
        --zero)
            ZERO_DRIVES=true
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown argument: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

for delay in "$MOVE_WAIT_SECONDS" "$SOFT_CLOSE_WAIT_SECONDS"; do
    if [[ ! "$delay" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
        printf 'Delay values must be non-negative numbers, got: %s\n' "$delay" >&2
        exit 2
    fi
done
if [[ ! "$SERVICE_TIMEOUT_SECONDS" =~ ^[1-9][0-9]*$ ]]; then
    printf 'SERVICE_TIMEOUT_SECONDS must be a positive integer, got: %s\n' \
        "$SERVICE_TIMEOUT_SECONDS" >&2
    exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
    printf 'ros2 is unavailable. Source ROS 2 and the workspace first.\n' >&2
    exit 1
fi

if ! ros2 pkg prefix candle_ros2 >/dev/null 2>&1; then
    printf 'candle_ros2 is unavailable. Run: source install/setup.bash\n' >&2
    exit 1
fi

disable_all()
{
    set +e
    timeout "${SERVICE_TIMEOUT_SECONDS}s" \
        ros2 service call /md/disable candle_ros2/srv/Generic \
        "{device_ids: ${DEVICE_IDS_YAML}}" >/dev/null 2>&1
}

abort_test()
{
    local status=${1:-$?}
    trap - ERR INT TERM
    printf '\nTest interrupted or failed; requesting disable for all drives.\n' >&2
    disable_all
    exit "$status"
}

trap 'abort_test 1' ERR
trap 'abort_test 130' INT
trap 'abort_test 143' TERM

call_service()
{
    local service_name=$1
    local service_type=$2
    local request=$3
    local output

    printf '\n$ ros2 service call %s %s %s\n' "$service_name" "$service_type" "$request"
    if ! output="$(timeout "${SERVICE_TIMEOUT_SECONDS}s" \
        ros2 service call "$service_name" "$service_type" "$request" 2>&1)"; then
        printf '%s\n' "$output" >&2
        printf 'Service call %s failed or timed out.\n' "$service_name" >&2
        return 1
    fi
    printf '%s\n' "$output"

    if [[ "$output" =~ [Ff]alse ]]; then
        printf 'Service %s reported a failed drive.\n' "$service_name" >&2
        return 1
    fi
    if [[ ! "$output" =~ success ]]; then
        printf 'Service %s returned no recognizable success field.\n' "$service_name" >&2
        return 1
    fi
}

wait_for_service()
{
    local target=$1
    local deadline=$((SECONDS + SERVICE_TIMEOUT_SECONDS))

    while ((SECONDS < deadline)); do
        mapfile -t services < <(ros2 service list 2>/dev/null)
        for service in "${services[@]}"; do
            if [[ "$service" == "$target" ]]; then
                return 0
            fi
        done
        sleep 0.25
    done

    printf 'Timed out waiting for service %s\n' "$target" >&2
    return 1
}

if [[ "$ASSUME_YES" != true ]]; then
    if [[ ! -t 0 ]]; then
        printf 'Interactive confirmation is unavailable; rerun with --yes only after checking the hardware.\n' >&2
        exit 1
    fi

    if [[ "$ZERO_DRIVES" == true ]]; then
        printf 'WARNING: this test will ZERO and move drives %s.\n' "$DEVICE_IDS_YAML"
        printf 'Place every gripper at its mechanical open zero reference.\n'
        expected_confirmation=ZERO
    else
        printf 'WARNING: this test will move drives %s using their existing encoder zeros.\n' \
            "$DEVICE_IDS_YAML"
        expected_confirmation=RUN
    fi

    read -r -p "Type ${expected_confirmation} to continue: " confirmation
    if [[ "$confirmation" != "$expected_confirmation" ]]; then
        printf 'Test cancelled.\n'
        exit 0
    fi
fi

printf 'Waiting for the MD services...\n'
wait_for_service /md/init_devices

init_zero_setting=""
for parameter_node in /candle_config /candle_md_node; do
    if init_zero_setting="$(
        timeout "${SERVICE_TIMEOUT_SECONDS}s" \
            ros2 param get "$parameter_node" init_devices_zero 2>&1
    )"; then
        break
    fi
    init_zero_setting=""
done
if [[ -z "$init_zero_setting" ]]; then
    printf 'Could not read init_devices_zero from /candle_config or /candle_md_node.\n' >&2
    exit 1
fi

init_will_zero=false
if [[ "$init_zero_setting" =~ [Tt]rue ]]; then
    init_will_zero=true
fi
if [[ "$init_will_zero" == true && "$ZERO_DRIVES" != true ]]; then
    printf '%s\n' "$init_zero_setting" >&2
    printf 'Refusing to initialize: init_devices_zero is true. Restart the node with init_devices_zero:=false or pass --zero at a verified reference.\n' >&2
    exit 1
fi

call_service /md/init_devices candle_ros2/srv/InitDevices \
    "{device_ids: ${DEVICE_IDS_YAML}, mode: 'IMPEDANCE'}"
if [[ "$ZERO_DRIVES" == true && "$init_will_zero" != true ]]; then
    call_service /md/zero candle_ros2/srv/Generic \
        "{device_ids: ${DEVICE_IDS_YAML}}"
fi

printf '\nApplying calibrated runtime PID gains...\n'
timeout "${SERVICE_TIMEOUT_SECONDS}s" ros2 topic pub --once \
    /md/position_command candle_ros2/msg/PositionPidCmd \
    "{device_ids: ${DEVICE_IDS_YAML},
      position_pid: [
        {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0},
        {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0},
        {kp: 12.5, ki: 0.5, kd: 0.05, i_windup: 1.0, max_output: 10.0}
      ],
      velocity_pid: [
        {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0},
        {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0},
        {kp: 1.5, ki: 0.02, kd: 0.0, i_windup: 1.0, max_output: 4.0}
      ]}"

call_service /md/open_gripper candle_ros2/srv/Generic \
    "{device_ids: ${DEVICE_IDS_YAML}}"
sleep "$MOVE_WAIT_SECONDS"

printf '\n=== Individual impedance close/open tests ===\n'
for id in "${DEVICE_IDS[@]}"; do
    printf '\nTesting drive %d individually.\n' "$id"
    call_service /md/close_gripper candle_ros2/srv/Generic \
        "{device_ids: [${id}]}"
    sleep "$MOVE_WAIT_SECONDS"
    call_service /md/open_gripper candle_ros2/srv/Generic \
        "{device_ids: [${id}]}"
    sleep "$MOVE_WAIT_SECONDS"
done

printf '\n=== Soft-close transition-gap tests ===\n'
for gap in "${SOFT_CLOSE_GAPS[@]}"; do
    for id in "${DEVICE_IDS[@]}"; do
        printf '\nTesting drive %d with a %d mm fast-to-slow transition gap.\n' "$id" "$gap"
        call_service /md/soft_close_gripper candle_ros2/srv/SoftCloseGripper \
            "{device_ids: [${id}], pre_close_gap_mm: ${gap}.0}"
        sleep "$SOFT_CLOSE_WAIT_SECONDS"
        call_service /md/open_gripper candle_ros2/srv/Generic \
            "{device_ids: [${id}]}"
        sleep "$MOVE_WAIT_SECONDS"
    done
done

trap - ERR INT TERM
printf '\nAll gripper tests completed successfully; grippers are open.\n'
