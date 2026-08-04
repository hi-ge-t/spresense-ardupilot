#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

from copy import deepcopy
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import m1_copter_bench_check as checker  # noqa: E402


class Parameter:
    def __init__(self, name):
        self.param_id = name


def expect(condition, message):
    if not condition:
        raise AssertionError(message)


def valid_monitor_state():
    state = checker.new_monitor_state(60.0)
    state["duration_observed_seconds"] = 60.0
    state["heartbeat_count"] = 60
    state["heartbeat_max_gap_seconds"] = 1.1
    state["gps"]["message_count"] = 60
    state["raw_imu"].update({
        "message_count": 300,
        "first_time_usec": 1,
        "last_time_usec": 60000001,
        "minimum_axes": [1, -2, -9800, 0, 0, 0],
        "maximum_axes": [3, 2, -9700, 0, 0, 0],
    })
    return state


def expect_invalid(state, expected):
    try:
        checker.validate_monitor_state(state)
    except checker.runtime_checker.CheckError as error:
        expect(expected in str(error), f"unexpected failure: {error}")
    else:
        raise AssertionError(f"invalid state accepted: {expected}")


def main() -> int:
    expect(
        checker.parameter_name(Parameter(b"GCS_PID_MASK\x00")) ==
        checker.SAFE_PARAMETER,
        "byte parameter ID is normalized",
    )
    expect(
        checker.parameter_name(Parameter("GCS_PID_MASK\x00")) ==
        checker.SAFE_PARAMETER,
        "string parameter ID is normalized",
    )
    expect(checker.choose_test_value(0.0) == 1.0, "zero toggles to one")
    expect(checker.choose_test_value(1.0) == 0.0, "one toggles to zero")
    expect(checker.choose_test_value(7.0) == 1.0, "other values use one")

    valid = valid_monitor_state()
    checker.validate_monitor_state(valid)

    armed = deepcopy(valid)
    armed["armed_observed"] = True
    expect_invalid(armed, "armed state")

    stalled_heartbeat = deepcopy(valid)
    stalled_heartbeat["heartbeat_max_gap_seconds"] = 10.1
    expect_invalid(stalled_heartbeat, "heartbeat gap")

    stalled_imu = deepcopy(valid)
    stalled_imu["raw_imu"]["last_time_usec"] = 1
    expect_invalid(stalled_imu, "timestamp did not advance")

    fixed_imu = deepcopy(valid)
    fixed_imu["raw_imu"]["maximum_axes"] = list(
        fixed_imu["raw_imu"]["minimum_axes"]
    )
    expect_invalid(fixed_imu, "axes did not change")

    missing_gps = deepcopy(valid)
    missing_gps["gps"]["message_count"] = 0
    expect_invalid(missing_gps, "GPS_RAW_INT")

    print(
        "spresense_m1_copter_bench_host=PASS "
        "safe_param=guarded restoration=required runtime=bounded"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
