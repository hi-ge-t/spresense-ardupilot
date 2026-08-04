#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Tools/spresense"))

import m1_rover_gcs_sequence as sequence  # noqa: E402
import run_m1_rover_gcs_sitl as gcs_sitl  # noqa: E402
import run_m1_rover_sitl_autonomy as sitl  # noqa: E402


class FakeMavlink:
    MAV_FRAME_GLOBAL = 0
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
    MAV_FRAME_GLOBAL_INT = 5
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
    MAV_FRAME_MISSION = 2
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_DO_CHANGE_SPEED = 178
    MAV_CMD_COMPONENT_ARM_DISARM = 400
    MAV_MODE_FLAG_SAFETY_ARMED = 128
    MAV_MISSION_ACCEPTED = 0
    MAV_MISSION_TYPE_MISSION = 0
    MAV_MISSION_TYPE_FENCE = 1
    MAV_RESULT_FAILED = 4


class FakeMessage:
    def __init__(self, message_type, **values):
        self._message_type = message_type
        self._source_system = values.pop("source_system", 1)
        for key, value in values.items():
            setattr(self, key, value)

    def get_type(self):
        return self._message_type

    def get_srcSystem(self):
        return self._source_system


def message_from_item(item):
    return FakeMessage(
        "MISSION_ITEM_INT",
        seq=item.seq,
        frame=item.frame,
        command=item.command,
        current=item.current,
        autocontinue=item.autocontinue,
        param1=item.param1,
        param2=item.param2,
        param3=item.param3,
        param4=item.param4,
        x=item.x,
        y=item.y,
        z=item.z,
    )


class FakeMav:
    def __init__(self, link):
        self.link = link
        self.pending = None
        self.request_int = False
        self.duplicate_first_request = False
        self.item_send_counts = {}

    def mission_request_list_send(self, system, component):
        del system, component
        self.link.messages.append(
            FakeMessage("MISSION_COUNT", count=len(self.link.mission))
        )

    def mission_request_int_send(self, system, component, seq):
        del system, component
        self.link.messages.append(message_from_item(self.link.mission[seq]))

    def mission_ack_send(self, system, component, result):
        del system, component, result

    def mission_clear_all_send(self, system, component):
        del system, component
        self.link.mission = []
        self.link.messages.append(
            FakeMessage("MISSION_ACK", type=FakeMavlink.MAV_MISSION_ACCEPTED)
        )

    def mission_count_send(self, system, component, count):
        del system, component
        self.pending = [None] * count
        if count == 0:
            self.link.mission = []
            self.link.messages.append(
                FakeMessage(
                    "MISSION_ACK", type=FakeMavlink.MAV_MISSION_ACCEPTED
                )
            )
        else:
            request = (
                "MISSION_REQUEST_INT" if self.request_int
                else "MISSION_REQUEST"
            )
            self.link.messages.append(FakeMessage(request, seq=0))
            if self.duplicate_first_request:
                self.link.messages.append(FakeMessage(request, seq=0))

    def store_item(self, item):
        self.pending[item.seq] = item
        next_seq = item.seq + 1
        if next_seq < len(self.pending):
            request = (
                "MISSION_REQUEST_INT" if self.request_int
                else "MISSION_REQUEST"
            )
            self.link.messages.append(FakeMessage(request, seq=next_seq))
        else:
            self.link.mission = list(self.pending)
            self.link.messages.append(
                FakeMessage(
                    "MISSION_ACK", type=FakeMavlink.MAV_MISSION_ACCEPTED
                )
            )

    def mission_item_int_send(
        self,
        system,
        component,
        seq,
        frame,
        command,
        current,
        autocontinue,
        param1,
        param2,
        param3,
        param4,
        x,
        y,
        z,
    ):
        del system, component
        self.item_send_counts[seq] = self.item_send_counts.get(seq, 0) + 1
        self.store_item(sequence.MissionItem(
            seq, frame, command, current, autocontinue,
            param1, param2, param3, param4, x, y, z,
        ))

    def command_long_send(self, system, component, command, confirmation, *params):
        del system, component, confirmation, params
        self.link.messages.extend([
            FakeMessage(
                "COMMAND_ACK", command=command,
                result=FakeMavlink.MAV_RESULT_FAILED,
            ),
            FakeMessage("HEARTBEAT", base_mode=0),
        ])


class FakeLink:
    def __init__(self, mission):
        self.mission = list(mission)
        self.messages = []
        self.mav = FakeMav(self)

    def recv_match(self, blocking, timeout):
        del blocking, timeout
        if not self.messages:
            return None
        return self.messages.pop(0)


def main():
    mavlink = FakeMavlink()
    diagnostics = bytearray()
    if any(b"GNSS" in marker or b"PWBIMU" in marker
           for marker in sequence.GCS_RUNTIME_MARKERS):
        raise AssertionError("GCS protocol gate must not claim sensor runtime")
    original = sequence.build_dry_run_mission(
        mavlink, 400713770, -1052297900
    )
    link = FakeLink(original)
    link.messages.extend([
        FakeMessage("HEARTBEAT", base_mode=0),
        FakeMessage("HEARTBEAT", base_mode=0),
    ])
    sequence.wait_disarmed_heartbeats(
        link, mavlink, 1, diagnostics, 2, 1.0
    )
    arm_result = sequence.request_denied_arm(
        link, mavlink, 1, 1, 0, diagnostics
    )
    if arm_result != mavlink.MAV_RESULT_FAILED:
        raise AssertionError("fail-closed ARM result was not preserved")
    downloaded = sequence.download_mission(
        link, mavlink, 1, 1, diagnostics
    )
    if not sequence.missions_equal(original, downloaded):
        raise AssertionError("mission download did not preserve items")

    replacement = sequence.build_dry_run_mission(
        mavlink, 350000000, 1390000000
    )
    if replacement[0].current != 0:
        raise AssertionError("test mission must not depend on current-item coercion")
    if any(item.frame != mavlink.MAV_FRAME_GLOBAL for item in replacement):
        raise AssertionError("test mission must use ArduPilot's stored frame")
    if sequence.missions_equal(original, replacement):
        raise AssertionError("dry-run coordinate fallback did not change mission")
    current_normalized = list(replacement)
    current_normalized[0] = sequence.MissionItem(
        **{
            **replacement[0].__dict__,
            "current": 1,
        }
    )
    if not sequence.missions_equal(replacement, current_normalized):
        raise AssertionError("ephemeral current-item flag broke mission equality")
    dynamic_home = list(replacement)
    dynamic_home[0] = sequence.MissionItem(
        **{
            **replacement[0].__dict__,
            "frame": mavlink.MAV_FRAME_GLOBAL_INT,
            "x": 0,
            "y": 0,
            "z": 123.0,
        }
    )
    if not sequence.missions_equal(replacement, dynamic_home):
        raise AssertionError("dynamic HOME fields broke mission equality")
    link.messages.append(
        FakeMessage(
            "MISSION_REQUEST_INT",
            seq=99,
            mission_type=mavlink.MAV_MISSION_TYPE_FENCE,
        )
    )
    link.mav.duplicate_first_request = True
    sequence.upload_mission(link, mavlink, 1, 1, replacement, diagnostics)
    link.mav.duplicate_first_request = False
    if link.mav.item_send_counts.get(0, 0) < 2:
        raise AssertionError("duplicate mission request was not retried")
    round_trip = sequence.download_mission(
        link, mavlink, 1, 1, diagnostics
    )
    if not sequence.missions_equal(replacement, round_trip):
        raise AssertionError("mission upload/download round trip failed")
    if sequence.missions_equal(original, round_trip):
        raise AssertionError("different mission was treated as equal")
    link.messages.append(
        FakeMessage(
            "MISSION_COUNT",
            count=99,
            mission_type=mavlink.MAV_MISSION_TYPE_FENCE,
        )
    )
    filtered = sequence.download_mission(
        link, mavlink, 1, 1, diagnostics
    )
    if not sequence.missions_equal(replacement, filtered):
        raise AssertionError("foreign mission type was not ignored")

    if link.mav.request_int:
        raise AssertionError("legacy request path was not exercised")
    link.mav.request_int = True
    sequence.upload_mission(link, mavlink, 1, 1, original, diagnostics)
    restored = sequence.download_mission(link, mavlink, 1, 1, diagnostics)
    if not sequence.missions_equal(original, restored):
        raise AssertionError("mission restoration failed")

    command = sitl.build_command(ROOT)
    if command[-2:] != ["build.Rover", "test.Rover.DriveMission"]:
        raise AssertionError("SITL runner does not select Rover DriveMission")
    if sitl.build_command(ROOT, skip_build=True)[-1] != "test.Rover.DriveMission":
        raise AssertionError("SITL no-build command lost DriveMission")
    gcs_build = gcs_sitl.build_rover_command(ROOT)
    if gcs_build[-1] != "build.Rover":
        raise AssertionError("GCS SITL runner does not build Rover")
    gcs_command = gcs_sitl.build_sitl_command(ROOT, 8)
    if gcs_command[1] != "-I8" or "--serial0" not in gcs_command:
        raise AssertionError("GCS SITL runner lost isolated TCP instance")
    if "-w" not in gcs_command:
        raise AssertionError("GCS SITL runner must start from isolated state")

    print(
        "spresense_m1_rover_gcs_sequence_host=PASS "
        "mission=round-trip restore=verified "
        "sitl=DriveMission+real-MAVLink"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
