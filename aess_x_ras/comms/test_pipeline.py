"""
test_pipeline.py
Standalone test for the comms pipeline (event -> beacon -> outside
network -> command post -> mission), using a fake straight-line
robot path instead of the real nav stack. No PyBullet/BreezySLAM
dependency, so it runs fast - useful as a quick sanity check that
doesn't require the full simulation.

For the real integration test (actual Writer robot + SLAM), see
run_integration_demo.py instead.

Run with: python test_pipeline.py
"""

from beacon import EventType, make_beacon
from outside_network import OutsideNetworkNode, CalibrationData
from command_post import CommandPost
from executor_mission import ExecutorMission

# Fake trigger zones for this isolated test only (not the real mine layout -
# see living_map_nav/nav/sim_world.py::EVENT_ZONES for that)
DEMO_TRIGGER_ZONES = [
    (3.0, 0.1, 0.4, EventType.GAS, 420),
    (6.0, -0.2, 0.4, EventType.COLLAPSE, 780),
    (9.0, 0.15, 0.4, EventType.TRAPPED, 1),
]


def run_fake_writer_pass(outside_network):
    beacon_id = 0
    triggered = set()
    for step in range(200):
        x, y = step * 0.05, 0.0
        for i, (zx, zy, r, event_type, sensor_value) in enumerate(DEMO_TRIGGER_ZONES):
            if i in triggered:
                continue
            if (x - zx) ** 2 + (y - zy) ** 2 <= r ** 2:
                triggered.add(i)
                beacon_id += 1
                beacon = make_beacon(
                    beacon_id=beacon_id, writer_id=1, event_type=event_type,
                    x_cm=x * 100, y_cm=y * 100, heading=0,
                    sensor_value=sensor_value, timestamp_ms=step * 50,
                )
                outside_network.receive_beacon(beacon)


def main():
    calibration = CalibrationData(entry_lat=34.4250, entry_lon=8.7842, heading_offset_deg=90)
    command_post = CommandPost()
    outside_network = OutsideNetworkNode(calibration, command_post)

    print("--- Running fake Writer pass ---")
    run_fake_writer_pass(outside_network)

    print("\n--- Live map ---")
    print(command_post.live_map_summary())

    print("\n--- Executor working the mission ---")
    executor = ExecutorMission(command_post)
    executor.receive_mission()
    while True:
        target = executor.next_target()
        if target is None:
            print("Mission complete.")
            break
        beacon, gps = target
        print(f"Executor -> beacon #{beacon.beacon_id} ({beacon.event_type.name}, "
              f"priority={beacon.priority.name}) at ({gps.lat:.6f}, {gps.lon:.6f})")
        executor.reached_target()


if __name__ == "__main__":
    main()
