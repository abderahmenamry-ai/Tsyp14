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

from beacon import EventType, make_beacon, is_expired
from outside_network import OutsideNetworkNode, CalibrationData, translate_to_gps
from command_post import CommandPost
from executor_mission import ExecutorMission

# Fake trigger zones for this isolated test only (not the real mine layout -
# see living_map_nav/nav/sim_world.py::EVENT_ZONES for that)
# The fake robot drives along y = 0, so the zones must sit on y = 0 to be hit.
DEMO_TRIGGER_ZONES = [
    (3.0, 0.0, 0.4, EventType.GAS, 420),
    (6.0, 0.0, 0.4, EventType.COLLAPSE, 780),
    (9.5, 0.0, 0.4, EventType.TRAPPED, 1),
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

    # 1. every zone must have produced a beacon that reached the command post
    assert len(command_post.received) == 3, f"expected 3 beacons, got {len(command_post.received)}"

    # 2. mission order: high priority first (freshest first within a priority), gas last
    order = [b.event_type for b, _ in command_post.build_mission()]
    assert order == [EventType.TRAPPED, EventType.COLLAPSE, EventType.GAS], order

    # 3. frame translation: a beacon 3 m "forward" with heading offset 90 deg (east)
    #    must land east of the entrance (same latitude, larger longitude)
    gas_beacon, gas_gps = command_post.received[0]
    assert abs(gas_gps.lat - calibration.entry_lat) < 1e-5
    assert gas_gps.lon > calibration.entry_lon

    # 3b. y axis: SLAM frame has +y to the LEFT. Facing east (heading 90), 5 m ahead and 5 m
    #     left must be 5 m NORTH and 5 m EAST of the entrance (regression test for a mirrored map)
    import math
    left = make_beacon(20, 1, EventType.GAS, 500, 500, 0, 0, 0)
    g = translate_to_gps(left, calibration)
    north_m = (g.lat - calibration.entry_lat) * 111320.0
    east_m = (g.lon - calibration.entry_lon) * 111320.0 * math.cos(math.radians(calibration.entry_lat))
    assert abs(north_m - 5.0) < 0.05 and abs(east_m - 5.0) < 0.05, (north_m, east_m)

    # 4. aging: a routine marker written at t=0 (TTL 5 ticks = 150 s) must expire
    #    before an urgent beacon (TTL 20 ticks = 600 s) written at the same time
    low = make_beacon(10, 1, EventType.EXPLORED, 0, 0, 0, 0, 0)
    high = make_beacon(11, 1, EventType.TRAPPED, 0, 0, 0, 1, 0)
    assert not is_expired(low, 149_000) and is_expired(low, 150_000)
    assert not is_expired(high, 599_000) and is_expired(high, 600_000)
    command_post.now_ms = 200_000
    outside_network.receive_beacon(low)
    outside_network.receive_beacon(high)
    assert command_post.status_of(low) == "expired" and command_post.status_of(high) == "pending"
    assert low.beacon_id not in [b.beacon_id for b, _ in command_post.build_mission()]
    # remove the extra beacons again before the mission walk-through below
    command_post.received = command_post.received[:3]
    command_post.now_ms = None

    print("\n--- Executor working the mission ---")
    executor = ExecutorMission(command_post)
    executor.receive_mission()
    visited = 0
    while True:
        target = executor.next_target()
        if target is None:
            print("Mission complete.")
            break
        beacon, gps = target
        print(f"Executor -> beacon #{beacon.beacon_id} ({beacon.event_type.name}, "
              f"priority={beacon.priority.name}) at ({gps.lat:.6f}, {gps.lon:.6f})")
        executor.reached_target()
        visited += 1
    assert visited == 3, visited
    print("\nALL CHECKS PASSED")


if __name__ == "__main__":
    main()
