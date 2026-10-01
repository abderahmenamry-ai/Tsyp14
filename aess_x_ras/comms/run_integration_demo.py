"""
run_integration_demo.py
Full Task 1 + Task 2 integration: runs the real Writer robot
navigation/SLAM stack (living_map_nav) and pipes every beacon it
generates through this comms pipeline (outside network -> command
post -> Executor mission).

Assumes this repo layout:
    aess/
      living_map_nav/
        nav/            <- Task 1's package (config.py, nav_stack.py, ...)
      comms/
        (this file and the rest of Task 2)

If living_map_nav still has the extra nested folder from the original
zip (living_map_nav/living_map_nav/nav/...), flatten it first, or
adjust NAV_PATH below to match.

Run from aess/comms/:
    python run_integration_demo.py
"""
import os
import sys

NAV_PATH = os.path.join(os.path.dirname(__file__), "..", "living_map_nav")
sys.path.insert(0, NAV_PATH)

from nav.nav_stack import NavStack
from nav.config import Config

from beacon_adapter import BeaconBridge
from outside_network import OutsideNetworkNode, CalibrationData
from command_post import CommandPost
from executor_mission import ExecutorMission


def main():
    command_post = CommandPost()

    # One-time calibration: GPS + compass bearing at the tunnel entrance.
    # Replace with real values once you have an actual deployment site.
    calibration = CalibrationData(entry_lat=34.4250, entry_lon=8.7842, heading_offset_deg=90)
    outside_network = OutsideNetworkNode(calibration, command_post)
    bridge = BeaconBridge(outside_network)

    cfg = Config()
    nav = NavStack(cfg, gui=False)
    reason = nav.run(callbacks=[bridge])
    print("Writer mission stopped:", reason)

    print("\n--- Live map after Writer's run ---")
    print(command_post.live_map_summary())

    print("\n--- Executor working the mission ---")
    executor = ExecutorMission(command_post)
    executor.receive_mission()
    while True:
        target = executor.next_target()
        if target is None:
            print("Mission complete - no more targets.")
            break
        beacon, gps = target
        print(f"Executor heading to beacon #{beacon.beacon_id} "
              f"({beacon.event_type.name}, priority={beacon.priority.name}) "
              f"at GPS ({gps.lat:.6f}, {gps.lon:.6f})")
        executor.reached_target()

    nav.close()


if __name__ == "__main__":
    main()
