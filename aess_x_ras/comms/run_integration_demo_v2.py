"""
run_integration_demo.py
Full Task 1 + Task 2 integration, Writer AND Executor both actually
driving: runs the Writer's exploration/SLAM, feeds every beacon it
drops through the comms pipeline (outside network -> command post),
then runs the Executor with a goal-seeking controller that visits
each beacon target in priority order using its own SLAM run.

Requires the nav_stack.py patch (see nav_stack_patch.txt) and
executor_controller.py to be in place under nav/.

Run from living_map_nav/comms/ (adjust NAV_PATH below if your repo
layout differs - see the note in README.md about the nesting):
    python run_integration_demo.py
"""
import os
import sys

NAV_PATH = os.path.join(os.path.dirname(__file__), "..", "living_map_nav")
sys.path.insert(0, NAV_PATH)

from nav.nav_stack import NavStack
from nav.config import Config
from nav.executor_controller import ExecutorController

from beacon_adapter import BeaconBridge
from outside_network import OutsideNetworkNode, CalibrationData
from command_post import CommandPost
from executor_mission import ExecutorMission


def run_writer(command_post, calibration):
    outside_network = OutsideNetworkNode(calibration, command_post)
    bridge = BeaconBridge(outside_network)

    cfg = Config()
    nav = NavStack(cfg, gui=False)
    reason = nav.run(callbacks=[bridge])
    print("Writer mission stopped:", reason)
    nav.close()


def run_executor(command_post):
    cfg = Config()
    mission = ExecutorMission(command_post)
    # events=False: the Executor doesn't drop its own beacons, only consumes them
    controller_box = {}

    def make_controller(lidar):
        ctrl = ExecutorController(cfg, lidar, mission)
        controller_box["ctrl"] = ctrl
        return ctrl

    # NavStack builds lidar internally, so we construct the controller
    # after NavStack exists, then swap it in - simplest without touching
    # NavStack's constructor order further.
    nav = NavStack(cfg, gui=False, events=False)
    nav.explorer = make_controller(nav.lidar)
    reason = nav.run(callbacks=[])
    print("Executor mission stopped:", reason)
    nav.close()


def main():
    command_post = CommandPost()

    # One-time calibration: GPS + compass bearing at the tunnel entrance.
    # Replace with real values once you have an actual deployment site.
    # Both Writer and Executor share this same calibration/origin.
    calibration = CalibrationData(entry_lat=34.4250, entry_lon=8.7842, heading_offset_deg=90)

    print("=== Writer phase ===")
    run_writer(command_post, calibration)

    print("\n--- Live map after Writer's run ---")
    print(command_post.live_map_summary())

    print("\n=== Executor phase ===")
    run_executor(command_post)

    print("\n--- Final live map ---")
    print(command_post.live_map_summary())


if __name__ == "__main__":
    main()
