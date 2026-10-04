"""
generate_full_demo.py
Runs the full Writer -> Executor mission and records BOTH phases into
one combined demo GIF, so the submission has visual proof of the
complete Living Map story (not just the Writer's exploration).

Run from aess_x_ras/comms/:
    python generate_full_demo.py
Output: aess_x_ras/living_map_nav/results/demo_full.gif
"""
import os
import sys

NAV_PATH = os.path.join(os.path.dirname(__file__), "..", "living_map_nav")
sys.path.insert(0, NAV_PATH)

from nav.nav_stack import NavStack
from nav.config import Config
from nav.executor_controller import ExecutorController
from nav.viz import FrameRecorder

from beacon_adapter import BeaconBridge
from outside_network import OutsideNetworkNode, CalibrationData
from command_post import CommandPost
from executor_mission import ExecutorMission


def main():
    command_post = CommandPost()
    calibration = CalibrationData(entry_lat=34.4250, entry_lon=8.7842, heading_offset_deg=90)
    outside_network = OutsideNetworkNode(calibration, command_post)
    bridge = BeaconBridge(outside_network)

    cfg = Config()

    print("=== Recording Writer phase ===")
    nav_w = NavStack(cfg, gui=False)
    rec_w = FrameRecorder(nav_w, every_s=4.0)
    nav_w.run(callbacks=[bridge, rec_w])
    nav_w.close()
    print(f"Writer: {len(rec_w.frames)} frames captured")

    print("\n=== Recording Executor phase ===")
    mission = ExecutorMission(command_post)
    nav_e = NavStack(cfg, gui=False, events=False)
    nav_e.explorer = ExecutorController(cfg, nav_e.lidar, mission)
    rec_e = FrameRecorder(nav_e, every_s=2.0)  # shorter run, sample more often
    nav_e.run(callbacks=[rec_e])
    nav_e.close()
    print(f"Executor: {len(rec_e.frames)} frames captured")

    # Combine both phases into one GIF: Writer frames, then Executor frames
    combined = rec_w.frames + rec_e.frames
    out_path = os.path.join(NAV_PATH, "results", "demo_full.gif")
    combined[0].save(out_path, save_all=True, append_images=combined[1:],
                      duration=90, loop=0, optimize=True)
    print(f"\nSaved combined demo: {out_path} ({len(combined)} frames)")


if __name__ == "__main__":
    main()
