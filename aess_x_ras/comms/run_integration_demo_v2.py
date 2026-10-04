"""
run_integration_demo_v2.py
Full Task 1 + Task 2 integration, Writer AND Executor both actually driving:
runs the Writer's exploration/SLAM, feeds every beacon it drops through the
comms pipeline (outside network -> command post), then runs the Executor with
a goal-seeking controller (A* over its own SLAM map) that visits each beacon
target in priority order. Beacons age on a shared mission clock (see
beacon.py::AGING_TICK_MS); expired ones are no longer part of the mission.

Run from this folder (comms/):
    python run_integration_demo_v2.py            # writes results/ (GIF + live-map PNGs)
    python run_integration_demo_v2.py --no-gif   # console output only, faster
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "living_map_nav"))

from nav.nav_stack import NavStack
from nav.config import Config
from nav.executor_controller import ExecutorController

from beacon_adapter import BeaconBridge
from outside_network import OutsideNetworkNode, CalibrationData
from command_post import CommandPost
from executor_mission import ExecutorMission


def run_writer(command_post, calibration, recorder=None):
    outside_network = OutsideNetworkNode(calibration, command_post)
    bridge = BeaconBridge(outside_network)

    def clock(nav):                       # shared mission clock = Writer sim time
        command_post.now_ms = int(nav.t * 1000)

    cfg = Config()
    nav = NavStack(cfg, gui=False)
    callbacks = [clock, bridge] + ([recorder] if recorder else [])
    reason = nav.run(callbacks=callbacks)
    print("Writer mission stopped:", reason)
    writer_map, end_ms = nav.get_map(), int(nav.t * 1000)
    nav.close()
    return writer_map, end_ms


def run_executor(command_post, start_ms, recorder=None):
    cfg = Config()
    mission = ExecutorMission(command_post)
    command_post.now_ms = start_ms
    controller = ExecutorController(cfg, None, mission)  # lidar is attached below
    nav = NavStack(cfg, gui=False, events=False)         # events=False: the Executor only consumes beacons
    controller.lidar = nav.lidar
    nav.explorer = controller

    def clock(nav):
        command_post.now_ms = start_ms + int(nav.t * 1000)

    reason = nav.run(callbacks=[clock] + ([recorder] if recorder else []))
    print("Executor mission stopped:", reason)
    nav.close()
    return controller


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gif", action="store_true")
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    args = ap.parse_args()

    command_post = CommandPost()

    # One-time calibration: GPS + compass bearing at the tunnel entrance.
    # Replace with real values once you have an actual deployment site.
    # Both Writer and Executor share this same calibration/origin.
    calibration = CalibrationData(entry_lat=34.4250, entry_lon=8.7842, heading_offset_deg=90)

    rec, phase = None, {"name": "WRITER exploring"}
    if not args.no_gif:
        from live_map import IntegratedRecorder
        os.makedirs(args.out, exist_ok=True)
        rec = IntegratedRecorder(command_post, lambda: phase["name"])

    print("=== Writer phase ===")
    writer_map, end_ms = run_writer(command_post, calibration, rec)
    print("\n--- Live map after Writer's run ---")
    print(command_post.live_map_summary())
    if rec:
        from live_map import save_live_map
        rec.freeze_map = True
        rec.map_snapshot = writer_map
        save_live_map(os.path.join(args.out, "live_map_after_writer.png"), command_post, writer_map,
                      title="Command post live map - after the Writer's run")

    print("\n=== Executor phase ===")
    phase["name"] = "EXECUTOR retrieving beacons"
    if rec:
        rec.reset_clock()
    ctrl = run_executor(command_post, end_ms, rec)

    print("\n--- Final live map ---")
    print(command_post.live_map_summary())
    visited = sum(1 for b, _ in command_post.received if command_post.status_of(b) == "visited")
    print(f"\nExecutor visited {visited}/{len(command_post.received)} beacons "
          f"({ctrl.plans_made} path plans, {ctrl.plan_failures} planning failures)")
    if rec:
        from live_map import save_live_map
        save_live_map(os.path.join(args.out, "live_map_final.png"), command_post, writer_map,
                      title="Command post live map - after the Executor's run")
        rec.save(os.path.join(args.out, "integrated_demo.gif"))
        print("saved GIF + live-map PNGs to", args.out)


if __name__ == "__main__":
    main()
