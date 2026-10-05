"""Executor SLAM host for the real robot.  Usage (from aess_x_ras/):
      python -m base_station.run_executor --port COM6
The mission list arrives by radio (hub -> Executor firmware); the robot stays still until the whole list is in,
then drives to each target in priority order with A* path planning on its own SLAM map."""
import argparse
import time

from .cli import add_transport_args, open_transport, loop
from .robot_host import RobotHost


def main():
    ap = add_transport_args(argparse.ArgumentParser(description=__doc__))
    args = ap.parse_args()
    host = RobotHost("executor", open_transport(args))
    last = [0.0]

    def step():
        host.poll()
        if time.time() - last[0] > 2.0:
            last[0] = time.time()
            p = host.pose
            print(f"pose=({p[0]:.2f}, {p[1]:.2f})  state={host.controller.state}  mission_ready={host.mission.ready}  "
                  f"targets_left={len(host.mission.targets)}", flush=True)
        return host.state != "DONE"
    loop(step)
    print("done:", host.stop_reason)


if __name__ == "__main__":
    main()
