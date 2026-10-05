"""Writer SLAM host for the real robot.  Usage (from aess_x_ras/):
      python -m base_station.run_writer --port COM5
Waits for the firmware to finish warm-up, starts the mission, runs SLAM + the wall-following explorer, turns
event messages into beacons and hands them to the firmware to send by radio. Stops when the Writer is back
at its start (or Ctrl+C: the firmware stops the robot by itself after 0.5 s without commands)."""
import argparse
import json
import time

from .cli import add_transport_args, open_transport, loop
from .robot_host import RobotHost


def main():
    ap = add_transport_args(argparse.ArgumentParser(description=__doc__))
    ap.add_argument("--log", default="writer_log.json", help="where to save the pose / beacon log at the end")
    args = ap.parse_args()
    tp = open_transport(args)
    host = RobotHost("writer", tp)
    last = [0.0]

    def step():
        host.poll()
        if time.time() - last[0] > 2.0 and host.log:
            last[0] = time.time()
            p = host.pose
            print(f"t={host.log[-1]['t']:6.1f}s  pose=({p[0]:.2f}, {p[1]:.2f})  state={host.controller.state}  "
                  f"beacons={len(host.beacons_sent)} acked={len(host.acked)}", flush=True)
        return host.state != "DONE"
    loop(step)
    with open(args.log, "w") as f:
        json.dump(dict(log=host.log, beacons=[b.__dict__ for b in host.beacons_sent]), f, default=str)
    print("done:", host.stop_reason, "- log saved to", args.log)


if __name__ == "__main__":
    main()
