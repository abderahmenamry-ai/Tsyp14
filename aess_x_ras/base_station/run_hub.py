"""Outside-network node + command post.  Usage (from aess_x_ras/):
      python -m base_station.run_hub --port COM7 --lat 34.4250 --lon 8.7842 --heading 90
--lat/--lon are the GPS position of the tunnel entrance and --heading the compass bearing (degrees clockwise
from north) of the Writer's start direction; both robots must start from that same marked pose."""
import argparse
import time

from . import paths  # noqa: F401
from .cli import add_transport_args, open_transport, loop
from .hub_host import HubHost
from command_post import CommandPost


def main():
    ap = add_transport_args(argparse.ArgumentParser(description=__doc__))
    ap.add_argument("--lat", type=float, required=True)
    ap.add_argument("--lon", type=float, required=True)
    ap.add_argument("--heading", type=float, required=True)
    args = ap.parse_args()
    cp = CommandPost()
    hub = HubHost(open_transport(args), cp, args.lat, args.lon, args.heading)
    t0 = time.time(); last = [0.0]

    def step():
        cp.now_ms = int((time.time() - t0) * 1000)       # mission clock; set t0 when the Writer starts
        hub.poll()
        if time.time() - last[0] > 3.0:
            last[0] = time.time()
            print(cp.live_map_summary(), flush=True)
    loop(step)


if __name__ == "__main__":
    main()
