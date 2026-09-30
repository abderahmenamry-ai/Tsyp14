"""Run the Writer-robot exploration demo (PyBullet + simulated LiDAR + BreezySLAM).

    python run_demo.py                 # headless, writes results/
    python run_demo.py --gui           # watch it in the PyBullet window
    python run_demo.py --max-time 120  # quick smoke test
"""
import argparse
import json
import os
import numpy as np

from nav.config import Config
from nav.nav_stack import NavStack
from nav.viz import FrameRecorder, save_csv, save_final_figures


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--max-time", type=float, default=None)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--out", default="results")
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()

    cfg = Config()
    if args.max_time: cfg.max_time_s = args.max_time
    if args.seed is not None: cfg.seed = args.seed
    os.makedirs(args.out, exist_ok=True)

    nav = NavStack(cfg, gui=args.gui)
    rec = None if args.no_gif else FrameRecorder(nav, every_s=4.0)
    reason = nav.run(callbacks=[rec] if rec else [])
    print("stopped:", reason)

    save_csv(nav, os.path.join(args.out, "log.csv"))
    dr_err = save_final_figures(nav, args.out)
    if rec: rec.save(os.path.join(args.out, "demo.gif"))

    errs = np.array([r["pos_err"] for r in nav.log])
    summary = dict(
        stop_reason=reason, sim_time_s=round(nav.t, 1),
        distance_travelled_m=round(nav.explorer.distance_travelled, 1),
        slam_err_mean_m=round(float(errs.mean()), 3), slam_err_max_m=round(float(errs.max()), 3),
        slam_err_final_m=round(float(errs[-1]), 3),
        odometry_only_err_final_m=round(float(dr_err[-1]), 3),
        wall_collision_ticks=nav.world.collisions,
        beacons=[dict(kind=b.kind, t=round(b.t, 1), error_m=round(b.error_m, 3)) for b in nav.events.beacons],
    )
    with open(os.path.join(args.out, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    nav.close()


if __name__ == "__main__":
    main()
