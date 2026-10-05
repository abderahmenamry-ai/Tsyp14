"""run_sil.py - full software-in-the-loop mission. Run from aess_x_ras/:
      python3 firmware/sil/run_sil.py                 # perfect radio
      python3 firmware/sil/run_sil.py --loss 0.5      # 50 % of radio sends fail (the firmware must retry)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from firmware.sil.sil_sim import run_full   # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--loss", type=float, default=0.0, help="probability that a radio send fails (default 0)")
ap.add_argument("--seed", type=int, default=7)
ap.add_argument("--check", action="store_true", help="exit non-zero unless the mission succeeded (used as the pass/fail test)")
args = ap.parse_args()
r = run_full(p_ok=1.0 - args.loss, seed=args.seed)
print("\n=== SUMMARY ===")
for k in ("writer_state", "writer_time_s", "writer_dist_m", "writer_err_mean", "writer_err_max", "collisions",
          "beacons_composed", "events_suppressed", "beacons_acked", "beacons_at_command_post", "hub_duplicates", "pir_false_alarms",
          "radio_sent", "radio_delivered", "writer_radio_fail", "host_bad_frames",
          "executor_state", "executor_time_s", "executor_dist_m", "executor_err_max", "targets_received_by_executor",
          "visited", "total_beacons", "executor_collisions"):
    v = r[k]
    print(f"  {k:30s} {v:.2f}" if isinstance(v, float) else f"  {k:30s} {v}")
print(r["command_post"].live_map_summary())

if args.check:
    ok = (r["writer_state"] == "DONE" and r["executor_state"] == "DONE" and r["beacons_composed"] == 3
          and r["beacons_acked"] == 3 and r["beacons_at_command_post"] == 3 and r["visited"] == 3
          and r["collisions"] == 0 and r["executor_collisions"] == 0 and r["writer_err_max"] < 0.8)
    print("\nSIL CHECK:", "PASSED" if ok else "FAILED")
    sys.exit(0 if ok else 1)
