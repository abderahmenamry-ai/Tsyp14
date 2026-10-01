# Living Map - Task 1: Navigation stack (Writer robot)

PyBullet mine world + simulated LiDAR + BreezySLAM + right-hand wall-following.

## Run
    pip install -r requirements.txt        # then build BreezySLAM (see requirements.txt)
    python run_demo.py                     # headless, full mission, writes results/
    python run_demo.py --gui               # watch in the PyBullet window
    python run_demo.py --seed 3            # different noise realisation
    python tests/test_nav.py               # smoke tests

Outputs in `results/`: `demo.gif`, `map_final.png`, `trajectory.png`, `drift.png`, `log.csv`, `summary.json`.

## Interface for the rest of the team
    nav = NavStack(cfg)
    nav.run(callbacks=[fn])          # fn(nav) called every 0.1 s tick
    nav.get_pose()                   # (x_m, y_m, theta_rad), SLAM frame, origin = start pose
    nav.get_map()                    # uint8 [1600,1600], 0 wall / 127 unknown / 255 free, 5 cm/px
    nav.slam.world_to_pixel(x, y)    # SLAM metres -> (col,row) of the map
    nav.log                          # per-tick rows: true / estimated / odometry-only pose, state...
    nav.events.beacons               # BeaconRecord(kind, t, est_xy, true_xy, error_m)

`nav/odometry.py::Odometry.measure(v, w, dt, true_dyaw) -> (dxy_mm, dtheta_deg, dt)` is the seam
where Person 4's real encoder/IMU model plugs in. `nav/events.py` is a placeholder for the event
sensors / beacon dropping: it stamps beacons with the SLAM pose and records the true position too.

## How it works
| Module | Role |
|---|---|
| sim_world.py | Rock block with carved galleries (main gallery, 4 branches, dead end, bend, chamber, rubble-blocked branch), differential-drive robot |
| lidar.py | 360 rays via `p.rayTestBatch`, 8 m range, 1 cm noise, 0.5 % dropouts |
| odometry.py | Commanded motion + 3 % scale bias + noise; gyro heading with bias 0.05 deg/s |
| slam_node.py | BreezySLAM RMHC scan matcher with damped correction + wall-direction heading reference |
| explorer.py | States FOLLOW / TURN_LEFT / SEEK / RECOVER. Uses SLAM pose+map for stuck detection, map look-ahead guard, return-to-start stop |

## Results (default seed 7; seeds 1-3 are similar)
Full mission: ~448 s simulated, ~146 m driven, back at start, 0 wall contacts, all 3 event zones reached.
Position error vs ground truth: mean 0.14 m, max 0.28 m. Odometry alone ends 4-8 m off.

## Things we learned (useful for the report)
* BreezySLAM reads **0 mm as "no return"**; sending max range creates phantom walls.
* It expects sample i at `-fov/2 + i*fov/(n-1)`; the simulated LiDAR uses that spacing.
* Scan de-skewing is disabled (`scan_rate_hz=5000`) because simulated scans are instantaneous.
  On a real spinning LiDAR set the true rate.
* **Failure case:** stock RMHC in long straight corridors drifts in heading (about -0.2 deg/m measured)
  and turning in place makes it worse (-36 deg over 18 s of turning). Mitigations: damped fusion with
  odometry+gyro, no scan-match correction while rotating fast, and a wall-direction heading reference.
* **Limitation:** the wall-direction heading reference assumes an orthogonal mine layout (room-and-pillar).
  For irregular tunnels set `manhattan_gain = 0` and expect gyro-bias heading drift.
* Wall recesses were added to the tunnels; a perfectly featureless straight tunnel gives scan matching
  nothing to hold on to along the corridor (a documented failure case).
* The map is centred on the start pose, so the mine must lie within +-40 m of it.
* The simulated event zones are placeholders; the real sensors (MQ-2, tilt, PIR) belong to Person 4.
