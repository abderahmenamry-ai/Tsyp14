"""Central configuration for the Living Map navigation stack (Task 1).

All distances are in metres / seconds / radians unless a name says otherwise
(BreezySLAM itself works in millimetres and degrees; conversions happen only
inside slam_node.py).
"""
from dataclasses import dataclass


@dataclass
class Config:
    # --- timing -----------------------------------------------------------
    dt: float = 0.1                 # control / LiDAR period (10 Hz)
    physics_hz: int = 240           # PyBullet internal step rate
    seed: int = 7

    # --- robot (differential drive, cylinder body) ------------------------
    robot_radius: float = 0.20
    robot_height: float = 0.30
    robot_mass: float = 5.0
    wheel_base: float = 0.30
    v_max: float = 0.60             # m/s
    w_max: float = 1.60             # rad/s

    # --- simulated 2D LiDAR (RPLidar-A1 like) -----------------------------
    lidar_rays: int = 360           # 1 ray / degree
    lidar_fov_deg: float = 360.0
    lidar_range_m: float = 8.0
    lidar_noise_std_m: float = 0.01
    lidar_dropout_prob: float = 0.005   # random "no return" rays
    lidar_height: float = 0.15

    # --- BreezySLAM --------------------------------------------------------
    map_size_px: int = 1600
    map_size_m: float = 80.0        # 5 cm / pixel; centred on the start pose, so the mine must lie within +-40 m of it
    slam_sigma_xy_mm: float = 30.0
    slam_sigma_theta_deg: float = 2.0
    slam_max_search_iter: int = 1000

    # --- odometry error model (this is what makes SLAM drift matter) ------
    odom_scale_bias: float = 0.03       # systematic +3 % on distance
    odom_yaw_bias: float = 0.02         # systematic +2 % on rotation
    odom_noise_std: float = 0.04        # per-step multiplicative noise
    true_slip_std: float = 0.02         # real wheel slip (robot != command)

    # --- IMU gyro (simulated): heading rate = truth + constant bias + noise -
    gyro_bias_dps: float = 0.05
    gyro_noise_deg: float = 0.05        # per-step noise (deg)

    # --- scan-match / odometry fusion (see slam_node.py) -------------------
    fuse_gain_xy: float = 0.5           # fraction of the scan-match XY correction applied
    fuse_gain_theta: float = 0.0        # fraction of the scan-match heading correction applied
    fuse_clip_xy_mm: float = 40.0       # max XY correction per step
    fuse_clip_theta_deg: float = 0.6    # max heading correction per step

    # --- wall-direction ("Manhattan") heading correction -------------------
    # Room-and-pillar mines are built on a roughly orthogonal grid, so long straight wall
    # segments are almost always parallel/perpendicular to the start heading. Measuring how far
    # those segments are from the nearest 90-degree multiple gives an absolute, drift-free heading
    # reference that removes the gyro bias. Set manhattan_gain = 0 for non-orthogonal mines.
    manhattan_gain: float = 0.30
    manhattan_clip_deg: float = 0.8
    manhattan_min_segments: int = 25

    # --- wall-following controller ----------------------------------------
    wall_target_m: float = 0.60         # desired distance to right wall
    front_stop_m: float = 0.75          # start turning left
    front_clear_m: float = 1.50         # stop turning left
    wall_lost_m: float = 1.70           # right wall considered gone
    k_heading: float = 1.6
    k_dist: float = 2.2
    cruise_v: float = 0.55
    turn_w: float = 0.60                # in-place turn rate (rad/s) - kept slow so scan matching keeps up

    # --- mission -----------------------------------------------------------
    max_time_s: float = 900.0
    return_radius_m: float = 1.5
    min_out_distance_m: float = 15.0    # must get this far before "return" counts
