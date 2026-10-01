"""PLACEHOLDER hook for Person 4 (event triggers) and Lead C (beacon format).

It shows how the navigation stack's pose is meant to be consumed:
the *physical* event is detected at the robot's TRUE position (the sensor really
is next to the gas / collapse / worker), but the beacon is stamped with the
SLAM-ESTIMATED pose. The difference is the localisation error that ends up in
the beacon - exactly the failure case to discuss in the report.
"""
import math
from dataclasses import dataclass, field
from .sim_world import EVENT_ZONES, START_POSE


@dataclass
class BeaconRecord:
    kind: str
    t: float
    est_xy: tuple           # SLAM frame (what goes into the beacon)
    true_xy: tuple          # SLAM frame (ground truth, for evaluation only)
    error_m: float = field(init=False)

    def __post_init__(self):
        self.error_m = math.hypot(self.est_xy[0] - self.true_xy[0], self.est_xy[1] - self.true_xy[1])


class EventSimulator:
    def __init__(self, zones=None):
        self.zones = [dict(z, done=False) for z in (zones or EVENT_ZONES)]
        self.beacons = []

    def check(self, t, true_pose_world, est_pose_slam):
        """Call every tick; returns list of new BeaconRecord."""
        new = []
        tx, ty = true_pose_world[0], true_pose_world[1]
        for z in self.zones:
            if not z["done"] and math.hypot(tx - z["x"], ty - z["y"]) < z["r"]:
                z["done"] = True
                rec = BeaconRecord(z["kind"], t,
                                   (est_pose_slam[0], est_pose_slam[1]),
                                   (tx - START_POSE[0], ty - START_POSE[1]))
                self.beacons.append(rec)
                new.append(rec)
        return new
