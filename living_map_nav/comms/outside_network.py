"""
outside_network.py
Simulated outside-network node: receives beacons (a direct function call
in simulation, standing in for the real RF link) and translates local
robot coordinates into real GPS, then forwards the result to the command
post. Mirrors firmware/frame_translation.h.
"""

import math
from dataclasses import dataclass

METERS_PER_DEG_LAT = 111320.0


@dataclass
class CalibrationData:
    """One-time calibration, set at deployment (e.g. read from a
    phone GPS + compass at the tunnel entrance before the mission)."""
    entry_lat: float
    entry_lon: float
    heading_offset_deg: float


@dataclass
class GPSCoord:
    lat: float
    lon: float


def translate_to_gps(beacon, cal: CalibrationData) -> GPSCoord:
    """Flat-earth approximation - accurate at tunnel/mine scale."""
    x_m = beacon.x_coord_cm / 100.0
    y_m = beacon.y_coord_cm / 100.0
    theta = math.radians(cal.heading_offset_deg)

    north_offset_m = x_m * math.cos(theta) - y_m * math.sin(theta)
    east_offset_m = x_m * math.sin(theta) + y_m * math.cos(theta)

    delta_lat = north_offset_m / METERS_PER_DEG_LAT
    delta_lon = east_offset_m / (METERS_PER_DEG_LAT * math.cos(math.radians(cal.entry_lat)))

    return GPSCoord(lat=cal.entry_lat + delta_lat, lon=cal.entry_lon + delta_lon)


class OutsideNetworkNode:
    """Receives beacons and forwards translated data to the command post.
    All robot <-> command post communication passes through here -
    satisfies the 'no direct link' rule."""

    def __init__(self, calibration: CalibrationData, command_post):
        self.calibration = calibration
        self.command_post = command_post

    def receive_beacon(self, beacon):
        gps = translate_to_gps(beacon, self.calibration)
        self.command_post.ingest(beacon, gps)
