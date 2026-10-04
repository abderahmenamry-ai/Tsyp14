"""
beacon.py
Beacon message format for the simulation. Mirrors firmware/beacon_protocol.h
field-for-field, so simulation behavior and real firmware behavior stay
consistent.
"""

from dataclasses import dataclass
from enum import IntEnum


class EventType(IntEnum):
    EXPLORED = 0   # routine marker, area checked, nothing found
    GAS = 1        # toxic gas detected
    COLLAPSE = 2   # structural collapse / instability detected
    TRAPPED = 3    # trapped worker detected


class Priority(IntEnum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2


# Starting TTL and rebroadcast interval (ms) per priority level
TTL_BY_PRIORITY = {Priority.LOW: 5, Priority.MEDIUM: 12, Priority.HIGH: 20}
REBROADCAST_MS_BY_PRIORITY = {Priority.LOW: 10000, Priority.MEDIUM: 5000, Priority.HIGH: 2000}

# --- Message aging -----------------------------------------------------------
# A beacon's TTL is its lifetime measured in aging ticks: one TTL unit is lost
# every AGING_TICK_MS of mission time since the beacon was written. So a beacon
# lives TTL * AGING_TICK_MS:  LOW 150 s,  MEDIUM 360 s,  HIGH 600 s.
# Urgent finds (trapped worker, collapse) therefore persist longest, and routine
# "explored" markers fade first. The rebroadcast interval is separate: it only
# sets how often a live beacon is re-sent so it survives packet loss.
# (Counting one TTL unit per rebroadcast would give HIGH beacons the SHORTEST
# life - 20 x 2 s = 40 s - which is the opposite of what we want.)
# Assumption: every node shares one mission clock (all started at deployment).
AGING_TICK_MS = 30_000

# Default priority mapping per event type
PRIORITY_BY_EVENT = {
    EventType.TRAPPED: Priority.HIGH,
    EventType.COLLAPSE: Priority.HIGH,
    EventType.GAS: Priority.MEDIUM,
    EventType.EXPLORED: Priority.LOW,
}


@dataclass
class BeaconMessage:
    beacon_id: int
    writer_id: int
    event_type: EventType
    priority: Priority
    x_coord_cm: int
    y_coord_cm: int
    heading: int
    sensor_value: int
    timestamp_ms: int
    ttl: int

    def is_urgent(self) -> bool:
        return self.priority >= Priority.MEDIUM


def make_beacon(beacon_id: int, writer_id: int, event_type: EventType,
                x_cm: float, y_cm: float, heading: int, sensor_value: int,
                timestamp_ms: int) -> BeaconMessage:
    """Builds a beacon message. Called by beacon_adapter.py whenever
    the Writer's nav stack reports a new event."""
    priority = PRIORITY_BY_EVENT[event_type]
    return BeaconMessage(
        beacon_id=beacon_id,
        writer_id=writer_id,
        event_type=event_type,
        priority=priority,
        x_coord_cm=int(x_cm),
        y_coord_cm=int(y_cm),
        heading=heading,
        sensor_value=sensor_value,
        timestamp_ms=timestamp_ms,
        ttl=TTL_BY_PRIORITY[priority],
    )


def ttl_remaining(beacon: BeaconMessage, now_ms: int) -> int:
    """TTL units left at mission time `now_ms` (0 = expired)."""
    elapsed = max(0, now_ms - beacon.timestamp_ms)
    return max(0, beacon.ttl - elapsed // AGING_TICK_MS)


def is_expired(beacon: BeaconMessage, now_ms: int) -> bool:
    return ttl_remaining(beacon, now_ms) == 0
