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
