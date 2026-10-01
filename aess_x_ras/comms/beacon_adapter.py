"""
beacon_adapter.py
The real integration point between Task 1 (nav_stack's BeaconRecord,
in living_map_nav/nav/events.py) and Task 2 (this package's
BeaconMessage + outside_network.py).

Use as a callback to NavStack.run():
    bridge = BeaconBridge(outside_network)
    nav.run(callbacks=[bridge])
"""
from beacon import EventType, make_beacon

# Task 1's event "kind" strings -> our EventType enum
KIND_TO_EVENT_TYPE = {
    "gas": EventType.GAS,
    "collapse": EventType.COLLAPSE,
    "worker": EventType.TRAPPED,
}


class BeaconBridge:
    def __init__(self, outside_network, writer_id=1):
        self.outside_network = outside_network
        self.writer_id = writer_id
        self._seen = 0  # how many of nav.events.beacons we've already forwarded

    def __call__(self, nav):
        """Called every tick by NavStack.run(). Forwards any new beacons."""
        beacons = nav.events.beacons
        for rec in beacons[self._seen:]:
            event_type = KIND_TO_EVENT_TYPE.get(rec.kind)
            if event_type is None:
                continue  # unknown kind, skip rather than crash

            x_m, y_m = rec.est_xy  # SLAM-estimated position, meters
            beacon = make_beacon(
                beacon_id=self._seen + 1,
                writer_id=self.writer_id,
                event_type=event_type,
                x_cm=x_m * 100,
                y_cm=y_m * 100,
                heading=0,          # not tracked by nav.events yet - fine for Phase 1
                sensor_value=0,     # placeholder until real MQ-2/tilt/PIR values exist
                timestamp_ms=int(rec.t * 1000),
            )
            self.outside_network.receive_beacon(beacon)
        self._seen = len(beacons)
