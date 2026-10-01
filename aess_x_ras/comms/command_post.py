"""
command_post.py
Aggregates translated beacon data, keeps a priority-sorted view of
what's been found, renders a simple live map, and builds the
Executor's mission.
"""

class CommandPost:
    def __init__(self):
        self.received = []  # list of (beacon, gps) tuples
        self.visited_ids = set()

    def ingest(self, beacon, gps):
        """Called by the outside network node whenever a beacon arrives."""
        self.received.append((beacon, gps))
        print(f"[command post] beacon {beacon.beacon_id} "
              f"({beacon.event_type.name}, priority={beacon.priority.name}) "
              f"at GPS ({gps.lat:.6f}, {gps.lon:.6f})")

    def live_map_summary(self):
        """Minimal text 'live map' for the Phase 1 demo. Swap for a
        matplotlib/pygame view later - the data structure underneath
        doesn't need to change."""
        lines = ["=== LIVE MAP ==="]
        for beacon, gps in self.received:
            status = "visited" if beacon.beacon_id in self.visited_ids else "pending"
            lines.append(f"  [{status}] #{beacon.beacon_id} {beacon.event_type.name} "
                         f"@ ({gps.lat:.6f}, {gps.lon:.6f}) priority={beacon.priority.name}")
        return "\n".join(lines)

    def build_mission(self):
        """
        Returns pending targets sorted by priority (high first), then
        freshest timestamp within the same priority - the aging/
        prioritization rule from the beacon design in action.
        """
        pending = [(b, g) for b, g in self.received if b.beacon_id not in self.visited_ids]
        pending.sort(key=lambda pair: (-pair[0].priority, -pair[0].timestamp_ms))
        return pending

    def mark_visited(self, beacon_id):
        self.visited_ids.add(beacon_id)
