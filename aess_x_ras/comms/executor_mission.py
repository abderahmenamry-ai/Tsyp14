"""
executor_mission.py
Executor robot's mission logic: takes the prioritized target list from
the command post and works through it in order. This file owns the
"what to do" decisions - the actual driving is Task 1's navigation
stack, which this hands each GPS/target to (once an Executor nav loop
exists - currently the repo only has the Writer's exploration).
"""

class ExecutorMission:
    def __init__(self, command_post):
        self.command_post = command_post
        self.targets = []
        self.current_target = None

    def receive_mission(self):
        """Pulls the current prioritized target list from the command post.
        Call this once at mission start, and again after each target is
        reached (in case new beacons arrived meanwhile)."""
        self.targets = self.command_post.build_mission()

    def next_target(self):
        """Returns the next (beacon, gps) target to navigate to, or None
        if the mission is complete. Hand the gps coordinate to the
        navigation stack to actually drive there."""
        if not self.targets:
            return None
        self.current_target = self.targets[0]
        return self.current_target

    def reached_target(self):
        """Call when the navigation stack reports arrival at the
        current target."""
        if self.current_target:
            beacon, _ = self.current_target
            self.command_post.mark_visited(beacon.beacon_id)
            self.current_target = None
            self.receive_mission()
