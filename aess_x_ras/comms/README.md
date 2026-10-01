# Living Map - Task 2: Sensing & Communication (comms)

Beacon design, outside network, command post, and Executor mission logic
for the "Living Map" TSYP14 project. Pairs with `living_map_nav` (Task 1 -
Writer robot navigation/SLAM).

## Files

| File | Role |
|---|---|
| `beacon.py` | Beacon message format + priority/aging rules (mirrors `firmware/beacon_protocol.h`) |
| `outside_network.py` | Receives beacons, translates local coords to GPS (mirrors `firmware/frame_translation.h`) |
| `command_post.py` | Aggregates beacon data, live map, builds the Executor's mission |
| `executor_mission.py` | Works through the prioritized target list |
| `beacon_adapter.py` | **The integration point with Task 1** - converts `nav.events.beacons` (BeaconRecord) into our BeaconMessage format |
| `run_integration_demo.py` | Runs the real Writer nav stack + this whole pipeline together |
| `test_pipeline.py` | Fast standalone test of just this pipeline (fake straight-line path, no PyBullet/SLAM needed) |
| `firmware/beacon_protocol.h` | C header for the real ESP32 firmware (Phase 2) |
| `firmware/frame_translation.h` | C header for the real outside-network node firmware (Phase 2) |

## Run

Quick sanity check (no dependencies beyond this package):

    python test_pipeline.py

Full integration with the real Writer robot simulation (needs
`living_map_nav`'s dependencies installed - see its own README):

    python run_integration_demo.py

## Interface with Task 1

`beacon_adapter.py::BeaconBridge` is a callback passed to
`NavStack.run(callbacks=[bridge])`. Every tick, it checks
`nav.events.beacons` for new entries and forwards them through
`outside_network.py` -> `command_post.py`, converting:

- `BeaconRecord.kind` (string: "gas"/"collapse"/"worker") -> `EventType` enum
- `BeaconRecord.est_xy` (meters) -> `x_coord_cm`/`y_coord_cm` (centimeters)
- `BeaconRecord.t` (seconds) -> `timestamp_ms` (milliseconds)

`heading` and `sensor_value` aren't tracked by Task 1's event simulator yet,
so they're placeholdered at 0 - fine for Phase 1, worth revisiting once
real MQ-2/tilt/PIR sensor values exist.

## Status

All 5 comms modules are complete and tested against a fake path
(`test_pipeline.py`). Integration with the real nav stack
(`run_integration_demo.py`) is wired and ready to run. Not yet built:
an Executor-side navigation loop in Task 1 to actually drive to each
`ExecutorMission` target - currently `living_map_nav` only implements
the Writer's exploration.
