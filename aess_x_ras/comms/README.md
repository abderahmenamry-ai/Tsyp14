# Living Map - Task 2: Sensing & Communication (comms)

Beacon design, outside network, command post, and Executor mission logic
for the "Living Map" TSYP14 project. Pairs with `living_map_nav` (Task 1 -
Writer robot navigation/SLAM).

## Files

| File | Role |
|---|---|
| `beacon.py` | Beacon message format, priority rules and TTL aging (mirrors `firmware/beacon_protocol.h`) |
| `outside_network.py` | Receives beacons, translates local coords to GPS (mirrors `firmware/frame_translation.h`) |
| `command_post.py` | Aggregates beacon data, live map, builds the Executor's mission |
| `executor_mission.py` | Works through the prioritized target list |
| `beacon_adapter.py` | **The integration point with Task 1** - converts `nav.events.beacons` (BeaconRecord) into our BeaconMessage format |
| `run_integration_demo_v2.py` | Runs the real Writer, then the real Executor, through this whole pipeline; writes the integrated GIF and live-map PNGs to `results/` |
| `live_map.py` | Graphical command-post live map + the recorder behind the integrated GIF |
| `test_pipeline.py` | Fast standalone test of just this pipeline (fake straight-line path, no PyBullet/SLAM needed) |
| `firmware/beacon_protocol.h` | C header for the real ESP32 firmware (Phase 2) |
| `firmware/frame_translation.h` | C header for the real outside-network node firmware (Phase 2) |

The ESP32 firmware that sends and receives these beacons lives in `../firmware/` (it includes
`firmware/beacon_protocol.h` and `frame_translation.h` from this folder), and the Python side that
connects it to the command post is `../base_station/`.

## Run

Quick sanity check (no dependencies beyond this package). It now asserts that all
three beacons arrive, the mission order is correct, the GPS conversion lands in the
right place, and aging expires beacons on schedule:

    python test_pipeline.py

Full integration with the real Writer and Executor simulation (needs
`living_map_nav`'s dependencies installed - see its own README). Run it from this folder:

    python run_integration_demo_v2.py            # writes results/integrated_demo.gif + live-map PNGs
    python run_integration_demo_v2.py --no-gif   # console output only

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

## Aging

TTL is a beacon's lifetime in aging ticks (`AGING_TICK_MS` = 30 s): low 150 s, medium
360 s, high 600 s. `CommandPost.now_ms` is the shared mission clock; the demo advances it,
and `None` disables aging. Expired beacons show as `expired` on the live map and are left
out of the Executor's mission.

## Status

All comms modules are complete and tested (`test_pipeline.py`). The integration demo drives
the real Writer and Executor end to end: the Executor visits all three beacons in simulation.
