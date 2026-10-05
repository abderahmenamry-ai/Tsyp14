# Robot firmware (ESP32)

Firmware for the three ESP32 nodes: **Writer**, **Executor** and the **outside network node (hub)**. One C++ code base, one image per role.
Design and rationale: `../architecture/robot_architecture.md`.

## Layout

| Path | What it is |
|---|---|
| `include/protocol.h` | Host-link messages and radio packets (single source of truth, mirrored by `../base_station/link.py`) |
| `include/hal.h` | Hardware abstraction: everything the logic needs from the outside world |
| `include/params.h` | Robot geometry, PID gains, safety limits, sensor thresholds |
| `include/*.h`, `src/apps.cpp` | Portable core: framing, LD06 parser, odometry, wheel PID, event detectors, beacon retry/aging, Writer / Executor / Hub applications |
| `include/pins.h`, `src/hal_esp32.cpp`, `src/main.cpp` | ESP32 hardware layer and entry point (Arduino-ESP32 core 2.x) |
| `platformio.ini` | Build environments `writer`, `executor`, `hub` |
| `host/` | PC build of the same core (simulated HAL + C interface for Python) |
| `sil/` | Test benches and the software-in-the-loop simulation (Python) |
| `test/test_core.cpp` | Unit tests of the portable core |
| `../comms/firmware/` | Beacon format and GPS translation shared with the Python side |

## Test it on a PC (no hardware needed)

Needs `g++`, `make`, Python 3 and the simulation dependencies from `../living_map_nav/requirements.txt` (including BreezySLAM). From this folder:

    make test          # 57 C++ unit checks + 62 behaviour checks on a simulated bench (~20 s)
    make check-esp32   # syntax-check the ESP32 glue for all three roles against stub headers
    make sil           # full mission through the real firmware in the PyBullet mine, perfect radio and 50 % radio loss (~6 min)

Single runs: `cd .. && python3 firmware/sil/run_sil.py [--loss 0.5] [--check]`.

## Build for the ESP32

    pip install platformio
    pio run -e writer              # also: -e executor, -e hub
    pio run -e writer -t upload

**These environments have not been built by us yet** (no PlatformIO toolchain where this was written). Expect to fix small compile issues on the first build, most likely around the RF24 library version or the Arduino-ESP32 core version (the code uses the 2.x `ledcSetup` / `ledcAttachPin` API).

## Bring-up checklist (first time on real hardware)

Do these in order; each step depends on the one before.

1. **Radio only (hub + one robot):** both boards boot, the hub LED does not blink fast (fast blinking means the nRF24 does not answer on SPI: check CE/CSN/3V3 and the 10 uF capacitor). Run the host bridge and watch for heartbeats.
2. **Motors off the ground:** flip `MOTOR_L_INVERT` / `MOTOR_R_INVERT` in `pins.h` until a positive command turns both wheels forward. Then flip `ENC_L_SIGN` / `ENC_R_SIGN` until encoder counts increase going forward.
3. **Calibrate geometry:** measure wheel diameter, wheel base and encoder ticks per wheel revolution; put them in `params.h`. Drive 2 m straight and spin 360 degrees and compare with the odometry in the `SCAN` messages.
4. **Tune the wheel PID** (`kp`, `ki`, `max_accel_ms2`) against a step in speed. Set `min_duty` just above the duty where the wheels start to turn.
5. **LiDAR:** check that packets pass the CRC-8 (the LD06 CRC polynomial in `crc.h` is from the vendor description and unverified), then set `lidar_sign` and `lidar_offset_deg` so that an object on the robot's left appears on the left of the map.
6. **Event sensors:** check the tilt-switch polarity (`TILT_TRIPPED_LEVEL`), baseline of the MQ-2 in clean air, and PIR warm-up. Adjust `gas_rise_counts` to the real sensor.
7. **Safety layer:** unplug the LiDAR (robot must stop), stop the host (robot must stop after 0.5 s), push the emergency-stop command.
8. **Radio range:** walk the Writer away from the hub in a corridor and record where beacons stop getting acknowledged (the Phase 2 radio test).

## Known gaps

- Battery voltage is not measured (all ADC1 pins are used); telemetry reports 0.
- The LD06 motor-speed PWM pin is not driven; check the vendor datasheet for the default behaviour of your unit.
- Beacon aging needs a shared mission clock across nodes; there is no clock-sync message yet.
- The SLAM host is a tethered laptop in this design. A Raspberry Pi on the robot runs the same Python unchanged but is not part of the bill of materials yet.
