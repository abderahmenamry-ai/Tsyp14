# Living Map: Spatial Memory for Emergency Robots

IEEE TSYP14 technical challenge (IEEE RAS x IEEE AESS, Tunisia Section Chapters) - **Phase 1 submission**.

A two-robot system that gives a GPS-denied disaster space its own memory. A **Writer** robot explores autonomously and records what it finds as compact radio beacons. An **Outside Network** node translates the beacons into real GPS coordinates and relays them to a **Command Post**, which briefs an **Executor** robot. The Executor then continues the mission using only the inherited beacon information, without starting from zero.

**Environment:** Mines / Tunnels, modelled on a Gafsa-style room-and-pillar phosphate mine.
**Events detected:** toxic gas, structural collapse, trapped worker.

```
Writer explores -> events detected, beacons written -> Outside Network (frame translation -> GPS)
   -> Command Post (live map, mission) -> Executor enters and navigates to each beacon
```

All robot <-> command-post traffic passes through the Outside Network node; there is no direct link.

## Repository layout

| Folder | What is in it |
|---|---|
| `aess_x_ras/living_map_nav/` | Simulation and navigation: PyBullet mine, simulated 2D LiDAR, BreezySLAM, Writer wall-following explorer, Executor goal-seeking controller (A*), results and plots |
| `aess_x_ras/comms/` | Beacon format and aging, Outside Network node (frame translation), Command Post and live map, Executor mission logic, integration demo |
| `aess_x_ras/firmware/` | **Robot firmware (ESP32, C++)** for the Writer, Executor and outside-network node: drive control, odometry, LiDAR parsing, event detectors, safety layer, nRF24 reliability, plus unit tests and a software-in-the-loop simulation |
| `aess_x_ras/base_station/` | SLAM host (Python): speaks the firmware's host link and connects it to BreezySLAM, the controllers and the command post |
| `aess_x_ras/architecture/` | Report sections and diagrams: architecture, **robot hardware / firmware / communication**, sensor research, implementation plan, failure cases |

## Quick start

```bash
cd aess_x_ras/living_map_nav
pip install -r requirements.txt
# BreezySLAM is not on PyPI (needs a C compiler):
git clone https://github.com/simondlevy/BreezySLAM.git && cd BreezySLAM/python && pip install . && cd ../..

python run_demo.py                      # Writer only: writes results/ (GIF, map, plots)

cd ../comms
python test_pipeline.py                 # fast checks of the beacon/comms pipeline (no PyBullet)
python run_integration_demo_v2.py       # Writer + Executor end to end -> comms/results/

# robot firmware: unit tests, behaviour tests, ESP32 syntax check, full mission with the firmware in the loop
cd ../firmware
make test && make check-esp32 && make sil
```

## Results (simulation, default seed)

| | |
|---|---|
| Writer mission | about 448 s simulated, about 146 m driven, returned to start, 0 wall contacts, all 3 event zones reached |
| Writer position error vs ground truth | mean 0.14 m, max 0.28 m (odometry alone ends 4-8 m off) |
| Beacon position error | 0.08 m (worker), 0.13 m (collapse), 0.23 m (gas) |
| Executor mission | visits 3 of 3 beacons in priority order (136 s simulated, 63 m driven, max position error 0.28 m, 0 wall contacts); also 3 of 3 on seeds 1-3 |
| Full system with the real firmware in the loop | Writer: mean error 0.25 m, max 0.45 m, 0 wall contacts; 3 beacons acknowledged and delivered; Executor visits 3 of 3 (144 s, max error 0.26 m). Same result with 50 % radio loss |

Demo recordings: `aess_x_ras/living_map_nav/results/demo.gif` (Writer) and `aess_x_ras/comms/results/integrated_demo.gif` (Writer, command post and Executor together).

## Phase 1 deliverables

- **Simulation demo:** `living_map_nav/` and `comms/run_integration_demo_v2.py`
- **Technical solution and architecture:** `aess_x_ras/architecture/architecture_section.md` and the diagrams in the same folder
- **Sensor research:** `aess_x_ras/architecture/sensor_research.pdf` (MQ-2 gas, SW-520D tilt, HC-SR501 PIR)
- **Robot hardware, firmware and communication:** `aess_x_ras/architecture/robot_architecture.pdf` (parts, wiring, protocols, state machine, safety layer) and the code in `aess_x_ras/firmware/` and `aess_x_ras/base_station/`
- **Implementation plan (Phase 2 roadmap):** `aess_x_ras/architecture/implementation_plan.pdf`
- **Failure cases:** `aess_x_ras/architecture/failure_cases.pdf`
- **Short technical report (max 6 pages):** compiled from the sections above

## Known limitations

Documented in `aess_x_ras/architecture/failure_cases.pdf`: SLAM heading drift in featureless corridors, sensor false positives and negatives, radio range underground, frame-translation assumptions, and the fact that everything so far is simulation only.

## Team

*(Add team members and IEEE AESS / RAS / YP membership details here.)*
