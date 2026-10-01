# Technical Solution & Architecture

## System overview

The Living Map system addresses GPS-denied, disconnected disaster environments with a
two-robot architecture: a **Writer robot** explores autonomously and deposits radio
beacons at points of interest, and a separate **Executor robot** later retrieves that
information via an **Outside Network Area** and continues the mission using inherited
beacon data — without requiring a direct link to the command post at any point.

Our chosen environment is **Mine/Tunnel Rescue**, modeled on a Gafsa-style room-and-pillar
mine layout: a main gallery with branching side tunnels, a dead-end chamber, and a
rock-fall-blocked branch used as the structural collapse scenario. We detect three event
types: toxic gas, structural collapse, and trapped workers.

*(Figure 1 - system_architecture.png)*

## Writer robot autonomy

The Writer robot combines a simulated 2D LiDAR (360 rays, 8 m range, PyBullet ray casting)
and a noisy IMU/odometry model with **BreezySLAM** (RMHC scan matching) to build an
occupancy map and estimate its own pose without GPS. Exploration uses a right-hand-rule
wall-following controller with four states (FOLLOW, TURN_LEFT, SEEK, RECOVER), using both
the LiDAR scan and the SLAM occupancy map for stuck detection and obstacle look-ahead.

*(Figure 2 - writer_robot_architecture.png)*

A known SLAM failure case was identified and mitigated during development: stock scan
matching drifts in heading along long, feature-poor corridors (measured at roughly
-0.2 deg/m). We address this with a damped odometry/scan-match fusion and an additional
wall-direction ("Manhattan") heading reference, which is effective in the orthogonal
room-and-pillar layout but assumes that structure — an explicit, documented limitation
for irregular tunnel geometries.

## Event detection & beacon deposition

Event detection is triggered when the robot's true position enters one of three predefined
hazard zones (gas, structural collapse, trapped worker). Each detection is stamped with
both the SLAM-estimated position (what goes into the beacon) and the ground-truth position
(kept for evaluation only) — the difference between the two is the localization error that
propagates into the beacon, an explicit design choice that lets us quantify and report
positioning accuracy.

## Beacon message & signal design

Beacons are a compact 16-byte message (beacon ID, writer ID, event type, priority,
local x/y coordinates, heading, raw sensor value, timestamp, TTL), designed to fit well
within the 32-byte payload limit of a low-cost nRF24L01 radio module. Priority is derived
from event type (trapped worker and structural collapse = high priority, gas = medium,
routine markers = low) and drives an **aging mechanism**: higher-priority beacons start
with a longer TTL and rebroadcast more frequently, so urgent findings persist and are
retrieved preferentially over routine ones.

## Frame translation

The Outside Network node converts each beacon's local SLAM-frame coordinates into real
GPS using a one-time manual calibration performed at the tunnel entrance (GPS position +
compass bearing of the robot's initial heading), combined with a flat-earth approximation
appropriate at tunnel/mine scale. Manual calibration was chosen deliberately over an
onboard magnetometer, since magnetic compass sensors are unreliable near the metal
structures and mineral content typical of a mine environment.

## Outside network area

All communication between the robots and the command post passes through the Outside
Network node — no direct robot-to-command-post link exists, satisfying the challenge's
architectural constraint. The node receives beacon broadcasts, performs frame translation,
and forwards the result to the command post, which aggregates incoming data, renders a
live map, and builds the Executor's mission as a priority-sorted list of unvisited beacon
targets.

## Executor robot

The Executor consumes the command post's prioritized target list and is designed to
navigate to each target in order, reusing the Writer's navigation stack. **Current status:
the Executor's own navigation loop (driving toward a specific inherited GPS target, as
opposed to free exploration) is not yet implemented** — this is the primary remaining gap
between the current codebase and the full system described here, and is the next
implementation priority heading into Phase 2.

## Results (simulation, default seed)

Full Writer mission: ~448 s simulated, ~146 m driven, returned to start, zero wall
collisions, all 3 event zones reached. Mean SLAM position error vs. ground truth: 0.14 m
(max 0.28 m). Odometry alone, without SLAM correction, drifts 4-8 m off over the same run —
demonstrating the practical necessity of the SLAM approach for this environment.
