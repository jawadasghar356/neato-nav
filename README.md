# neato_nav – C++ navigation stack for a Neato Botvac D3

Personal portfolio project of Jawad Asghar (M.Eng. Mechatronic and Cyber-Physical Systems, DIT).
Goal: LiDAR-based localisation with an *integrity monitor* that detects when the robot's
position estimate can no longer be trusted. Written in plain C++17, no ROS, no external libraries.

## Idea: one interface, two robots

All algorithms talk to `neato::IRobot`. There are two implementations:

| Backend | File | What it is |
|---|---|---|
| `RealRobot` | `src/real_robot.cpp` | The physical Neato over USB (COM9). Later also via the ESP32. |
| `SimRobot` | `src/sim_robot.cpp` | A simulated Neato driving in a map of the real apartment (see below). |

So the navigation code can be developed in simulation now and run on the real robot later
without changes.

## Project layout

```
include/core/geometry.hpp     2D maths: poses, angles, ray-segment intersection
include/core/odometry.hpp     wheel odometry (differential drive)
include/neato/                robot interface, serial port, Neato text protocol
include/sim/world.hpp         map loader, ray casting, collision test
include/sim/sim_robot.hpp     simulated Neato (D-shaped body, bumpers, encoders, LDS)
maps/apartment.map            the apartment as a plain text file (edit with any editor)
apps/apartment_sim.cpp        SIMULATION: odometry-only missions in the apartment
apps/battery_monitor.cpp      REAL robot: logs battery voltage and charging status
viewer/index.html             animated playback of a simulation run (open in a browser)
tests/test_parsers.cpp        unit tests (uses real output recorded from the D3)
```

## Build on Windows

1. Install **Visual Studio 2022 Community** with the workload **"Desktop development with C++"**
   (this includes CMake).
2. In Visual Studio: *File → Open → Folder…* and select this folder. It detects `CMakeLists.txt`.
3. Choose the configuration *x64-Release* at the top, then *Build → Build All*.
4. The programs are in `out\build\x64-Release\`.

Command-line alternative (in the "Developer PowerShell for VS 2022"):

```
cmake -S . -B build
cmake --build build --config Release
build\Release\test_parsers.exe
```

On Linux: `cmake -S . -B build && cmake --build build -j`.

## Run

**Simulation (works anywhere):**
```
apartment_sim                                   # maps/apartment.map, sim_log.json, 2 missions
apartment_sim maps/apartment.map run.json 5     # 5 missions
```
The robot leaves its dock, drives the waypoint route to the entrance door and back, and tries
to reverse onto the dock, using only its wheel odometry. It prints whether each docking worked,
the distance driven, the final position and heading error and the bumper hits, and writes
`sim_log.json` (one frame per LiDAR turn: true pose, believed pose, 180 ranges, bumps, ghosts).

Then open `viewer/index.html` in a browser and click *Load sim_log.json* (or drag the file onto
the page). The viewer plays the run on the floor plan: true robot, believed robot (dashed), LiDAR
beams, ghost returns from glass in magenta, and the error curve over time.

What the simulator models: D-shaped footprint for collisions; front/side bumpers; encoder scale
mismatch (0.2 %), noise, 1 mm quantisation and wheel slip while pushing against an obstacle;
a 5 Hz rotating LDS mounted 9 cm behind the axle with 1° steps measured during the turn;
range noise that grows with distance squared (triangulation); weak returns from dark surfaces
and at grazing angles; mirror-like reflections from glass and gloss that create ghost points.

**Real robot (works now, on dock power):** close PuTTY first, then
```
battery_monitor COM9 60 battery_log.csv
```
Reads `GetCharger` every 60 s and logs it.

## The map file

`maps/apartment.map` is plain text and is read at start-up, so changing it needs no recompiling.
Units are metres and degrees. Origin is the back corner on the kitchen side, x runs towards the
entrance door, y from the kitchen wall to the wardrobe wall.

```
room      W H                          room size
material  name reflectivity mirror colour
wall      x1 y1 x2 y2 material
box       name x0 y0 x1 y1 material    axis-aligned obstacle
circle    name cx cy r material        round obstacle (stools, the Ecovacs)
start     x y heading                  pose on the dock
waypoint  x y                          mission route, in order
```

**All dimensions are estimates from photos.** Measure and correct: room length and width, the
back wall (not in the photos) relative to the dock, wardrobe depth and length, the table column
and stool positions, the kitchen run and oven position, dock and Ecovacs station positions.
Only things below about 9 cm height matter to the LiDAR (table tops and hanging coats are invisible).

## Status

- [x] Serial port (Windows + Linux), Neato protocol, parsers with unit tests
- [x] Hardware abstraction (`IRobot`) with real and simulated backend
- [x] Wheel odometry
- [x] Apartment simulator (map file, bumpers, realistic LDS incl. glass ghosts) and web viewer
- [x] Experiment 1: odometry-only missions (about 0.36 m error after 22 m, docking fails)
- [ ] LiDAR feature extraction: split-and-merge line fitting, corner detection
- [ ] Scan-to-map matching (ICP) to correct odometry drift
- [ ] EKF / particle filter localisation
- [ ] Localisation integrity monitor (main research contribution)
- [ ] Real robot: verify LDS angle direction and wheel distance calibration (needs new battery)

## Notes to verify on the real robot

- **LDS angle direction:** the code assumes 0° = forward, counter-clockwise. Place a box in front
  and on the left of the robot and check `GetLDSScan`.
- **Wheelbase:** 0.248 m is assumed. Calibrate by commanding a 360° turn and measuring.
- **TestMode** is needed for `SetMotor` and `SetLDSRotation`; with a deeply discharged battery
  the robot reboots when TestMode is switched on.

## AI usage

Initial code structure generated with Claude (Anthropic) and reviewed, tested and extended by me.
