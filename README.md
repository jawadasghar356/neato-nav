# neato-nav

LiDAR localisation for a low-cost mobile robot, built from scratch in C++17 on a second-hand
Neato Botvac D3. The long-term goal is an integrity monitor: the robot should not only estimate
where it is, but also detect when that estimate can no longer be trusted (kidnapping, changed
surroundings, degenerate geometry) and react safely.

No ROS and no third-party libraries for now. The robot is driven through its serial test
interface; a simulator of my apartment is used as a test bench where ground truth is known.

## Hardware

| Item | Notes |
|---|---|
| Neato Botvac D3 Connected | Firmware 4.5.3, LDS firmware 2.7.4 |
| LDS (rotating 2D LiDAR) | 360 readings per turn, 5.02 Hz measured |
| Wheel encoders | Position in mm per wheel via `GetMotors` |
| Battery | 4S Li-ion pack (bq40z50 BMS), recovered from deep discharge |
| Link | USB CDC serial today, ESP32-C3 WiFi bridge planned |

## Repository layout

```
include/neato/   serial port, Neato text protocol, IRobot interface, real robot backend
include/core/    geometry and differential-drive odometry
include/sim/     map loader, ray casting, simulated robot (encoders, bumpers, LDS model)
include/datalog/ drive recorder (JSON Lines)
apps/            teleop, battery_monitor, apartment_sim
tests/           parser, odometry and recorder tests, partly with output recorded from the robot
tools/           Python scripts for looking at recordings
maps/            apartment.map, plain text, editable without recompiling
viewer/          browser playback of simulation runs
data/            recordings from the real robot
docs/            design decisions and calibration log
```

All algorithms talk to `neato::IRobot`. `RealRobot` implements it over the serial port,
`SimRobot` in simulation, so the same navigation code runs on both.

## Build

Developed on Ubuntu 24.04 (WSL2). Any C++17 compiler and CMake 3.16+ should work, Windows/MSVC
included.

```
cmake -S . -B build
cmake --build build -j
ctest --test-dir build
```

GitHub Actions builds the project and runs the tests on every push.

## Running

Real robot (robot on the dock, USB connected):

```
./build/battery_monitor /dev/ttyACM0 60 battery_log.csv     # Linux / WSL
battery_monitor.exe COM9 60 battery_log.csv                 # Windows
```

Keyboard driving with recording (w/a/s/d, space to stop, +/- speed, q to quit):

```
./build/teleop                 # real robot on /dev/ttyACM0, lift it off the dock first
./build/teleop --sim           # simulated robot in the apartment
python3 tools/plot_drive.py logs/drive_<date>_<time>.jsonl
```

Each key press drives a 100 mm segment or a 15° turn; holding the key keeps the robot moving,
releasing it stops the robot within one segment. A bumper hit stops and backs off. Every run
is recorded to `logs/` (format: [ADR 0002](docs/decisions/0002-drive-log-format.md)).

Simulation:

```
./build/apartment_sim maps/apartment.map sim_log.json 2
```

The simulated robot drives a waypoint route to the entrance door and back to its dock using
wheel odometry only. Open `viewer/index.html` and load `sim_log.json` to replay the run.

The simulator models a D-shaped footprint with bumpers, encoder scale error, noise and
quantisation, wheel slip against obstacles, an LDS mounted 9 cm behind the axle that measures
while rotating, distance-dependent range noise, weak returns from dark surfaces and grazing
angles, and specular reflections from glass that produce ghost points.

## Results so far

- Odometry only, simulation: 0.36 m position error and 10.6° heading error after 22 m,
  docking failed in both runs. This is the baseline LiDAR correction has to beat.
- First real scan: 213 of 360 beams valid, 26 flagged weak, 113 without return. The LDS marks
  weak readings by setting bit 14 of the distance (e.g. 16954 = 570 mm, weak). The parser
  originally read these as 17 m; fixed and covered by a test using the recorded lines.
- Encoders: 1003 / 1004 mm reported for a 1000 mm command. Tape measurement pending.
- First teleop drive through the apartment: 20 m in 7 minutes, 1903 scans recorded. Projected
  with odometry only, walls already appear doubled after a few turns, mostly from heading
  drift. The recording also showed that the LDS counts angles counter-clockwise and that the
  zero-distance stop command is ignored by the firmware (see `docs/calibration.md`).

## Roadmap

- [x] Serial driver, protocol parsers, unit tests
- [x] Hardware abstraction with real and simulated backend
- [x] Apartment simulator and replay viewer
- [x] Battery recovery, first LDS scan and wheel test on the real robot
- [x] Teleoperation and drive recorder, tested in simulation
- [ ] First recorded drive through the apartment on the real robot
- [ ] Calibration: encoder scale, wheelbase (UMBmark), LDS angle direction
- [ ] Scan matching and occupancy grid mapping of the apartment
- [ ] Particle filter localisation
- [ ] Integrity monitor with protection level and fault injection experiments
- [ ] Web dashboard, ESP32 WiFi bridge, ROS 2 wrapper

## AI usage

Parts of the code were first drafted with Claude (Anthropic, Opus 5.5). I reviewed, tested and
extended them on the robot.

## License

MIT
