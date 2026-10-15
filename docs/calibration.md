# Calibration log (real Neato Botvac D3)

| Date | Test | Result | Notes |
|---|---|---|---|
| 2026-10-15 | LDS rotation speed | 5.02 Hz | `GetLDSScan` ROTATION_SPEED |
| 2026-10-15 | Wheels in the air, 2 x 100 mm command | 205 / 205 mm | Both sides equal; ~2.5 % coasting without load |
| 2026-10-15 | 1 m drive on floor, encoders | 1003 / 1004 mm | Tape-measured distance: **TODO** |
| 2026-10-15 | First scan | 213 valid, 26 weak, 113 no-return of 360 | Weak flag = 0x4000 in DistInMM |
| 2026-10-15 | LDS angle direction | Counter-clockwise | From two on-the-spot turns in the first drive (-92.6° encoders vs +94° wall shift, -80.4° vs +76°), `tools/check_lds_direction.py`. Box test optional as a cross-check |
| 2026-10-15 | First teleop drive, 20 m in 417 s | Scans every 0.21 s, encoders every 0.06 s | 69 % of beams valid, 3 % weak, 28 % no return; 8 of 1903 scans repeated |
| 2026-10-15 | Stop with `SetMotor LWheelDist 0 RWheelDist 0` | **Ignored** | Robot finishes the running segment (35 + 68 mm of a 100 mm segment). A new non-zero SetMotor does replace the running one (bumper back-off works). Candidates tested with `stop_test`: **TODO** |
