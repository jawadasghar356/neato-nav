# Calibration log (real Neato Botvac D3)

| Date | Test | Result | Notes |
|---|---|---|---|
| 2026-10-15 | LDS rotation speed | 5.02 Hz | `GetLDSScan` ROTATION_SPEED |
| 2026-10-15 | Wheels in the air, 2 x 100 mm command | 205 / 205 mm | Both sides equal; ~2.5 % coasting without load |
| 2026-10-15 | 1 m drive on floor, encoders | 1003 / 1004 mm | Tape-measured distance: **TODO** |
| 2026-10-15 | First scan | 213 valid, 26 weak, 113 no-return of 360 | Weak flag = 0x4000 in DistInMM |
| – | LDS angle direction (box on the left) | **TODO** | Expect change near 90° if counter-clockwise |
