# 0002 – Drive recordings as JSON Lines

Date: 2026-10-15
Status: accepted

## Context

From now on every drive of the robot should be recorded: LDS scans, wheel encoder readings,
the motor commands that were sent and events such as bumper hits. These recordings feed the
next steps (encoder and wheelbase calibration, scan matching, mapping) and let me rerun an
algorithm on the same data as often as I want instead of driving the robot again.

Requirements:

- written while driving, and nothing lost if the program crashes or I press Ctrl+C
- easy to read from Python for plotting and quick analysis
- no external library in the C++ code
- size is not critical: one scan is about 4 KB, so 10 minutes at 5 Hz is roughly 12 MB

## Options

| Option | For | Against |
|---|---|---|
| One JSON document (like `sim_log.json`) | One `json.load` | Invalid until the closing bracket is written, so a crash loses everything |
| CSV per stream | Simple | Scans do not fit a row well, several files to keep in sync |
| JSON Lines, one object per line | Appendable, every line valid on its own, `json.loads` per line | Larger than binary |
| MCAP / rosbag | Tooling (Foxglove), industry standard | Needs a library and schema work now, before I know what I need |

## Decision

JSON Lines. Each line has a `type` (`header`, `scan`, `odom`, `cmd`, `event`) and a time `t`
in seconds from a monotonic clock started with the recording. The first line is a header with
the data source, start time in UTC, wheelbase, LDS mounting offset and the assumed scan angle
convention.

Scans are stored as three arrays of 360 integers indexed by angle in degrees: range in mm,
intensity, and a flag (0 valid, 1 weak, 2 no return). Indexing by angle means a line missing
from the robot's answer cannot shift the data. Weak readings keep their range because they are
often still usable, and whether to use them should be decided by the algorithm, not the
recorder.

The writer flushes after every line.

## Consequences

- Recordings are self-describing and can be read with a few lines of Python
  (`tools/plot_drive.py`).
- Timestamps are taken when the answer arrives on the laptop, not when the LDS measured. The
  serial delay (tens of ms) is therefore part of the data and has to be considered in scan
  matching.
- If the angle direction of the LDS turns out to be clockwise, old recordings stay valid: the
  header states the convention they were recorded with.
- An MCAP exporter can be added later for Foxglove without changing the recorder.
