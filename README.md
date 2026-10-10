# Limelight Simulator

A browser simulator for the BIOBUZZ pollen-tracking vision stack, so the
detector and the pursuit controller can be tested and tuned without a robot.

It is not a mock. It renders a synthetic camera view with the Limelight's real
lens calibration, runs a faithful port of the on-camera SnapScript over those
pixels, and feeds the result into a port of the robot's `BallPursuit`. If a
gain works here it should transfer; if the detector drops a ball here it would
drop it on the field.

## What it simulates

| Piece | Mirrors |
|-------|---------|
| `src/geometry.js` | the `Geometry` class in `ball_snapscript.py` |
| `src/detector.js` | the rest of `ball_snapscript.py` |
| `src/pursuit.js`  | `BallPursuit.java` |
| `src/robot.js`    | mecanum kinematics, the drivetrain the commands assume |

## What it cannot tell you

Simulated pixels are clean. The real failures so far have been motion blur,
cardboard that reads as yellow, a ball touching same-coloured tape, and the
camera browning out on USB power. None of those appear here. This is for
geometry and control tuning, not for proving the detector is robust.

## Running

No build step. Open `index.html`, or:

    python3 -m http.server 8000

Then http://localhost:8000

## State

Built so far:

- `src/geometry.js` — pinhole + flat-floor solver, a port of the `Geometry`
  class in `ball_snapscript.py`, plus `project()` (the inverse, which the
  renderer needs) and `nearestVisibleIn()`.
- `test/` — a dependency-free runner, `npm test` or `node test/run.js`.

The geometry tests pin the port to measurements taken on the real robot: a
ball at a tape-measured 24 in and 48 in, and the blob radius observed at the
24 in row. They pass. If they ever stop passing, the simulator has stopped
predicting the robot and nothing tuned in it can be trusted.

One fact those tests recorded, which is easy to get wrong: at the calibrated
25.9 deg the horizon sits about 23 px ABOVE the top of the frame, so every
pixel hits the floor. Far-field rejection is done by `MAX_RANGE_IN`, not by a
horizon check.

## Still to build

1. `src/render.js` — draw the synthetic camera view: balls on a floor,
   projected with `Geometry.project()`.
2. `src/detector.js` — port the rest of `ball_snapscript.py`: HSV threshold,
   morphological close, distance-transform peak picking, the directional ring
   test, then the geometry solve. Must emit the same eight doubles the
   SnapScript writes to `llpython`.
3. `src/pursuit.js` — port `BallPursuit.java`, including the commit-and-run
   final approach and the breakaway minimums with deadbands.
4. `src/robot.js` — mecanum kinematics.
5. `index.html` + UI — camera view, mask view, top-down field, and sliders for
   the mounting numbers, thresholds and gains.

Port targets live in `21229QualityControl/BioBuzzV1.1`, branch `ball-tracking`
(`TeamCode/.../subsystems/LimelightVision.java`, `commands/BallPursuit.java`),
and the SnapScript is in the `ftc-ball-tracking` working folder.
