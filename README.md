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
