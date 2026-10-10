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

All five pieces are built and the loop runs end to end:

| Piece | Mirrors |
|-------|---------|
| `src/geometry.js`   | the `Geometry` class in `ball_snapscript.py` |
| `src/thresholds.js` | that script's HSV windows |
| `src/detector.js`   | the rest of `ball_snapscript.py` |
| `src/pursuit.js`    | `BallPursuit.java` |
| `src/robot.js`      | mecanum kinematics, with static friction |
| `src/render.js`     | the synthetic camera view |

`node test/run.js` — **71 tests, all passing.**

## What the tests are actually for

They check the ports against reality, not against themselves.

- The geometry is pinned to measurements taken on the robot: a ball at a
  tape-measured 24 in and 48 in, and the blob radius seen at the 24 in row.
- The pursuit numbers match the Java simulations to two decimals.
- The detector is run on frames with known ground truth, covering the cases
  that actually caused trouble: tape alone must yield nothing, a blue ball
  touching blue tape must still be found, two touching balls must stay two.
- The robot model reproduces the original stall bug. With `kDriveMin` at zero
  it halts 10.9 in short of the ball; with the breakaway minimums restored it
  commits and finishes 1.6 in away.

If the geometry tests ever fail, the simulator has stopped predicting the
robot and nothing tuned in it can be trusted.

## Two things it answered that the robot could not

The hardware has never had a working camera connection, so these had never
been tested anywhere until now:

- a **blue ball touching blue tape** is still detected, and its range survives
- **two touching balls stay two**, rather than merging into one blob with a
  meaningless centre

Both pass. That is evidence the algorithm is sound, not proof it survives a
real field.

## What it cannot tell you

Simulated pixels are clean. Every failure seen on real hardware so far —
motion blur, cardboard reading as yellow, the camera browning out on USB
power, an uncalibrated tilt angle poisoning every range — is invisible here.
This is for geometry and control tuning. Robustness has to be proven on the
robot.

Port targets live in `21229QualityControl/BioBuzzV1.1`, branch `ball-tracking`
(`TeamCode/.../subsystems/LimelightVision.java`, `commands/BallPursuit.java`),
and the SnapScript is in the `ftc-ball-tracking` working folder.
