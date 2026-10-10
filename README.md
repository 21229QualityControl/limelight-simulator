# Limelight Simulator

**https://21229qualitycontrol.github.io/limelight-simulator/**

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

**Easiest: use the link above.** Nothing to install.

Offline, or to hand someone a copy: **open `dist/limelight-simulator.html`.** One file, no server, no
install, works offline. Double-click it, or email it to someone.

That file exists because browsers refuse to load ES modules over `file://`,
so `index.html` alone needs a web server — a silly thing to ask of someone who
just wants to look at a simulator. If you are editing the source, use the
modules and a server:

    python3 -m http.server 8000     # then http://localhost:8000

and rebuild the single file with `node build.js`.

## Using it with your own robot

Nothing here is hardcoded to one team any more. The two settings that change
every distance are at the top of the side panel:

- **Camera.** Lens calibration is per-unit. Using someone else's silently
  biases every range. Read yours off the camera and add a preset in
  `src/geometry.js`:

      curl -s http://172.29.0.1:5807/hwreport | python3 -m json.tool

  Copy `FOCAL_LEN_X_PIXELS`, `FOCAL_LEN_Y_PIXELS`, `PRINCIPAL_X_PIXELS`,
  `PRINCIPAL_Y_PIXELS`, `RES_X`, `RES_Y`.

- **Ball.** A 2.91 in pickleball and a 5.0 in DECODE artifact are not
  interchangeable; the diameter scales every distance.

Then set the **mounting** sliders to your lens height and tilt, and the
**pursuit gains** to whatever your code uses. The panel tells you the nearest
ball your camera can physically see and warns when your commit distance does
not clear it.

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

## Camera realism

`src/realism.js` applies exposure, gain, sensor noise and motion blur to the
rendered frame before the detector sees it, in the order a camera does:
the image smears during the exposure, then exposure and gain scale it, then
noise is added on readout.

The ball colours are not arbitrary. Real pollen measured 99 yellow, 61 red,
53 blue on a correctly exposed frame, and those ratios are reproduced,
because blue being the dimmest is the whole reason the exposure bug
happened. Drag brightness down and blue drops out first, then red, then
yellow — the simulator loses them at 0.45, 0.40 and 0.25.

### Three things it has already told us

- **Motion blur is not the problem it was assumed to be.** A ball at 60 in
  survives up to about 4 rad/s and is only lost past 6. A search sweep is
  roughly 1 rad/s, so at realistic FTC turn rates blur does not break
  detection. A ball at 30 in survives anything.
- **Tape running directly away from the camera is ball-width up close**, and
  generates phantom balls along its length. This had not been considered.
- **The ring test is a trade.** Raising `ringFreeMin` to 0.80 rejects that
  tape completely and still splits touching balls, but loses a close ball
  under heavy blur. The default stays permissive, because the blur tolerance
  is proven and the tape case has only been seen in simulation. The slider is
  in the UI; try "Ball on tape" at 0.45 and again at 0.80.

## What it still cannot tell you

JPEG compression, real gym lighting, a camera that browns out on USB power,
an uncalibrated tilt angle poisoning every range. Robustness has to be proven
on the robot.

Port targets live in `21229QualityControl/BioBuzzV1.1`, branch `ball-tracking`
(`TeamCode/.../subsystems/LimelightVision.java`, `commands/BallPursuit.java`),
and the SnapScript is in the `ftc-ball-tracking` working folder.
