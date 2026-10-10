import { Robot, defaultChassis } from "../src/robot.js";
import { BallPursuit, defaultParams } from "../src/pursuit.js";

const DT = 0.02;

export default function (t) {
  // --- basic motion ---------------------------------------------------
  {
    const r = new Robot();
    for (let i = 0; i < 50; i++) r.step({ forward: 0.5, strafe: 0, turn: 0 }, DT);
    t.ok("driving forward increases x", r.pose.x > 20);
    t.near("and does not drift sideways", r.pose.y, 0, 1e-9);
  }
  {
    const r = new Robot();
    for (let i = 0; i < 50; i++) r.step({ forward: 0, strafe: 0.5, turn: 0 }, DT);
    t.ok("strafing left increases y", r.pose.y > 15);
  }
  {
    const r = new Robot();
    for (let i = 0; i < 50; i++) r.step({ forward: 0, strafe: 0, turn: 0.5 }, DT);
    t.ok("positive turn is counter-clockwise", r.pose.headingRad > 0);
  }

  // --- static friction, the thing this model exists for ---------------
  {
    const r = new Robot();
    for (let i = 0; i < 200; i++) r.step({ forward: 0.05, strafe: 0, turn: 0 }, DT);
    t.near("power below breakaway moves nothing, ever", r.pose.x, 0, 1e-9);
  }

  // --- camera frame ----------------------------------------------------
  {
    const r = new Robot(defaultChassis(), { x: 0, y: 0, headingRad: 0 });
    const c = r.worldToCamera(30, 0);
    t.near("ball dead ahead: forward is distance minus camera offset",
           c.forwardIn, 30 - defaultChassis().cameraForwardIn, 1e-9);
    t.near("and lateral is zero", c.lateralIn, 0, 1e-9);
    const left = r.worldToCamera(30, 10);
    t.ok("a ball to the robot's left reads positive lateral", left.lateralIn > 0);
  }

  // --- the regression that motivated the breakaway minimums -----------
  // With kDriveMin at zero the proportional term dies under friction and the
  // robot halts short. This is the bug, reproduced.
  {
    const broken = defaultParams();
    broken.kDriveMin = 0; broken.kStrafeMin = 0; broken.kTurnMin = 0;
    broken.blindApproachIn = 0;      // disable commit, so only the P term drives
    const p = new BallPursuit(broken);
    const r = new Robot();
    let range = 24;
    for (let i = 0; i < 1500 && range > 1; i++) {
      const o = p.update({ rangeIn: range, txDeg: 0, lateralIn: 0 }, DT);
      r.step(o, DT);
      range = 24 - r.pose.x;
    }
    t.ok(`without breakaway minimums the robot stalls short (stopped at ${range.toFixed(1)} in)`,
         range > 3);
  }
  {
    const p = new BallPursuit(defaultParams());
    const r = new Robot();
    let range = 24, captured = false;
    for (let i = 0; i < 1500 && !captured; i++) {
      const target = range > defaultParams().cameraBlindIn
        ? { rangeIn: range, txDeg: 0, lateralIn: 0 } : null;
      const o = p.update(target, DT);
      r.step(o, DT);
      range = 24 - r.pose.x;
      captured = o.captured;
    }
    t.ok("with them, it commits and reaches the ball", captured);
    t.ok(`and ends on top of it (final range ${range.toFixed(1)} in)`, range < 4);
  }
}
