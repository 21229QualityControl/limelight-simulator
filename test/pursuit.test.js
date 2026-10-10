// Mirrors the simulations run against the Java before it shipped.
import { BallPursuit, defaultParams } from "../src/pursuit.js";

const at = (rangeIn, txDeg = 0, lateralIn = 0) => ({ rangeIn, txDeg, lateralIn });
const DT = 0.02;

/** Run to steady state, past the slew ramp. */
function settle(p, target, ticks = 80) {
  let o;
  for (let i = 0; i < ticks; i++) o = p.update(target, DT);
  return o;
}

export default function (t) {
  // --- breakaway: small errors must still move the robot -------------
  {
    const p = new BallPursuit();
    for (const r of [30, 20, 16, 15]) {
      p.reset();
      const o = settle(p, at(r));
      t.ok(`forward at ${r} in does not stall (${o.forward.toFixed(2)})`, o.forward >= 0.10);
    }
  }

  // --- deadbands: no hunting at tiny errors ---------------------------
  {
    const p = new BallPursuit();
    p.reset();
    t.eq("turn is zero inside the deadband", settle(p, at(30, 1.4)).turn, 0);
    p.reset();
    t.ok("turn is live outside it", Math.abs(settle(p, at(30, 2.0)).turn) >= 0.08);
    p.reset();
    t.eq("strafe is zero inside the deadband", settle(p, at(30, 0, 0.8)).strafe, 0);
    p.reset();
    t.ok("strafe is live outside it", Math.abs(settle(p, at(30, 0, 1.2)).strafe) >= 0.12);
  }

  // --- the worst case that used to stall: close AND badly misaimed ----
  {
    const p = new BallPursuit();
    const o = settle(p, at(15, 20, 4));
    t.ok(`close and 20 deg off still moves (${o.forward.toFixed(3)})`, o.forward >= 0.10);
  }

  // --- turn sign: a ball to the RIGHT must turn clockwise -------------
  {
    const p = new BallPursuit();
    p.reset();
    t.ok("ball right of centre turns negative (clockwise)", settle(p, at(30, 10)).turn < 0);
    p.reset();
    t.ok("ball left of centre turns positive", settle(p, at(30, -10)).turn > 0);
  }

  // --- commit and run: the whole reason this controller exists --------
  {
    const p = new BallPursuit();
    let o;
    for (const r of [48, 40, 32, 26, 20, 16, 14.5]) o = p.update(at(r, 2), DT);
    t.ok("not committed above the commit range", !p.isCommitted());

    o = p.update(at(13.5, 2), DT);
    t.ok("commits inside blindApproachIn", p.isCommitted());
    t.near("remembers where it committed", p.rangeAtCommit, 13.5, 0.01);

    // From here the camera reports nothing, which is expected.
    let elapsed = 0, captured = false;
    for (let i = 0; i < 200 && !captured; i++) {
      o = p.update(null, DT);
      elapsed += DT;
      captured = o.captured;
    }
    t.ok("keeps driving through the blind zone", elapsed > 0.5);
    t.ok("declares capture at the end of the run", captured);
    t.near("after about blindDriveSeconds", elapsed, defaultParams().blindDriveSeconds, 0.08);
  }

  // --- a null target before committing must NOT fake a capture --------
  {
    const p = new BallPursuit();
    let any = false;
    for (let i = 0; i < 100; i++) if (p.update(null, DT).captured) any = true;
    t.ok("no target, no capture", !any);
  }

  // --- commit must be above the blind radius, or it never sees close --
  {
    const d = defaultParams();
    t.ok("blindApproachIn is clear of the blind radius",
         d.blindApproachIn > d.cameraBlindIn + 2.0);
  }
}
