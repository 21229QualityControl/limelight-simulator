// Port of BallPursuit.java.
//
// One deliberate difference: update() takes dt explicitly instead of reading
// a clock. The robot uses System.nanoTime(); a simulator needs to step at a
// fixed rate and replay identically, so time is an argument here.
//
// The shape of this controller is driven by one hardware fact: the camera
// cannot see a ball closer than about 10 inches, because it drops below the
// bottom of the frame. So capture CANNOT be detected by a range threshold --
// there is no number the camera can report. Instead the controller commits
// and runs the last leg open loop.

export function defaultParams() {
  return {
    standoffIn: 8.0,

    // Nearest range the camera can report. Closer than this there are no
    // detections at all, only silence. At the calibrated mount (lens 11.7 in
    // up, 25.9 deg down) that is 9.9 in. Recompute if the camera is re-aimed:
    //   blind = (lensHeight - ballRadius) / tan(pitch + 20.17 deg)
    cameraBlindIn: 9.9,

    // Commit point. MUST be comfortably greater than cameraBlindIn, or the
    // ball disappears at the same instant this phase begins and the robot
    // never learns it got close.
    blindApproachIn: 14.0,

    // The final run, open loop.
    blindDrivePower: 0.22,
    blindDriveSeconds: 0.9,

    aimToleranceDeg: 6.0,

    kTurn: 0.022,
    kDrive: 0.030,
    kStrafe: 0.020,

    // Breakaway powers. A proportional term alone goes to zero as the error
    // does, and long before that it drops under the power needed to move the
    // robot -- so it stops short with a standing error and cannot tell.
    // Strafing a mecanum costs more than driving it.
    kTurnMin: 0.08,
    kDriveMin: 0.10,
    kStrafeMin: 0.12,

    // Deadbands. A breakaway power with no deadband is a limit cycle: apply
    // the minimum, overshoot through zero, apply it the other way, hunt
    // forever. Inside these the output is exactly zero.
    turnDeadbandDeg: 1.5,
    driveDeadbandIn: 1.0,
    strafeDeadbandIn: 1.0,

    maxTurn: 0.45,
    maxDrive: 0.55,
    maxStrafe: 0.35,

    slewPerSec: 3.0,
    useStrafe: true,
  };
}

export class BallPursuit {
  constructor(params = defaultParams()) {
    this.p = params;
    this.reset();
  }

  reset() {
    this.lastForward = 0;
    this.lastStrafe = 0;
    this.lastTurn = 0;
    this.committed = false;
    this.sinceCommit = 0;
    this.rangeAtCommit = 0;
  }

  /** True once committed to the final run; losing the ball is then expected. */
  isCommitted() { return this.committed; }

  /**
   * @param {{rangeIn:number, lateralIn:number, txDeg:number}|null} t
   * @param {number} dt seconds since the last call
   */
  update(t, dt) {
    const p = this.p;
    const out = { forward: 0, strafe: 0, turn: 0, aimed: false, captured: false };

    // --- committed: the final run, open loop ------------------------
    // The camera is ignored from here. The ball is about to pass under the
    // blind radius and stop being reported, and that silence is normal.
    if (this.committed) {
      this.sinceCommit += dt;
      if (this.sinceCommit >= p.blindDriveSeconds) {
        out.captured = true;
        return this._slew(out, dt);
      }
      out.forward = p.blindDrivePower;
      return this._slew(out, dt);
    }

    if (!t) return this._slew(out, dt);

    out.aimed = Math.abs(t.txDeg) <= p.aimToleranceDeg;
    const rangeError = t.rangeIn - p.standoffIn;

    if (t.rangeIn <= p.blindApproachIn) {
      this.committed = true;
      this.sinceCommit = 0;
      this.rangeAtCommit = t.rangeIn;
      out.forward = p.blindDrivePower;
      return this._slew(out, dt);
    }

    out.turn = command(-p.kTurn * t.txDeg, t.txDeg, p.turnDeadbandDeg,
                       p.kTurnMin, p.maxTurn);

    // Scaled down while badly misaimed, so it arcs in rather than driving past.
    const aimScale = Math.max(0.15, 1 - Math.abs(t.txDeg) / (3 * p.aimToleranceDeg));
    out.forward = command(p.kDrive * rangeError * aimScale, rangeError,
                          p.driveDeadbandIn, p.kDriveMin, p.maxDrive);

    if (p.useStrafe) {
      out.strafe = command(p.kStrafe * t.lateralIn, t.lateralIn,
                           p.strafeDeadbandIn, p.kStrafeMin, p.maxStrafe);
    }
    return this._slew(out, dt);
  }

  _slew(out, dt) {
    const step = this.p.slewPerSec * Math.min(Math.max(dt, 0), 0.1);
    this.lastForward = ramp(this.lastForward, out.forward, step);
    this.lastStrafe = ramp(this.lastStrafe, out.strafe, step);
    this.lastTurn = ramp(this.lastTurn, out.turn, step);
    out.forward = this.lastForward;
    out.strafe = this.lastStrafe;
    out.turn = this.lastTurn;
    return out;
  }
}

/** Proportional term plus a breakaway minimum, with a deadband. */
function command(raw, error, deadband, min, max) {
  if (Math.abs(error) < deadband) return 0;
  const v = raw + Math.sign(raw) * min;
  return Math.max(-max, Math.min(max, v));
}

function ramp(from, to, step) {
  const d = to - from;
  if (d > step) return from + step;
  if (d < -step) return from - step;
  return to;
}
