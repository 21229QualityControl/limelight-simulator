// Kinematic mecanum robot.
//
// Not a dynamics model -- no mass, no wheel slip. It models the one physical
// effect the controller is actually shaped around: STATIC FRICTION. Below a
// breakaway power the robot does not move at all, however long you ask.
//
// That matters because it is the bug the breakaway minimums exist to fix. Set
// kDriveMin to 0 in the pursuit params and this model will reproduce the
// original failure: the robot creeps to a halt with a standing error and the
// controller has no way to notice. A frictionless model would hide it.

export function defaultChassis() {
  return {
    // Full-power speeds. Measure these on the real robot to make the sim's
    // timing meaningful; the defaults are a plausible FTC mecanum.
    maxForwardIps: 60.0,
    maxStrafeIps: 45.0,       // strafing a mecanum is slower than driving it
    maxTurnDps: 220.0,

    // Power below which nothing moves. The reason kDriveMin and friends exist.
    breakawayForward: 0.09,
    breakawayStrafe: 0.11,
    breakawayTurn: 0.07,

    // Where the camera sits relative to robot centre, inches.
    cameraForwardIn: 7.0,
    cameraLateralIn: 0.0,
  };
}

export class Robot {
  constructor(chassis = defaultChassis(), pose = { x: 0, y: 0, headingRad: 0 }) {
    this.c = chassis;
    this.pose = { ...pose };
  }

  /**
   * Advance one tick.
   * @param {{forward:number, strafe:number, turn:number}} power robot-centric
   * @param {number} dt seconds
   */
  step(power, dt) {
    const c = this.c;
    const f = deadStart(power.forward, c.breakawayForward);
    const s = deadStart(power.strafe, c.breakawayStrafe);
    const r = deadStart(power.turn, c.breakawayTurn);

    const vf = f * c.maxForwardIps;
    const vs = s * c.maxStrafeIps;
    const w = (r * c.maxTurnDps * Math.PI) / 180;

    // robot frame -> field frame
    const h = this.pose.headingRad;
    this.pose.x += (vf * Math.cos(h) - vs * Math.sin(h)) * dt;
    this.pose.y += (vf * Math.sin(h) + vs * Math.cos(h)) * dt;
    this.pose.headingRad = wrap(h + w * dt);
    return this.pose;
  }

  /** Camera position in field coordinates. */
  cameraPose() {
    const h = this.pose.headingRad;
    return {
      x: this.pose.x + this.c.cameraForwardIn * Math.cos(h) - this.c.cameraLateralIn * Math.sin(h),
      y: this.pose.y + this.c.cameraForwardIn * Math.sin(h) + this.c.cameraLateralIn * Math.cos(h),
      headingRad: h,
    };
  }

  /**
   * A field point as the CAMERA sees it.
   * @returns {{forwardIn:number, lateralIn:number}} forward + ahead, lateral + left
   */
  worldToCamera(x, y) {
    const cam = this.cameraPose();
    const dx = x - cam.x, dy = y - cam.y;
    const ch = Math.cos(-cam.headingRad), sh = Math.sin(-cam.headingRad);
    return { forwardIn: dx * ch - dy * sh, lateralIn: dx * sh + dy * ch };
  }
}

/** Static friction: below breakaway, nothing happens. */
function deadStart(power, breakaway) {
  return Math.abs(power) < breakaway ? 0 : power;
}

function wrap(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}
