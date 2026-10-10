// Pinhole camera + flat-floor solver.
//
// A direct port of the Geometry class in ball_snapscript.py. Any change here
// must be mirrored there and vice versa, or the simulator stops predicting
// the robot. test/geometry.test.js pins it to measurements taken on the real
// camera.

// Lens calibration read from this Limelight's own /hwreport, at 1280x960.
// Per-unit values: a different camera needs its own numbers.
export const CAL_W = 1280.0;
export const CAL_H = 960.0;
export const FX = 1221.445;
export const FY = 1223.398;
export const CX = 637.226;
export const CY = 502.549;

/** Half-FOV below the optical axis, degrees. Sets where the camera goes blind. */
export const VFOV_BOTTOM_DEG = 20.169729814416087;

/** 74 mm pickleball. A DECODE artifact is 5.0; this scales every distance. */
export const BALL_DIAMETER_IN = 2.91;

export class Geometry {
  /**
   * @param {number} imgW  working image width, px
   * @param {number} imgH  working image height, px
   * @param {number} camHeightIn lens centre above the floor
   * @param {number} pitchDeg degrees BELOW horizontal
   */
  constructor(imgW, imgH, camHeightIn, pitchDeg) {
    const s = imgW / CAL_W;
    this.fx = FX * s;
    this.fy = FY * (imgH / CAL_H);
    this.cx = CX * s;
    this.cy = CY * (imgH / CAL_H);
    this.h = camHeightIn;
    this.ballR = BALL_DIAMETER_IN * 0.5;
    const t = (pitchDeg * Math.PI) / 180;
    this.st = Math.sin(t);
    this.ct = Math.cos(t);
    this.pitchDeg = pitchDeg;
    this.imgW = imgW;
    this.imgH = imgH;
  }

  /** Unit-ish ray in robot frame: X forward, Y left, Z up. */
  ray(u, v) {
    const x = (u - this.cx) / this.fx;
    const y = (v - this.cy) / this.fy;
    return [this.ct - y * this.st, -x, -y * this.ct - this.st];
  }

  /**
   * Where the pixel's ray meets a ball centre sitting on the floor.
   * @returns {{rangeIn:number, lateralIn:number, forwardIn:number}|null}
   *          null when the ray never reaches the floor (above the horizon).
   */
  solve(u, v) {
    const [X, Y, Z] = this.ray(u, v);
    if (Z >= -1e-6) return null;
    const t = (this.ballR - this.h) / Z;
    if (t <= 0) return null;
    const fwd = t * X;
    const lat = t * Y;
    return { rangeIn: Math.hypot(fwd, lat), lateralIn: lat, forwardIn: fwd };
  }

  /**
   * Apparent radius of a real ball whose centre sits at image row v.
   * This is what separates a ball from floor tape: tape is far thinner.
   */
  expectedRadiusPx(v) {
    const s = this.solve(this.cx, v);
    if (s === null) return null;
    const slant = Math.hypot(s.rangeIn, this.h - this.ballR);
    if (slant < 1.0) return null;
    return (this.ballR * this.fx) / slant;
  }

  /** @returns {{txDeg:number, tyDeg:number}} + tx is right, + ty is up. */
  angles(u, v) {
    return {
      txDeg: (Math.atan2(u - this.cx, this.fx) * 180) / Math.PI,
      tyDeg: (Math.atan2(-(v - this.cy), this.fy) * 180) / Math.PI,
    };
  }

  /**
   * Inverse of solve(): where a ball at this floor position lands in the
   * image. The renderer needs this; the detector does not.
   * @returns {{u:number, v:number, radiusPx:number}|null} null if behind or
   *          outside the frame.
   */
  project(forwardIn, lateralIn) {
    // robot frame -> camera frame
    const dz = this.ballR - this.h;          // ball centre relative to lens
    const zc = forwardIn * this.ct - dz * this.st;
    if (zc <= 0.1) return null;              // behind the camera
    const xc = -lateralIn;
    const yc = -(forwardIn * this.st + dz * this.ct);
    const u = this.cx + (xc / zc) * this.fx;
    const v = this.cy + (yc / zc) * this.fy;
    const slant = Math.hypot(forwardIn, lateralIn, dz);
    return { u, v, radiusPx: (this.ballR * this.fx) / slant };
  }

  /** Nearest ball the camera can see, inches. Below this there is nothing. */
  nearestVisibleIn() {
    const a = ((this.pitchDeg + VFOV_BOTTOM_DEG) * Math.PI) / 180;
    return (this.h - this.ballR) / Math.tan(a);
  }
}
