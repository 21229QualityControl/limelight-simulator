// Pinhole camera + flat-floor solver.
//
// A direct port of the Geometry class in ball_snapscript.py. Any change here
// must be mirrored there and vice versa, or the simulator stops predicting
// the robot. test/geometry.test.js pins it to measurements taken on the real
// camera.

// Lens calibration. These are NOT universal -- every camera has its own, and
// using someone else's silently biases every distance. Read yours from the
// camera itself:
//
//     curl -s http://172.29.0.1:5807/hwreport | python3 -m json.tool
//
// and copy FOCAL_LEN_X_PIXELS, FOCAL_LEN_Y_PIXELS, PRINCIPAL_X_PIXELS,
// PRINCIPAL_Y_PIXELS, RES_X, RES_Y into a preset below.
export const CAMERAS = {
  "limelight-3a": {
    label: "Limelight 3A (measured)",
    calW: 1280.0, calH: 960.0,
    fx: 1221.445, fy: 1223.398,
    cx: 637.226, cy: 502.549,
    vfovBottomDeg: 20.169729814416087,
    note: "Read from a real unit's /hwreport. Other units should be close.",
  },
  "generic-70": {
    label: "Generic 70 deg diagonal",
    calW: 1280.0, calH: 960.0,
    fx: 1150.0, fy: 1150.0,
    cx: 640.0, cy: 480.0,
    vfovBottomDeg: 22.6,
    note: "A plausible stand-in. Replace with your own calibration.",
  },
};

export const DEFAULT_CAMERA = "limelight-3a";

/** Mutable active calibration. Change through setCamera(). */
export let CAL = { ...CAMERAS[DEFAULT_CAMERA] };

/** @param {string|object} c preset key, or a calibration object. */
export function setCamera(c) {
  CAL = typeof c === "string" ? { ...CAMERAS[c] } : { ...CAL, ...c };
  return CAL;
}

export const BALLS = {
  pickleball: { label: 'Pickleball / pollen (2.91")', diameterIn: 2.91 },
  decode: { label: 'DECODE artifact (5.0")', diameterIn: 5.0 },
};

/** Scales EVERY distance. Must match the ball you are actually chasing. */
export let BALL_DIAMETER_IN = 2.91;
export function setBallDiameter(inches) { BALL_DIAMETER_IN = inches; }

export class Geometry {
  /**
   * @param {number} imgW  working image width, px
   * @param {number} imgH  working image height, px
   * @param {number} camHeightIn lens centre above the floor
   * @param {number} pitchDeg degrees BELOW horizontal
   */
  /** @param {object} [cal] calibration override; defaults to the active one. */
  constructor(imgW, imgH, camHeightIn, pitchDeg, cal = CAL, ballDiameterIn = BALL_DIAMETER_IN) {
    const s = imgW / cal.calW;
    this.fx = cal.fx * s;
    this.fy = cal.fy * (imgH / cal.calH);
    this.cx = cal.cx * s;
    this.cy = cal.cy * (imgH / cal.calH);
    this.cal = cal;
    this.h = camHeightIn;
    this.ballR = ballDiameterIn * 0.5;
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
    const a = ((this.pitchDeg + this.cal.vfovBottomDeg) * Math.PI) / 180;
    return (this.h - this.ballR) / Math.tan(a);
  }
}
