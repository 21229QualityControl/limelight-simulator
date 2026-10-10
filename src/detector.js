// Port of ball_snapscript.py, the detector that runs ON the Limelight.
//
// Faithful on purpose: same constants, same order of operations, same output.
// If this drifts from the Python, the simulator stops predicting the robot.
//
// Why not a colour-blob finder: thresholding and taking contours fails twice
// on a real field. A blue ball touching blue tape becomes one blob, and a row
// of touching balls becomes one blob with a meaningless centre. So instead:
//   1. morphological close, to fill the wiffle holes
//   2. distance-transform peak picking, which splits touching balls and gives
//      a real radius rather than a bounding box
//   3. a directional ring test, so a ball may touch things but must be free
//      in most directions, while a blob inside a wall is enclosed and fails
//   4. a ground-plane size gate, once the mounting numbers are calibrated

import { Geometry, BALL_DIAMETER_IN } from "./geometry.js";
import { THRESHOLDS, COLOR_ID } from "./thresholds.js";

export const PROC_W = 320;          // detection runs downscaled; angles stay exact
export const MIN_R = 5.0;           // px at PROC_W, smallest ball we will chase
export const MAX_CANDIDATES = 8;    // peaks pulled per colour per frame
export const FILL_MIN = 0.72;       // mask coverage inside the candidate circle
export const CLOSE_R = 3;           // hole-fill radius
export const OPEN_R = 2;            // speckle-kill radius

export const RING_SECTORS = 16;
export const RING_SECTOR_FULL = 0.5;  // a sector this covered counts as blocked
export const RING_FREE_MIN = 0.45;    // fraction of sectors that must be free

export const SIZE_GATE_LO = 0.35;
export const SIZE_GATE_HI = 3.0;
export const GATE_MAX_RANGE_IN = 48.0; // past this the prediction is too soft to gate on
export const MAX_RANGE_IN = 100.0;     // beyond this is not a drivable target

export const TURN_WEIGHT_IN_PER_DEG = 0.9;

/**
 * @param {{width:number,height:number,data:Uint8ClampedArray}} image RGBA
 * @param {object} opts
 * @param {number} [opts.camHeightIn=11.7]
 * @param {number} [opts.camPitchDeg=25.9]
 * @param {string} [opts.wantedColor] 'red'|'blue'|'yellow', omit for any
 * @param {boolean} [opts.sizeGate=false] only enable once calibrated
 * @param {number} [opts.minR]
 * @returns {{llpython:number[], balls:object[], best:object|null, masks:object}}
 */
export function detect(image, opts = {}) {
  const {
    camHeightIn = 11.7,
    camPitchDeg = 25.9,
    wantedColor = null,
    sizeGate = false,
    minR = MIN_R,
  } = opts;

  const small = downscale(image, PROC_W);
  const geom = new Geometry(small.width, small.height, camHeightIn, camPitchDeg);
  const hsv = toHsv(small);

  const colors = wantedColor ? [wantedColor] : ["red", "blue", "yellow"];
  const balls = [];
  const masks = {};

  for (const color of colors) {
    const mask = buildMask(hsv, small.width, small.height, color);
    masks[color] = mask;
    for (const c of peaks(mask, small.width, small.height, geom, minR, sizeGate)) {
      const sol = geom.solve(c.u, c.v);
      if (sol === null || sol.rangeIn > MAX_RANGE_IN) continue;
      const { txDeg, tyDeg } = geom.angles(c.u, c.v);
      balls.push({
        color, u: c.u, v: c.v, r: c.r, fill: c.fill, sizeScore: c.sizeScore,
        rangeIn: sol.rangeIn, lateralIn: sol.lateralIn, forwardIn: sol.forwardIn,
        txDeg, tyDeg,
      });
    }
  }

  // Cross-colour de-duplication: one physical ball, one report.
  balls.sort((a, b) => b.r * b.fill * b.sizeScore - a.r * a.fill * a.sizeScore);
  const kept = [];
  for (const b of balls) {
    const clash = kept.some(
      (k) => (b.u - k.u) ** 2 + (b.v - k.v) ** 2 <= (0.8 * Math.max(b.r, k.r)) ** 2
    );
    if (!clash) kept.push(b);
  }

  let best = null;
  if (kept.length) {
    best = kept.reduce((lo, b) =>
      b.rangeIn + TURN_WEIGHT_IN_PER_DEG * Math.abs(b.txDeg) <
      lo.rangeIn + TURN_WEIGHT_IN_PER_DEG * Math.abs(lo.txDeg) ? b : lo
    );
  }

  const llpython = [0, 0, 0, 0, 0, 0, 0, kept.length];
  if (best) {
    llpython[0] = 1;
    llpython[1] = COLOR_ID[best.color];
    llpython[2] = best.txDeg;
    llpython[3] = best.tyDeg;
    llpython[4] = best.rangeIn;
    llpython[5] = best.lateralIn;
    llpython[6] = best.r;
  }
  return { llpython, balls: kept, best, masks, width: small.width, height: small.height, geom };
}

// ---------------------------------------------------------------- pixels

function downscale(image, targetW) {
  if (image.width <= targetW) return image;
  const scale = targetW / image.width;
  const w = targetW, h = Math.round(image.height * scale);
  const out = new Uint8ClampedArray(w * h * 4);
  const bx = image.width / w, by = image.height / h;
  // Box average, matching cv2.INTER_AREA closely enough for thresholding.
  for (let y = 0; y < h; y++) {
    const y0 = Math.floor(y * by), y1 = Math.min(image.height, Math.ceil((y + 1) * by));
    for (let x = 0; x < w; x++) {
      const x0 = Math.floor(x * bx), x1 = Math.min(image.width, Math.ceil((x + 1) * bx));
      let r = 0, g = 0, b = 0, n = 0;
      for (let yy = y0; yy < y1; yy++) {
        for (let xx = x0; xx < x1; xx++) {
          const i = (yy * image.width + xx) * 4;
          r += image.data[i]; g += image.data[i + 1]; b += image.data[i + 2]; n++;
        }
      }
      const o = (y * w + x) * 4;
      out[o] = r / n; out[o + 1] = g / n; out[o + 2] = b / n; out[o + 3] = 255;
    }
  }
  return { width: w, height: h, data: out };
}

function toHsv(img) {
  const n = img.width * img.height;
  const H = new Uint8Array(n), S = new Uint8Array(n), V = new Uint8Array(n);
  for (let i = 0; i < n; i++) {
    const r = img.data[i * 4], g = img.data[i * 4 + 1], b = img.data[i * 4 + 2];
    const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
    let h = 0;
    if (d !== 0) {
      if (mx === r) h = ((g - b) / d) % 6;
      else if (mx === g) h = (b - r) / d + 2;
      else h = (r - g) / d + 4;
    }
    h = Math.round((h * 60) / 2);
    if (h < 0) h += 180;
    H[i] = h;
    S[i] = mx === 0 ? 0 : Math.round((d / mx) * 255);
    V[i] = mx;
  }
  return { H, S, V };
}

function buildMask(hsv, w, h, color) {
  const n = w * h;
  const m = new Uint8Array(n);
  const boxes = THRESHOLDS[color];
  for (let i = 0; i < n; i++) {
    const H = hsv.H[i], S = hsv.S[i], V = hsv.V[i];
    for (const [lo, hi] of boxes) {
      if (H >= lo[0] && H <= hi[0] && S >= lo[1] && V >= lo[2]) { m[i] = 1; break; }
    }
  }
  // close (fill the wiffle holes), then open (kill speckle)
  return morphOpen(morphClose(m, w, h, CLOSE_R), w, h, OPEN_R);
}

function diskOffsets(r) {
  const offs = [];
  for (let dy = -r; dy <= r; dy++)
    for (let dx = -r; dx <= r; dx++) if (dx * dx + dy * dy <= r * r) offs.push([dx, dy]);
  return offs;
}

function dilate(m, w, h, r) {
  const offs = diskOffsets(r), out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      if (!m[y * w + x]) continue;
      for (const [dx, dy] of offs) {
        const nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < w && ny >= 0 && ny < h) out[ny * w + nx] = 1;
      }
    }
  return out;
}

function erode(m, w, h, r) {
  const offs = diskOffsets(r), out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      let all = 1;
      for (const [dx, dy] of offs) {
        const nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h || !m[ny * w + nx]) { all = 0; break; }
      }
      out[y * w + x] = all;
    }
  return out;
}

const morphClose = (m, w, h, r) => erode(dilate(m, w, h, r), w, h, r);
const morphOpen = (m, w, h, r) => dilate(erode(m, w, h, r), w, h, r);

/**
 * Chamfer distance transform, the 5x5 weights OpenCV uses for DIST_L2.
 * Each pixel ends up holding its distance to the nearest zero.
 */
function distanceTransform(m, w, h) {
  const A = 1.0, B = 1.4, C = 2.1969;
  const INF = 1e9;
  const d = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) d[i] = m[i] ? INF : 0;
  const at = (x, y) => (x < 0 || x >= w || y < 0 || y >= h ? INF : d[y * w + x]);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      if (!m[y * w + x]) continue;
      let v = d[y * w + x];
      v = Math.min(v, at(x - 1, y) + A, at(x, y - 1) + A,
                      at(x - 1, y - 1) + B, at(x + 1, y - 1) + B,
                      at(x - 2, y - 1) + C, at(x + 2, y - 1) + C,
                      at(x - 1, y - 2) + C, at(x + 1, y - 2) + C);
      d[y * w + x] = v;
    }
  for (let y = h - 1; y >= 0; y--)
    for (let x = w - 1; x >= 0; x--) {
      if (!m[y * w + x]) continue;
      let v = d[y * w + x];
      v = Math.min(v, at(x + 1, y) + A, at(x, y + 1) + A,
                      at(x + 1, y + 1) + B, at(x - 1, y + 1) + B,
                      at(x + 2, y + 1) + C, at(x - 2, y + 1) + C,
                      at(x + 1, y + 2) + C, at(x - 1, y + 2) + C);
      d[y * w + x] = v;
    }
  return d;
}

function peaks(mask, w, h, geom, minR, gate) {
  const dt = distanceTransform(mask, w, h);
  const work = Float32Array.from(dt);
  const found = [];

  for (let iter = 0; iter < MAX_CANDIDATES; iter++) {
    let bi = -1, bv = 0;
    for (let i = 0; i < work.length; i++) if (work[i] > bv) { bv = work[i]; bi = i; }
    if (bi < 0 || bv < minR) break;
    const u = bi % w, v = Math.floor(bi / w), r = bv;
    suppress(work, w, h, u, v, r * 1.4);

    // size gate first: cheapest, and it removes tape before the pixel work
    let sizeScore = 1.0;
    if (gate) {
      const centre = geom.solve(geom.cx, v);
      if (centre !== null && centre.rangeIn <= GATE_MAX_RANGE_IN) {
        const exp = geom.expectedRadiusPx(v);
        if (exp === null || exp <= 0) continue;
        const ratio = r / exp;
        if (ratio < SIZE_GATE_LO || ratio > SIZE_GATE_HI) continue;
        sizeScore = Math.exp(-((ratio - 1) ** 2) / 0.5);
      }
    }

    const shape = shapeTest(mask, w, h, u, v, r);
    if (!shape) continue;
    found.push({ u, v, r, fill: shape.fill, sizeScore });
  }
  return found;
}

function suppress(work, w, h, cu, cv, rad) {
  const r2 = rad * rad;
  const x0 = Math.max(0, Math.floor(cu - rad)), x1 = Math.min(w - 1, Math.ceil(cu + rad));
  const y0 = Math.max(0, Math.floor(cv - rad)), y1 = Math.min(h - 1, Math.ceil(cv + rad));
  for (let y = y0; y <= y1; y++)
    for (let x = x0; x <= x1; x++)
      if ((x - cu) ** 2 + (y - cv) ** 2 <= r2) work[y * w + x] = 0;
}

/**
 * Fill plus a DIRECTIONAL ring test.
 *
 * Not a scalar average: a ball touching a tape line or resting against
 * another ball has a partly filled ring, and so does a peak sitting inside a
 * large blob of the same colour. They differ in SHAPE. A ball has to be free
 * in most directions; a blob enclosed on every side fails.
 */
function shapeTest(mask, w, h, cu, cv, r) {
  const R = Math.ceil(r * 1.7) + 1;
  const x0 = Math.max(0, cu - R), x1 = Math.min(w - 1, cu + R);
  const y0 = Math.max(0, cv - R), y1 = Math.min(h - 1, cv + R);
  const r2 = r * r, ring2 = (1.7 * r) ** 2;

  let inside = 0, insideSet = 0;
  const occupied = new Float64Array(RING_SECTORS);
  const total = new Float64Array(RING_SECTORS);

  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      const dx = x - cu, dy = y - cv, d2 = dx * dx + dy * dy;
      const on = mask[y * w + x];
      if (d2 <= r2) { inside++; if (on) insideSet++; continue; }
      if (d2 > ring2) continue;
      let sec = Math.floor(((Math.atan2(dy, dx) + Math.PI) * RING_SECTORS) / (2 * Math.PI));
      if (sec < 0) sec = 0;
      if (sec >= RING_SECTORS) sec = RING_SECTORS - 1;
      total[sec]++;
      if (on) occupied[sec]++;
    }
  }
  if (!inside) return null;
  const fill = insideSet / inside;
  if (fill < FILL_MIN) return null;

  let used = 0, free = 0;
  for (let s = 0; s < RING_SECTORS; s++) {
    if (!total[s]) continue;
    used++;
    if (occupied[s] / total[s] < RING_SECTOR_FULL) free++;
  }
  if (!used || free / used < RING_FREE_MIN) return null;
  return { fill };
}
