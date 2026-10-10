// Draws the synthetic camera view the detector runs on.
//
// The point of this file is to produce pixels that exercise the SAME parts of
// the detector that the real world does. Two details matter more than looking
// pretty:
//
//   * balls get wiffle holes, because the detector's morphological close
//     exists purely to fill them. Solid discs would skip that step entirely.
//   * the floor gets tape, because same-coloured tape next to a ball is the
//     classic false positive and the hardest case the ring test handles.
//
// Colours are chosen to land inside the detector's HSV windows; the test
// suite asserts that rather than trusting it.

/** Ball colours, picked to sit inside the detector's hue/saturation windows. */
export const BALL_RGB = {
  red:    [205, 28, 32],
  blue:   [28, 70, 205],
  yellow: [232, 200, 30],
};

const FLOOR_RGB = [108, 110, 112];   // grey tile, deliberately low saturation
const TAPE_RGB  = [40, 85, 190];     // blue gaffer tape, the usual troublemaker

/**
 * @typedef {{forwardIn:number, lateralIn:number, color:'red'|'blue'|'yellow'}} Ball
 * @typedef {{from:[number,number], to:[number,number], widthIn:number}} Tape
 * @typedef {{balls:Ball[], tape:Tape[]}} World
 */

/**
 * Render one camera frame.
 * @param {CanvasRenderingContext2D} ctx
 * @param {import('./geometry.js').Geometry} geom
 * @param {World} world
 * @param {{shading?:boolean, holes?:boolean}} [opts]
 */
export function renderCameraView(ctx, geom, world, opts = {}) {
  const { shading = true, holes = true } = opts;
  const W = geom.imgW, H = geom.imgH;

  drawFloor(ctx, W, H);

  for (const t of world.tape || []) drawTape(ctx, geom, t);

  // Painter's algorithm: far balls first, so near ones occlude them.
  const sorted = [...(world.balls || [])].sort(
    (a, b) => Math.hypot(b.forwardIn, b.lateralIn) - Math.hypot(a.forwardIn, a.lateralIn)
  );
  for (const b of sorted) drawBall(ctx, geom, b, shading, holes);
}

function drawFloor(ctx, W, H) {
  // Slight vertical gradient: real floors are darker close to the robot,
  // where the chassis shadows them.
  const g = ctx.createLinearGradient(0, 0, 0, H);
  g.addColorStop(0, rgb(scale(FLOOR_RGB, 1.1)));
  g.addColorStop(1, rgb(scale(FLOOR_RGB, 0.82)));
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, W, H);
}

function drawTape(ctx, geom, tape) {
  // Project a run of points along the strip; width shrinks with distance, so
  // it cannot be stroked as one straight line.
  const steps = 48;
  const [f0, l0] = tape.from, [f1, l1] = tape.to;
  ctx.fillStyle = rgb(TAPE_RGB);
  for (let i = 0; i < steps; i++) {
    const a = i / steps, b = (i + 1) / steps;
    const pa = geom.project(f0 + (f1 - f0) * a, l0 + (l1 - l0) * a);
    const pb = geom.project(f0 + (f1 - f0) * b, l0 + (l1 - l0) * b);
    if (!pa || !pb) continue;
    // width in px from the same scale factor the ball radius uses
    const wpx = Math.max(
      1,
      (tape.widthIn * geom.fx) /
        Math.hypot(f0 + (f1 - f0) * a, l0 + (l1 - l0) * a, geom.h)
    );
    ctx.lineWidth = wpx;
    ctx.strokeStyle = rgb(TAPE_RGB);
    ctx.beginPath();
    ctx.moveTo(pa.u, pa.v);
    ctx.lineTo(pb.u, pb.v);
    ctx.stroke();
  }
}

function drawBall(ctx, geom, ball, shading, holes) {
  const p = geom.project(ball.forwardIn, ball.lateralIn);
  if (!p) return;
  const r = p.radiusPx;
  if (r < 1) return;

  const base = BALL_RGB[ball.color];
  if (!base) return;

  if (shading) {
    // Lit from above and slightly left, like gym lighting.
    const g = ctx.createRadialGradient(
      p.u - r * 0.35, p.v - r * 0.4, r * 0.1,
      p.u, p.v, r
    );
    g.addColorStop(0, rgb(scale(base, 1.18)));
    g.addColorStop(0.65, rgb(base));
    g.addColorStop(1, rgb(scale(base, 0.62)));
    ctx.fillStyle = g;
  } else {
    ctx.fillStyle = rgb(base);
  }
  ctx.beginPath();
  ctx.arc(p.u, p.v, r, 0, Math.PI * 2);
  ctx.fill();

  if (holes && r > 4) drawHoles(ctx, p.u, p.v, r, base);
}

/**
 * The dimples. These are what break a naive colour threshold into a ring of
 * fragments, and why the detector closes the mask before looking for shapes.
 */
function drawHoles(ctx, cu, cv, r, base) {
  ctx.fillStyle = rgb(scale(base, 0.3));
  const rings = [
    { count: 1, dist: 0.0, size: 0.17 },
    { count: 6, dist: 0.45, size: 0.15 },
    { count: 8, dist: 0.78, size: 0.11 },
  ];
  for (const ring of rings) {
    for (let i = 0; i < ring.count; i++) {
      const a = (i / ring.count) * Math.PI * 2 + ring.dist;
      const hu = cu + Math.cos(a) * r * ring.dist;
      const hv = cv + Math.sin(a) * r * ring.dist * 0.92;
      // foreshorten holes near the rim, as a sphere does
      const squash = 1 - 0.55 * ring.dist;
      ctx.beginPath();
      ctx.ellipse(hu, hv, r * ring.size * squash, r * ring.size, a, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}

const scale = (c, k) => c.map((v) => Math.max(0, Math.min(255, Math.round(v * k))));
const rgb = (c) => `rgb(${c[0]},${c[1]},${c[2]})`;

/** RGB -> OpenCV-convention HSV (H 0-179, S/V 0-255). Shared with the detector. */
export function rgbToHsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
  let h = 0;
  if (d !== 0) {
    if (mx === r) h = ((g - b) / d) % 6;
    else if (mx === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
  }
  h = Math.round((h * 60) / 2);           // degrees -> OpenCV half-degrees
  if (h < 0) h += 180;
  const s = mx === 0 ? 0 : Math.round((d / mx) * 255);
  return [h, s, Math.round(mx)];
}
