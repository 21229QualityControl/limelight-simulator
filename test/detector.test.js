// Exercises the detector on synthetic frames with known ground truth.
// These are the cases that actually broke on the real robot.
import { detect } from "../src/detector.js";
import { Geometry } from "../src/geometry.js";
import { BALL_RGB } from "../src/render.js";
import { makeImage, drawBall, drawStripe } from "./raster.js";

const W = 640, H = 480, CAM_H = 11.7, PITCH = 25.9;
const full = () => new Geometry(W, H, CAM_H, PITCH);

/** Put a ball at a known floor position and return the frame. */
function frameWith(balls, tape = null) {
  const img = makeImage(W, H);
  const g = full();
  if (tape) drawStripe(img, tape[0], tape[1], tape[2], tape[3], tape[4], [40, 85, 190]);
  const sorted = [...balls].sort(
    (a, b) => Math.hypot(b.f, b.l) - Math.hypot(a.f, a.l));
  for (const b of sorted) {
    const p = g.project(b.f, b.l);
    if (p) drawBall(img, p.u, p.v, p.radiusPx, BALL_RGB[b.c]);
  }
  return img;
}

export default function (t) {
  // --- a single ball, at a distance we know exactly -----------------
  {
    const img = frameWith([{ f: 36, l: 0, c: "yellow" }]);
    const d = detect(img);
    t.eq("single ball: found exactly one", d.balls.length, 1);
    t.eq("single ball: reported valid", d.llpython[0], 1);
    t.eq("single ball: correct colour", d.best && d.best.color, "yellow");
    t.near("single ball: range recovered", d.llpython[4], 36.0, 2.0);
    t.near("single ball: bearing centred", d.llpython[2], 0.0, 1.5);
  }

  // --- colours are told apart ----------------------------------------
  for (const c of ["red", "blue", "yellow"]) {
    const d = detect(frameWith([{ f: 30, l: 0, c }]));
    t.eq(`${c} ball identified as ${c}`, d.best && d.best.color, c);
  }

  // --- tape must not become a ball -----------------------------------
  {
    // A blue stripe across the floor, no balls at all.
    const img = makeImage(W, H);
    drawStripe(img, 40, 300, 600, 260, 14, [40, 85, 190]);
    const d = detect(img);
    t.eq("bare tape produces no detections", d.balls.length, 0);
  }

  // --- a blue ball TOUCHING blue tape: the hardest real case ----------
  {
    const g = full();
    const p = g.project(30, 0);
    const img = makeImage(W, H);
    drawStripe(img, 0, p.v + p.radiusPx * 0.9, W, p.v + p.radiusPx * 0.9, 14, [40, 85, 190]);
    drawBall(img, p.u, p.v, p.radiusPx, BALL_RGB.blue);
    const d = detect(img);
    t.ok("blue ball on blue tape is still found", d.balls.length >= 1);
    if (d.best) t.near("and its range survives the tape", d.best.rangeIn, 30.0, 3.0);
  }

  // --- two touching balls must not merge into one ---------------------
  {
    const g = full();
    const pa = g.project(30, 0);
    const ballIn = 2.91;
    const d = detect(frameWith([
      { f: 30, l: -ballIn * 0.52, c: "yellow" },
      { f: 30, l: ballIn * 0.52, c: "yellow" },
    ]));
    t.eq("two touching balls stay two", d.balls.length, 2);
  }

  // --- a row of balls, as on the practice field -----------------------
  {
    const row = [];
    for (let i = -3; i <= 3; i++) row.push({ f: 40, l: i * 4, c: i % 2 ? "yellow" : "blue" });
    const d = detect(frameWith(row));
    t.ok(`row of 7 balls: found at least 5 (got ${d.balls.length})`, d.balls.length >= 5);
  }

  // --- nothing at all ------------------------------------------------
  {
    const d = detect(makeImage(W, H));
    t.eq("empty floor reports no ball", d.llpython[0], 0);
    t.eq("empty floor reports zero count", d.llpython[7], 0);
  }

  // --- colour filter --------------------------------------------------
  {
    const img = frameWith([{ f: 30, l: -6, c: "red" }, { f: 30, l: 6, c: "blue" }]);
    const d = detect(img, { wantedColor: "blue" });
    t.eq("colour filter returns only blue", d.balls.length, 1);
    t.eq("and it is the blue one", d.best && d.best.color, "blue");
  }
}
