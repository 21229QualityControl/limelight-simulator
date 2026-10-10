// Headless closed-loop chase, for control questions.
//
// Skips rendering and detection entirely and hands the controller a perfect
// measurement, because the question here is about the LOOP -- gains, delay,
// stability -- not about pixels. Rendering every frame would be slower and
// would confound a control result with a vision one.
import { BallPursuit, defaultParams } from "../src/pursuit.js";
import { Robot, defaultChassis } from "../src/robot.js";
import { DelayLine, defaultPipeline } from "../src/latency.js";

/**
 * @returns {{captured:boolean, seconds:number, reversals:number,
 *            maxAbsTx:number, finalRangeIn:number, path:Array}}
 */
export function runChase(opts = {}) {
  const {
    params = defaultParams(),
    pipeline = defaultPipeline(),
    ball = { x: 48, y: 14 },
    start = { x: 0, y: 0, headingRad: 0 },
    dt = 0.02,
    maxSeconds = 20,
    chassis = defaultChassis(),
  } = opts;

  const robot = new Robot(chassis, { ...start });
  const pursuit = new BallPursuit(params);
  const line = new DelayLine(pipeline);

  let t = 0, captured = false, reversals = 0, lastTurnSign = 0, maxAbsTx = 0;
  const path = [];

  while (t < maxSeconds && !captured) {
    const c = robot.worldToCamera(ball.x, ball.y);
    const rangeIn = Math.hypot(c.forwardIn, c.lateralIn);
    const txDeg = (Math.atan2(-c.lateralIn, c.forwardIn) * 180) / Math.PI;

    // The camera cannot see inside its blind radius; silence is normal there.
    const visible = c.forwardIn > 0 && rangeIn > params.cameraBlindIn;
    line.push(t * 1000, visible ? { rangeIn, lateralIn: c.lateralIn, txDeg } : null);

    const seen = line.read(t * 1000);
    const o = pursuit.update(seen, dt);
    robot.step(o, dt);

    if (visible) maxAbsTx = Math.max(maxAbsTx, Math.abs(txDeg));
    const s = Math.sign(o.turn);
    if (s !== 0 && lastTurnSign !== 0 && s !== lastTurnSign) reversals++;
    if (s !== 0) lastTurnSign = s;

    path.push({ t, x: robot.pose.x, y: robot.pose.y, txDeg, turn: o.turn });
    captured = o.captured;
    t += dt;
  }

  const c = robot.worldToCamera(ball.x, ball.y);
  return {
    captured,
    seconds: t,
    reversals,
    maxAbsTx,
    finalRangeIn: Math.hypot(c.forwardIn, c.lateralIn),
    path,
  };
}
