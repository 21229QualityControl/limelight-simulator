// Delay is the thing a simulator is most tempted to leave out, and the thing
// most likely to make its tuning wrong. These pin what it actually costs.
import { runChase } from "./loop.js";
import { defaultParams } from "../src/pursuit.js";
import { DelayLine, defaultPipeline } from "../src/latency.js";

const chase = (latencyMs, over = {}) =>
  runChase({
    params: { ...defaultParams(), ...over },
    pipeline: { latencyMs, pollHz: 20 },
  });

export default function (t) {
  // --- the delay line itself -------------------------------------------
  {
    const line = new DelayLine({ latencyMs: 50, pollHz: 1000 });
    line.push(0, { rangeIn: 10, lateralIn: 0, txDeg: 0 });
    t.eq("nothing is readable before the latency has elapsed", line.read(10), null);
    line.push(10, { rangeIn: 9, lateralIn: 0, txDeg: 0 });
    t.eq("still nothing at 40 ms", line.read(40), null);
    t.eq("the first frame arrives at 50 ms", line.read(50).rangeIn, 10);
    t.eq("and the second at 60", line.read(60).rangeIn, 9);
  }
  {
    // Poll rate holds a stale value between polls, as the SDK does.
    const line = new DelayLine({ latencyMs: 0, pollHz: 20 });   // 50 ms polls
    line.push(0, { rangeIn: 10, lateralIn: 0, txDeg: 0 });
    t.eq("first read delivers", line.read(0).rangeIn, 10);
    line.push(10, { rangeIn: 5, lateralIn: 0, txDeg: 0 });
    t.eq("a newer frame is NOT delivered before the next poll", line.read(20).rangeIn, 10);
    t.eq("it arrives once the poll interval passes", line.read(60).rangeIn, 5);
  }

  // --- what delay costs the loop ----------------------------------------
  {
    const ideal = chase(0);
    const real = chase(34);          // measured on the camera
    const bad = chase(150);

    t.ok("the loop still converges with no delay", ideal.captured);
    t.ok("and at the real 34 ms", real.captured);
    t.ok("and even at a punishing 150 ms", bad.captured);

    t.eq(`no delay: the approach does not hunt (${ideal.reversals} reversals)`,
         ideal.reversals, 0);
    t.ok(`34 ms of delay introduces hunting (${real.reversals} reversals)`,
         real.reversals > ideal.reversals);
    t.ok(`150 ms overshoots far more (${bad.maxAbsTx.toFixed(0)} deg vs ` +
         `${ideal.maxAbsTx.toFixed(0)} deg)`,
         bad.maxAbsTx > ideal.maxAbsTx * 1.5);
    t.ok(`and takes longer (${bad.seconds.toFixed(1)}s vs ${ideal.seconds.toFixed(1)}s)`,
         bad.seconds > ideal.seconds);
  }

  // --- the warning this exists to give ----------------------------------
  // Tuning with no delay blesses gains that hunt once the real pipeline is
  // in the loop. This records that the effect is real, not theoretical.
  {
    const p = defaultParams();
    const ideal = chase(0, { kTurn: p.kTurn });
    const real = chase(defaultPipeline().latencyMs, { kTurn: p.kTurn });
    t.ok("the shipped kTurn is clean at zero delay but not at the real one",
         ideal.reversals === 0 && real.reversals > 0);
  }
}
