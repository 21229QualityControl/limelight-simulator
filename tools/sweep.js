// Headless gain sweep.
//
//     node tools/sweep.js            default grid
//     node tools/sweep.js --latency 0    to see what tuning without delay
//                                        would have recommended
//
// Runs the chase across a grid of gains and a spread of ball positions, and
// reports the sets that behave best. Two choices make the answer worth
// trusting:
//
//   * Delay is ON. Tuning against an instant pipeline recommends gains that
//     hunt on the robot -- the shipped kTurn is clean at 0 ms and reverses
//     five times at the real 34 ms.
//   * Each candidate is judged by its WORST scenario, not its average. A
//     gain set that is quick on a ball straight ahead and awful on one off
//     to the side is not a good gain set; auto does not get to choose.

import { runChase } from "../test/loop.js";
import { defaultParams } from "../src/pursuit.js";
import { defaultPipeline } from "../src/latency.js";

const arg = (name, dflt) => {
  const i = process.argv.indexOf("--" + name);
  return i >= 0 ? parseFloat(process.argv[i + 1]) : dflt;
};

const latencyMs = arg("latency", defaultPipeline().latencyMs);

// A spread of situations auto actually produces: near and far, straight
// ahead and well off to one side, both sides.
const SCENARIOS = [
  { ball: { x: 24, y: 0 } },
  { ball: { x: 36, y: 10 } },
  { ball: { x: 36, y: -10 } },
  { ball: { x: 52, y: 22 } },
  { ball: { x: 52, y: -22 } },
  { ball: { x: 70, y: 6 } },
  { ball: { x: 30, y: 26 } },
  { ball: { x: 66, y: -30 } },
];

const range = (lo, hi, step) => {
  const out = [];
  for (let v = lo; v <= hi + 1e-9; v += step) out.push(+v.toFixed(4));
  return out;
};

// The current values are forced into the grid, or the comparison row has
// nothing to compare against -- 0.022 is not on a 0.005 lattice.
const cur0 = defaultParams();
const uniq = (a) => [...new Set(a)].sort((x, y) => x - y);
const GRID = {
  kTurn: uniq([...range(0.004, 0.060, 0.004), cur0.kTurn]),
  kDrive: uniq([...range(0.015, 0.060, 0.005), cur0.kDrive]),
};

/**
 * Cost for one scenario. Lower is better.
 *
 * Time is the thing auto actually spends, so it is the base. A turn reversal
 * costs 0.2 s because that is roughly what a hunt wastes, and because five
 * of them on the field look like a broken robot rather than a tuning choice.
 * Overshoot past 45 degrees is penalised separately: swinging that wide
 * risks losing the ball out of frame entirely.
 */
function cost(r) {
  if (!r.captured) return 1e6;
  return r.seconds + 0.2 * r.reversals + Math.max(0, r.maxAbsTx - 45) * 0.05;
}

const results = [];
for (const kTurn of GRID.kTurn) {
  for (const kDrive of GRID.kDrive) {
    const params = { ...defaultParams(), kTurn, kDrive };
    let worst = -1, worstCase = null, total = 0, reversals = 0, failed = 0, totalSeconds = 0;
    for (const s of SCENARIOS) {
      const r = runChase({ params, pipeline: { latencyMs, pollHz: 20 }, ball: s.ball });
      const c = cost(r);
      if (!r.captured) failed++;
      total += c;
      totalSeconds += r.seconds;
      reversals += r.reversals;
      if (c > worst) { worst = c; worstCase = s.ball; }
    }
    results.push({
      kTurn, kDrive, worst, mean: total / SCENARIOS.length,
      reversals, failed, worstCase, seconds: totalSeconds,
    });
  }
}

const usable = results.filter((r) => r.failed === 0);
const cur = defaultParams();
const curRow = results.find((r) =>
  Math.abs(r.kTurn - cur.kTurn) < 1e-9 && Math.abs(r.kDrive - cur.kDrive) < 1e-9);

const row = (r, label) =>
  `  ${label.padEnd(10)} ${String(r.kTurn).padEnd(7)} ${String(r.kDrive).padEnd(7)}` +
  `${r.seconds.toFixed(1).padStart(8)}s${String(r.reversals).padStart(12)}` +
  `${r.worst.toFixed(2).padStart(13)}`;

console.log(`\nlatency ${latencyMs} ms, ${SCENARIOS.length} scenarios, ` +
            `${results.length} gain combinations, ${usable.length} captured all\n`);
console.log("             kTurn   kDrive  total time   reversals  worst cost");

// Three honest answers rather than one arbitrary weighting, because the
// right trade depends on whether you care more about seconds or about the
// robot not looking broken.
const byTime = [...usable].sort((a, b) => a.seconds - b.seconds)[0];
const bySmooth = [...usable].sort(
  (a, b) => a.reversals - b.reversals || a.seconds - b.seconds)[0];
const byBalance = [...usable].sort((a, b) => a.worst - b.worst)[0];

console.log(row(byTime, "fastest"));
console.log(row(bySmooth, "smoothest"));
console.log(row(byBalance, "balanced"));
if (curRow) {
  console.log(row(curRow, "current"));
}

const best = byBalance;
console.log(`\n  worst scenario for the balanced pick: ` +
            `ball at ${JSON.stringify(best.worstCase)}`);
if (curRow) {
  const dt = curRow.seconds - bySmooth.seconds;
  console.log(`\n  The smoothest gains cut turn reversals from ` +
              `${curRow.reversals} to ${bySmooth.reversals} across all ` +
              `scenarios,\n  ${dt >= 0 ? "and are also" : "at a cost of"} ` +
              `${Math.abs(dt).toFixed(1)}s ${dt >= 0 ? "faster" : "slower"} in total.`);
}
// Two honesty checks on the answer itself.
const kTurnGrid = GRID.kTurn;
if (bySmooth.kTurn <= kTurnGrid[0] + 1e-9) {
  console.log(`\n  NOTE: the pick sits at the bottom of the kTurn grid ` +
              `(${kTurnGrid[0]}), so the true\n  optimum may be lower. But ` +
              `below about 0.008 the proportional term is swamped\n  by ` +
              `kTurnMin (${cur.kTurnMin}) and the controller is effectively ` +
              `bang-bang with a\n  deadband. Prefer a value that is still ` +
              `doing proportional work.`);
}
console.log("\n  These are simulated, on clean pixels, with a kinematic " +
            "robot that has no\n  mass. Treat them as a starting point to " +
            "try on the robot, not as values\n  to trust without driving.\n");
