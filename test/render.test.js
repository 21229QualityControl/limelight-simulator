// The renderer only earns its keep if the pixels it draws are pixels the
// detector would accept. These check that, rather than checking it looks nice.
import { BALL_RGB, rgbToHsv } from "../src/render.js";
import { classify, matches } from "../src/thresholds.js";

export default function (t) {
  for (const [name, c] of Object.entries(BALL_RGB)) {
    const [h, s, v] = rgbToHsv(...c);
    t.eq(`rendered ${name} classifies as ${name}`, classify(h, s, v), name);
  }

  // Shading darkens the rim to 62% and brightens the highlight to 118%.
  // Both extremes must still pass, or the detector would see a ring or a hole.
  for (const [name, c] of Object.entries(BALL_RGB)) {
    for (const [k, where] of [[0.62, "shaded rim"], [1.18, "lit highlight"]]) {
      const scaled = c.map((x) => Math.max(0, Math.min(255, Math.round(x * k))));
      const [h, s, v] = rgbToHsv(...scaled);
      t.ok(`${name} ${where} still inside the window`, matches(name, h, s, v));
    }
  }

  // The floor and the tape must NOT look like balls, except that tape is
  // deliberately the same blue family -- that is the point of it.
  const floor = rgbToHsv(108, 110, 112);
  t.eq("grey floor matches nothing", classify(...floor), null);
  t.ok("floor saturation is below the floor-of-150 rule", floor[1] < 150);

  // Cardboard, the real-world false positive, must be excluded by saturation.
  const cardboard = rgbToHsv(170, 140, 95);
  t.eq("tan cardboard matches nothing", classify(...cardboard), null);
}
