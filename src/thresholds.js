// HSV windows, mirroring THRESHOLDS in ball_snapscript.py.
// OpenCV convention: H 0-179, S/V 0-255.
//
// Hue came from measuring 109 photos of the team's own balls. The saturation
// floor of 150 came later, from the camera's own footage: real balls read
// median S 163-244, while everything that produced a false positive -- sky-lit
// metal, a window frame, tan cardboard -- read S 103-140. Saturation, not hue,
// is what separates a plastic ball from the building it is sitting in.
//
// The value floor is 60, well under the 104 a blue ball reads on a correctly
// exposed frame. Blue is always the dimmest of the three.

export const COLOR_ID = { none: 0, red: 1, blue: 2, yellow: 3 };
export const ID_COLOR = { 0: "none", 1: "red", 2: "blue", 3: "yellow" };

/** Each entry is a list of [hMin,sMin,vMin],[hMax,sMax,vMax] boxes. */
export const THRESHOLDS = {
  red: [
    [[0, 150, 60], [8, 255, 255]],       // red wraps the hue seam
    [[170, 150, 60], [179, 255, 255]],
  ],
  blue: [[[103, 150, 60], [122, 255, 255]]],
  yellow: [[[16, 150, 60], [32, 255, 255]]],
};

/** True if an HSV triple falls inside the window for `color`. */
export function matches(color, h, s, v) {
  const boxes = THRESHOLDS[color];
  if (!boxes) return false;
  for (const [lo, hi] of boxes) {
    if (h >= lo[0] && h <= hi[0] && s >= lo[1] && s <= hi[1] && v >= lo[2] && v <= hi[2]) {
      return true;
    }
  }
  return false;
}

/** Which colour window an HSV triple falls into, or null. */
export function classify(h, s, v) {
  for (const c of ["red", "blue", "yellow"]) if (matches(c, h, s, v)) return c;
  return null;
}
