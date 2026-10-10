// Minimal software rasteriser so the detector can be tested in node, with no
// canvas and no browser. Only what the tests need.
export function makeImage(w, h, rgb = [108, 110, 112]) {
  const data = new Uint8ClampedArray(w * h * 4);
  for (let i = 0; i < w * h; i++) {
    data[i * 4] = rgb[0]; data[i * 4 + 1] = rgb[1];
    data[i * 4 + 2] = rgb[2]; data[i * 4 + 3] = 255;
  }
  return { width: w, height: h, data };
}

export function fillCircle(img, cu, cv, r, rgb) {
  const x0 = Math.max(0, Math.floor(cu - r)), x1 = Math.min(img.width - 1, Math.ceil(cu + r));
  const y0 = Math.max(0, Math.floor(cv - r)), y1 = Math.min(img.height - 1, Math.ceil(cv + r));
  for (let y = y0; y <= y1; y++)
    for (let x = x0; x <= x1; x++) {
      if ((x - cu) ** 2 + (y - cv) ** 2 > r * r) continue;
      const i = (y * img.width + x) * 4;
      img.data[i] = rgb[0]; img.data[i + 1] = rgb[1]; img.data[i + 2] = rgb[2];
    }
}

/** A ball with its dimples, which is what the morphological close is for. */
export function drawBall(img, cu, cv, r, rgb) {
  fillCircle(img, cu, cv, r, rgb);
  const dark = rgb.map((c) => Math.round(c * 0.3));
  const rings = [{ n: 1, d: 0 }, { n: 6, d: 0.45 }, { n: 8, d: 0.78 }];
  for (const ring of rings)
    for (let i = 0; i < ring.n; i++) {
      const a = (i / ring.n) * Math.PI * 2 + ring.d;
      fillCircle(img, cu + Math.cos(a) * r * ring.d, cv + Math.sin(a) * r * ring.d,
                 Math.max(1, r * 0.14), dark);
    }
}

/** A straight strip of tape, constant width in pixels. */
export function drawStripe(img, u0, v0, u1, v1, widthPx, rgb) {
  const steps = Math.ceil(Math.hypot(u1 - u0, v1 - v0));
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    fillCircle(img, u0 + (u1 - u0) * t, v0 + (v1 - v0) * t, widthPx / 2, rgb);
  }
}
