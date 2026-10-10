// Sensor and motion effects, applied to the rendered frame before the
// detector sees it.
//
// Without these the simulator only answers "is the maths right". The real
// robot has never failed because the maths was wrong -- it failed because
// blue pollen went dark at low exposure, because a tan box read as yellow,
// and (still untested on hardware) because the image smears when the robot
// turns. Those are the things worth being able to reproduce on a laptop.
//
// Applied in the order a camera actually does it: the image smears during
// the exposure, then exposure and gain scale it, then the sensor adds noise
// on readout. Gain amplifies noise as well as signal, which is why turning
// gain up is not free.

export function defaultSensor() {
  return {
    // Combined exposure x gain, as a brightness multiplier. 1.0 is the
    // correctly exposed frame the colours were chosen for. The real camera
    // ran at 1500/45 after tuning; before that it was dark enough that blue
    // read value 53 against a floor of 80 and vanished completely.
    brightness: 1.0,

    // Sensor noise. Scales with gain on a real sensor, so the UI ties them.
    noise: 0.0,

    // Exposure time in seconds. Longer means more light but more smear --
    // the trade that made raising gain preferable to raising exposure.
    exposureSec: 0.015,

    motionBlur: true,
  };
}

/**
 * @param {{width:number,height:number,data:Uint8ClampedArray}} img modified in place
 * @param {object} sensor
 * @param {{turnRadPerSec:number, forwardIps:number}} motion
 * @param {number} fx focal length px, to turn angular rate into pixels
 */
export function applySensor(img, sensor, motion, fx) {
  if (sensor.motionBlur) {
    const px = blurPixels(sensor, motion, fx);
    if (px >= 1) horizontalBlur(img, px);
  }
  if (sensor.brightness !== 1.0) scaleBrightness(img, sensor.brightness);
  if (sensor.noise > 0) addNoise(img, sensor.noise);
}

/**
 * How far the image slides during one exposure, in pixels.
 * Rotation dominates: a robot turning at 90 deg/s with a 15 ms exposure
 * smears by 1.35 deg, which at this focal length is about 7 px -- enough to
 * shrink a distant ball below the minimum radius.
 */
export function blurPixels(sensor, motion, fx) {
  const yaw = Math.abs(motion.turnRadPerSec || 0) * sensor.exposureSec;
  return yaw * fx;
}

function horizontalBlur(img, px) {
  const r = Math.min(40, Math.round(px / 2));
  if (r < 1) return;
  const { width: w, height: h, data } = img;
  const row = new Float32Array(w * 3);
  for (let y = 0; y < h; y++) {
    const o = y * w * 4;
    for (let x = 0; x < w; x++)
      for (let c = 0; c < 3; c++) row[x * 3 + c] = data[o + x * 4 + c];
    for (let x = 0; x < w; x++) {
      let n = 0, acc0 = 0, acc1 = 0, acc2 = 0;
      for (let k = -r; k <= r; k++) {
        const xx = x + k;
        if (xx < 0 || xx >= w) continue;
        acc0 += row[xx * 3]; acc1 += row[xx * 3 + 1]; acc2 += row[xx * 3 + 2]; n++;
      }
      data[o + x * 4] = acc0 / n;
      data[o + x * 4 + 1] = acc1 / n;
      data[o + x * 4 + 2] = acc2 / n;
    }
  }
}

function scaleBrightness(img, k) {
  const d = img.data;
  for (let i = 0; i < d.length; i += 4) {
    d[i] = d[i] * k; d[i + 1] = d[i + 1] * k; d[i + 2] = d[i + 2] * k;
  }
}

function addNoise(img, sigma) {
  const d = img.data;
  for (let i = 0; i < d.length; i += 4) {
    // Box-Muller is overkill here; summed uniforms are close enough and fast.
    const n = (Math.random() + Math.random() + Math.random() - 1.5) * 2 * sigma;
    d[i] += n; d[i + 1] += n; d[i + 2] += n;
  }
}
