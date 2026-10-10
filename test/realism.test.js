// These assert the sensor model reproduces real, measured failures.
import { applySensor, defaultSensor, blurPixels } from "../src/realism.js";
import { detect } from "../src/detector.js";
import { Geometry } from "../src/geometry.js";
import { BALL_RGB, rgbToHsv } from "../src/render.js";
import { classify } from "../src/thresholds.js";
import { makeImage, drawBall } from "./raster.js";

const W = 640, H = 480;
const still = { turnRadPerSec: 0, forwardIps: 0 };

function frame(color, forwardIn = 30) {
  const img = makeImage(W, H);
  const g = new Geometry(W, H, 11.7, 25.9);
  const p = g.project(forwardIn, 0);
  drawBall(img, p.u, p.v, p.radiusPx, BALL_RGB[color]);
  return img;
}

export default function (t) {
  // --- the real bug: blue goes dark before yellow does ----------------
  // On the robot, an underexposed frame read value 53 for blue against a
  // floor of 80, so blue vanished while yellow was still fine.
  {
    const dim = { ...defaultSensor(), brightness: 0.45, motionBlur: false };
    const blue = detect(applyTo(frame("blue"), dim));
    const yellow = detect(applyTo(frame("yellow"), dim));
    t.eq("underexposed: blue is lost", blue.balls.length, 0);
    t.eq("underexposed: yellow survives", yellow.balls.length, 1);
  }
  {
    const ok = { ...defaultSensor(), motionBlur: false };
    for (const c of ["red", "blue", "yellow"]) {
      t.eq(`correctly exposed: ${c} is found`, detect(applyTo(frame(c), ok)).balls.length, 1);
    }
  }

  // --- the brightness at which each colour drops out -------------------
  // Ordering is the point: blue first, then red, then yellow.
  {
    const floor = {};
    for (const c of ["red", "blue", "yellow"]) {
      for (let b = 1.0; b > 0.1; b -= 0.05) {
        const s = { ...defaultSensor(), brightness: b, motionBlur: false };
        if (detect(applyTo(frame(c), s)).balls.length === 0) { floor[c] = b; break; }
      }
    }
    t.ok(`blue dies first (blue ${floor.blue?.toFixed(2)}, red ${floor.red?.toFixed(2)})`,
         floor.blue > floor.red);
    t.ok(`red dies before yellow (red ${floor.red?.toFixed(2)}, yellow ${floor.yellow?.toFixed(2)})`,
         floor.red > floor.yellow);
  }

  // --- motion blur ------------------------------------------------------
  {
    const g = new Geometry(640, 480, 11.7, 25.9);
    const slow = blurPixels(defaultSensor(), { turnRadPerSec: 0.3 }, g.fx);
    const fast = blurPixels(defaultSensor(), { turnRadPerSec: 3.0 }, g.fx);
    t.ok(`a gentle turn barely smears (${slow.toFixed(1)} px)`, slow < 4);
    t.ok(`a fast spin smears a lot (${fast.toFixed(1)} px)`, fast > 20);

    // Measured, not assumed. Blur turns out to be far less destructive than
    // expected: a ball at 60 in survives up to about 4 rad/s and is only
    // lost past 6. A search sweep at 0.25 power is roughly 1 rad/s, so at
    // realistic FTC turn rates blur is not what breaks detection.
    const sensor = defaultSensor();
    t.eq("60 in ball survives a realistic search sweep (1 rad/s)",
         detect(applyTo(frame("yellow", 60), sensor, { turnRadPerSec: 1.0 })).balls.length, 1);
    t.eq("60 in ball survives 4 rad/s",
         detect(applyTo(frame("yellow", 60), sensor, { turnRadPerSec: 4.0 })).balls.length, 1);
    t.eq("60 in ball IS lost at an unrealistic 8 rad/s",
         detect(applyTo(frame("yellow", 60), sensor, { turnRadPerSec: 8.0 })).balls.length, 0);
    t.eq("a close ball survives even that",
         detect(applyTo(frame("yellow", 30), sensor, { turnRadPerSec: 8.0 })).balls.length, 1);
  }

  // --- noise must not conjure balls out of nothing ----------------------
  {
    const noisy = { ...defaultSensor(), noise: 25, motionBlur: false };
    const img = makeImage(W, H);
    applySensor(img, noisy, still, 600);
    t.eq("heavy noise on bare floor finds nothing", detect(img).balls.length, 0);
  }
}

function applyTo(img, sensor, motion = still) {
  applySensor(img, sensor, motion, 610.7);
  return img;
}
