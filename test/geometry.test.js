// Pins the JS geometry to numbers measured on the real camera and robot.
// If these drift, the simulator has stopped predicting the robot.
import { Geometry } from "../src/geometry.js";

export default function (t) {
  // Calibrated mount: lens 11.7 in up, 25.9 deg down, detector works at 320x240.
  const g = new Geometry(320, 240, 11.7, 25.9);

  // Two real detections, with the ball at a tape-measured distance.
  // Peak pixels and truths taken from the on-robot calibration session.
  t.near("ball at a measured 24 in", g.solve(190, 111).rangeIn, 24.0, 0.6);
  t.near("ball at a measured 48 in", g.solve(149, 50).rangeIn, 48.0, 1.0);

  // The size model agreed with the measured blob radius to 0.1 px.
  t.near("predicted radius at the 24 in row", g.expectedRadiusPx(111), 17.4, 0.4);

  // Above the horizon there is no floor intersection. Note this cannot
  // happen at 25.9 deg: the horizon sits about 23 px ABOVE the top of the
  // frame, so every pixel hits the floor and far-field junk is rejected by
  // MAX_RANGE_IN rather than by the horizon. Tilt back up to exercise it.
  const level = new Geometry(320, 240, 11.7, 5.0);
  t.eq("ray above the horizon has no solution", level.solve(160, 50), null);
  t.ok("ray below the horizon does", level.solve(160, 200) !== null);
  t.ok("at 25.9 deg the horizon is off the top of the frame",
       g.solve(160, 0) !== null);

  // Blind radius drives blindApproachIn in the pursuit controller.
  t.near("nearest visible ball", g.nearestVisibleIn(), 9.9, 0.2);

  // project() must invert solve().
  const s = g.solve(190, 111);
  const p = g.project(s.forwardIn, s.lateralIn);
  t.near("project inverts solve (u)", p.u, 190, 0.01);
  t.near("project inverts solve (v)", p.v, 111, 0.01);
  t.near("projected radius matches prediction", p.radiusPx, g.expectedRadiusPx(111), 0.6);

  // Bearing sign: a ball right of centre must read positive tx.
  t.ok("tx positive to the right", g.angles(250, 150).txDeg > 0);
  t.ok("tx negative to the left", g.angles(60, 150).txDeg < 0);
}
