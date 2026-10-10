// Main loop: render what the camera would see, run the real detector over
// those pixels, feed the result to the real pursuit controller, drive the
// robot, repeat. Nothing here fakes a detection.

import { Geometry, CAMERAS, BALLS, setCamera, setBallDiameter } from "./geometry.js";
import { renderCameraView, BALL_RGB } from "./render.js";
import { detect, PROC_W } from "./detector.js";
import { BallPursuit, defaultParams } from "./pursuit.js";
import { Robot, defaultChassis } from "./robot.js";

const CAM_W = 640, CAM_H = 480;
const FIELD_IN = 144;                      // 12 ft, drawn on a 520 px canvas

const cam = document.getElementById("cam").getContext("2d", { willReadFrequently: true });
const maskCv = document.getElementById("mask");
const maskCtx = maskCv.getContext("2d");
const field = document.getElementById("field").getContext("2d");

const mount = { heightIn: 11.7, pitchDeg: 25.9 };
const params = defaultParams();
const chassis = defaultChassis();

let robot = new Robot(chassis, { x: -40, y: 0, headingRad: 0 });
let pursuit = new BallPursuit(params);
let balls = [
  { x: 20, y: 6, color: "yellow" },
  { x: 34, y: -18, color: "blue" },
  { x: 46, y: 20, color: "red" },
];
let tape = [{ from: [-60, -70], to: [120, -70], widthIn: 2 }];
let running = false, lastT = 0, last = null;

// ------------------------------------------------------------------ loop

function tick(now) {
  const dt = lastT ? Math.min((now - lastT) / 1000, 0.05) : 0.02;
  lastT = now;

  const geom = new Geometry(CAM_W, CAM_H, mount.heightIn, mount.pitchDeg);

  // Balls, as the camera sees them from where the robot currently is.
  const world = {
    balls: balls
      .map((b) => ({ ...robot.worldToCamera(b.x, b.y), color: b.color }))
      .filter((b) => b.forwardIn > 0),
    tape: tape.map((t) => ({
      from: Object.values(robot.worldToCamera(t.from[0], t.from[1])),
      to: Object.values(robot.worldToCamera(t.to[0], t.to[1])),
      widthIn: t.widthIn,
    })),
  };
  renderCameraView(cam, geom, world);

  const image = cam.getImageData(0, 0, CAM_W, CAM_H);
  const det = detect(image, {
    camHeightIn: mount.heightIn,
    camPitchDeg: mount.pitchDeg,
    wantedColor: document.getElementById("wanted").value || null,
    sizeGate: document.getElementById("gate").checked,
  });
  last = det;

  drawDetections(det);
  drawMask(det);

  if (running && document.getElementById("chase").checked) {
    const t = det.best
      ? { rangeIn: det.best.rangeIn, lateralIn: det.best.lateralIn, txDeg: det.best.txDeg }
      : null;
    const o = pursuit.update(t, dt);
    robot.step(o, dt);
    if (o.captured) {
      // Collect it: remove whatever is closest to the robot, then look again.
      const cm = robot.cameraPose();
      let bi = -1, bd = 1e9;
      balls.forEach((b, i) => {
        const d = Math.hypot(b.x - cm.x, b.y - cm.y);
        if (d < bd) { bd = d; bi = i; }
      });
      if (bi >= 0 && bd < 18) balls.splice(bi, 1);
      pursuit.reset();
    }
  }

  drawField();
  writeTelemetry(det);
  requestAnimationFrame(tick);
}

// --------------------------------------------------------------- drawing

function drawDetections(det) {
  const k = CAM_W / det.width;             // detector worked at PROC_W
  cam.lineWidth = 2;
  for (const b of det.balls) {
    cam.strokeStyle = b === det.best ? "#fff" : "rgba(255,255,255,.55)";
    cam.beginPath();
    cam.arc(b.u * k, b.v * k, b.r * k, 0, Math.PI * 2);
    cam.stroke();
    cam.fillStyle = "#fff";
    cam.font = "12px ui-monospace, monospace";
    cam.fillText(`${b.color[0].toUpperCase()} ${b.rangeIn.toFixed(0)}"`,
                 b.u * k - b.r * k, b.v * k - b.r * k - 5);
  }
  // crosshair
  cam.strokeStyle = "rgba(0,255,120,.9)";
  cam.beginPath();
  cam.moveTo(CAM_W / 2 - 10, CAM_H / 2); cam.lineTo(CAM_W / 2 + 10, CAM_H / 2);
  cam.moveTo(CAM_W / 2, CAM_H / 2 - 10); cam.lineTo(CAM_W / 2, CAM_H / 2 + 10);
  cam.stroke();
}

function drawMask(det) {
  maskCv.width = det.width; maskCv.height = det.height;
  const img = maskCtx.createImageData(det.width, det.height);
  const tint = { red: [255, 70, 70], blue: [90, 130, 255], yellow: [255, 220, 60] };
  for (const [color, m] of Object.entries(det.masks)) {
    const c = tint[color];
    for (let i = 0; i < m.length; i++) {
      if (!m[i]) continue;
      img.data[i * 4] = c[0]; img.data[i * 4 + 1] = c[1];
      img.data[i * 4 + 2] = c[2]; img.data[i * 4 + 3] = 255;
    }
  }
  for (let i = 0; i < det.width * det.height; i++) if (!img.data[i * 4 + 3]) img.data[i * 4 + 3] = 255;
  maskCtx.putImageData(img, 0, 0);
}

const toPx = (v) => (v / FIELD_IN + 0.5) * 520;

function drawField() {
  const g = field;
  g.clearRect(0, 0, 520, 520);
  g.strokeStyle = "rgba(128,128,128,.25)";
  for (let i = -72; i <= 72; i += 24) {
    g.beginPath(); g.moveTo(toPx(i), 0); g.lineTo(toPx(i), 520); g.stroke();
    g.beginPath(); g.moveTo(0, toPx(i)); g.lineTo(520, toPx(i)); g.stroke();
  }

  // the camera's field of view on the floor
  const cm = robot.cameraPose();
  const geom = new Geometry(CAM_W, CAM_H, mount.heightIn, mount.pitchDeg);
  const near = geom.nearestVisibleIn(), far = 100;
  const halfFov = (Math.atan2(CAM_W / 2, geom.fx) * 180) / Math.PI;
  g.fillStyle = "rgba(90,160,240,.13)";
  g.beginPath();
  for (const [d, dir] of [[near, 1], [far, -1]]) {
    for (let a = -halfFov; a <= halfFov; a += 2) {
      const ang = cm.headingRad + (a * Math.PI) / 180;
      const x = cm.x + d * Math.cos(ang), y = cm.y + d * Math.sin(ang);
      g.lineTo(toPx(x), toPx(-y));
    }
  }
  g.closePath(); g.fill();

  for (const b of balls) {
    g.fillStyle = `rgb(${BALL_RGB[b.color].join(",")})`;
    g.beginPath(); g.arc(toPx(b.x), toPx(-b.y), 6, 0, Math.PI * 2); g.fill();
  }

  // robot
  g.save();
  g.translate(toPx(robot.pose.x), toPx(-robot.pose.y));
  g.rotate(-robot.pose.headingRad);
  g.fillStyle = "rgba(120,130,145,.85)";
  const s = (18 / FIELD_IN) * 520;
  g.fillRect(-s / 2, -s / 2, s, s);
  g.fillStyle = "#fff";
  g.fillRect(s / 2 - 4, -2, 6, 4);
  g.restore();
}

function writeTelemetry(det) {
  const p = det.llpython;
  const names = ["valid", "colour", "tx deg", "ty deg", "range in", "lateral in", "radius px", "count"];
  const body = p.map((v, i) =>
    `<tr><td class="k">${names[i]}</td><td>${i === 0 || i === 1 || i === 7 ? v : v.toFixed(2)}</td></tr>`
  ).join("");
  document.querySelector("#out tbody").innerHTML =
    body + `<tr><td class="k">committed</td><td>${pursuit.isCommitted()}</td></tr>`;
}

// --------------------------------------------------------------- controls

function slider(host, label, obj, key, min, max, step, suffix = "") {
  const d = document.createElement("div");
  d.className = "slider";
  d.dataset.key = key;
  d.innerHTML = `<div class="lab"><span>${label}</span><b>${obj[key]}${suffix}</b></div>`;
  const r = document.createElement("input");
  r.type = "range"; r.min = min; r.max = max; r.step = step; r.value = obj[key];
  r.oninput = () => {
    obj[key] = parseFloat(r.value);
    d.querySelector("b").textContent = obj[key] + suffix;
    updateBlind();
  };
  d.appendChild(r);
  host.appendChild(d);
}

// Camera and ball are the two things that make this simulate SOMEONE ELSE'S
// robot rather than ours. Both change every distance, so they come first.
const camSel = document.getElementById("camera");
for (const [k, c] of Object.entries(CAMERAS)) {
  camSel.insertAdjacentHTML("beforeend", `<option value="${k}">${c.label}</option>`);
}
camSel.onchange = () => {
  const c = setCamera(camSel.value);
  document.getElementById("camnote").textContent = c.note;
  updateBlind();
};

const ballSel = document.getElementById("ball");
for (const [k, b] of Object.entries(BALLS)) {
  ballSel.insertAdjacentHTML("beforeend", `<option value="${b.diameterIn}">${b.label}</option>`);
}
ballSel.onchange = () => { setBallDiameter(parseFloat(ballSel.value)); updateBlind(); };

document.getElementById("camnote").textContent = CAMERAS[camSel.value].note;

const mountHost = document.getElementById("mount");
slider(mountHost, "lens height", mount, "heightIn", 4, 20, 0.1, " in");
slider(mountHost, "pitch below horizontal", mount, "pitchDeg", 0, 45, 0.1, " deg");

const gainHost = document.getElementById("gains");
for (const [k, lo, hi, st] of [
  ["kTurn", 0, 0.08, 0.001], ["kDrive", 0, 0.08, 0.001],
  ["kTurnMin", 0, 0.25, 0.01], ["kDriveMin", 0, 0.25, 0.01],
  ["blindApproachIn", 8, 30, 0.5], ["blindDriveSeconds", 0, 3, 0.05],
]) slider(gainHost, k, params, k, lo, hi, st);

function updateBlind() {
  const g = new Geometry(CAM_W, CAM_H, mount.heightIn, mount.pitchDeg);
  const b = g.nearestVisibleIn();
  const ok = params.blindApproachIn > b + 2;
  document.getElementById("blind").innerHTML =
    `Nearest visible ball <b>${b.toFixed(1)} in</b>. ` +
    `Commit at ${params.blindApproachIn.toFixed(1)} in — ` +
    (ok ? `<span class="good">clear of it</span>.`
        : `<span class="bad">too close; the ball vanishes as the run begins</span>.`);
  params.cameraBlindIn = b;
}

document.getElementById("run").onclick = (e) => {
  running = !running;
  e.target.textContent = running ? "Pause" : "Start";
  e.target.classList.toggle("primary", !running);
  lastT = 0;
};
document.getElementById("reset").onclick = () => {
  robot = new Robot(chassis, { x: -40, y: 0, headingRad: 0 });
  pursuit = new BallPursuit(params);
  balls = [
    { x: 20, y: 6, color: "yellow" },
    { x: 34, y: -18, color: "blue" },
    { x: 46, y: 20, color: "red" },
  ];
};

// ------------------------------------------------------------- demos
//
// Each of these reproduces something that actually went wrong on the real
// robot, which is more use to a newcomer than a list of sliders.

const DEMOS = [
  {
    name: "Normal chase",
    why: "Three balls on an open floor. It should collect all three.",
    apply: () => {
      balls = [
        { x: 20, y: 6, color: "yellow" },
        { x: 34, y: -18, color: "blue" },
        { x: 46, y: 20, color: "red" },
      ];
      tape = [{ from: [-60, -70], to: [120, -70], widthIn: 2 }];
      params.kDriveMin = 0.10;
      params.blindApproachIn = 14;
    },
  },
  {
    name: "Stall bug",
    why: "kDriveMin is 0. The proportional term dies under friction and the " +
         "robot halts about ten inches short, with no way to notice. This is " +
         "the bug the breakaway minimums fix.",
    apply: () => {
      balls = [{ x: 24, y: 0, color: "yellow" }];
      params.kDriveMin = 0;
      params.blindApproachIn = 0;      // no commit, so only the P term drives
    },
  },
  {
    name: "Commit too late",
    why: "Commit distance is inside the blind zone, so the ball vanishes at " +
         "the same instant the final run should begin. The robot never learns " +
         "it got close. Watch the Mounting warning turn amber.",
    apply: () => {
      balls = [{ x: 30, y: 0, color: "yellow" }];
      params.kDriveMin = 0.10;
      params.blindApproachIn = 9;
    },
  },
  {
    name: "Ball on tape",
    why: "A blue ball sitting on blue tape: the hardest case for the " +
         "detector, and one that was never testable on the robot. The ring " +
         "test should still find it.",
    apply: () => {
      balls = [{ x: 30, y: 0, color: "blue" }];
      tape = [{ from: [-20, -3], to: [120, -3], widthIn: 2 }];
      params.kDriveMin = 0.10;
      params.blindApproachIn = 14;
    },
  },
  {
    name: "Crowded row",
    why: "Seven touching balls. A plain colour-blob finder merges these into " +
         "one shape with a meaningless centre; the distance transform splits " +
         "them.",
    apply: () => {
      balls = [];
      for (let i = -3; i <= 3; i++) {
        balls.push({ x: 42, y: i * 4, color: i % 2 ? "yellow" : "blue" });
      }
      tape = [];
      params.kDriveMin = 0.10;
      params.blindApproachIn = 14;
    },
  },
];

const demoHost = document.getElementById("demos");
for (const d of DEMOS) {
  const b = document.createElement("button");
  b.textContent = d.name;
  b.onclick = () => {
    d.apply();
    robot = new Robot(chassis, { x: -40, y: 0, headingRad: 0 });
    pursuit = new BallPursuit(params);
    document.getElementById("demonote").textContent = d.why;
    syncGainSliders();
    updateBlind();
  };
  demoHost.appendChild(b);
}

/** Demos change gains behind the sliders' backs; put the sliders right. */
function syncGainSliders() {
  for (const el of document.querySelectorAll("#gains .slider")) {
    const key = el.dataset.key;
    if (!key) continue;
    el.querySelector("input").value = params[key];
    el.querySelector("b").textContent = params[key];
  }
}

document.getElementById("field").addEventListener("click", (e) => {
  const r = e.target.getBoundingClientRect();
  const x = ((e.clientX - r.left) / r.width - 0.5) * FIELD_IN;
  const y = -(((e.clientY - r.top) / r.height - 0.5) * FIELD_IN);
  if (e.shiftKey) {
    const cs = ["yellow", "blue", "red"];
    balls.push({ x, y, color: cs[balls.length % 3] });
  } else if (balls.length) {
    let bi = 0, bd = 1e9;
    balls.forEach((b, i) => {
      const d = Math.hypot(b.x - x, b.y - y);
      if (d < bd) { bd = d; bi = i; }
    });
    balls[bi].x = x; balls[bi].y = y;
  }
});

updateBlind();
requestAnimationFrame(tick);
