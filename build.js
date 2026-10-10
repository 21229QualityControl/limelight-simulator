// Bundles everything into one self-contained HTML file.
//
// The point is that a browser refuses to load ES modules over file://, so
// index.html does not work by double-clicking it -- you need a web server.
// That is a silly thing to ask of someone who just wants to look at a
// simulator. dist/limelight-simulator.html has no such requirement: one file,
// no server, no install, works offline, and can be emailed.
//
//     node build.js
//
// The module graph here is small and acyclic, so inlining in dependency
// order and stripping import/export is enough; no real bundler needed.

import { readFileSync, writeFileSync, mkdirSync } from "node:fs";

const ORDER = [
  "src/geometry.js",
  "src/thresholds.js",
  "src/render.js",
  "src/detector.js",
  "src/pursuit.js",
  "src/robot.js",
  "src/sim.js",
];

const strip = (src, file) =>
  "\n// ===== " + file + " " + "=".repeat(Math.max(0, 60 - file.length)) + "\n" +
  src
    .replace(/^\s*import\s[\s\S]*?from\s*["'][^"']+["'];?\s*$/gm, "")
    .replace(/^\s*export\s+default\s+/gm, "")
    .replace(/^(\s*)export\s+/gm, "$1");

const code = ORDER.map((f) => strip(readFileSync(f, "utf8"), f)).join("\n");
const css = readFileSync("style.css", "utf8");
const html = readFileSync("index.html", "utf8");

const out = html
  .replace('<link rel="stylesheet" href="style.css">', `<style>\n${css}\n</style>`)
  .replace(
    '<script type="module" src="src/sim.js"></script>',
    `<script type="module">\n${code}\n</script>`
  );

mkdirSync("dist", { recursive: true });
writeFileSync("dist/limelight-simulator.html", out);
const kb = (out.length / 1024).toFixed(0);
console.log(`dist/limelight-simulator.html  ${kb} KB  (open it directly, no server)`);
