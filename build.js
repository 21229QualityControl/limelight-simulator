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

import { readFileSync, writeFileSync, mkdirSync, readdirSync } from "node:fs";

const ORDER = [
  "src/geometry.js",
  "src/thresholds.js",
  "src/render.js",
  "src/realism.js",
  "src/detector.js",
  "src/pursuit.js",
  "src/robot.js",
  "src/latency.js",
  "src/sim.js",
];

// Forgetting to list a new module here produced a bundle that threw
// "X is not defined" only at runtime, in the browser, with the page
// half-built. Fail loudly at build time instead.
const onDisk = readdirSync("src").filter((f) => f.endsWith(".js")).map((f) => "src/" + f);
const missing = onDisk.filter((f) => !ORDER.includes(f));
if (missing.length) {
  console.error("build.js: these modules exist but are not in ORDER:\n  " +
                missing.join("\n  ") + "\nAdd them in dependency order.");
  process.exit(1);
}

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
