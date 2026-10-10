// Tiny test runner. No dependencies, so it runs anywhere node does.
const results = [];
const t = {
  ok: (name, cond) => results.push([!!cond, name, ""]),
  eq: (name, got, want) =>
    results.push([got === want, name, `got ${got}, want ${want}`]),
  near: (name, got, want, tol) =>
    results.push([
      Math.abs(got - want) <= tol,
      name,
      `got ${Number(got).toFixed(3)}, want ${want} +/- ${tol}`,
    ]),
};

const suites = ["./geometry.test.js", "./render.test.js", "./detector.test.js", "./pursuit.test.js", "./robot.test.js"];
for (const s of suites) (await import(s)).default(t);

let failed = 0;
for (const [pass, name, detail] of results) {
  if (!pass) failed++;
  console.log(`${pass ? "  ok  " : "FAIL  "}${name}${pass ? "" : "   " + detail}`);
}
console.log(`\n${results.length - failed}/${results.length} passed`);
process.exit(failed ? 1 : 0);
