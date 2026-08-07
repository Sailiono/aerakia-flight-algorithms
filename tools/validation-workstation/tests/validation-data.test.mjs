import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFile } from "node:fs/promises";
import test from "node:test";

const dataUrl = new URL("../public/data/validation-data.json", import.meta.url);

test("validation catalog declares provenance and evidence boundaries", async () => {
  const data = JSON.parse(await readFile(dataUrl, "utf8"));
  assert.equal(data.schemaVersion, 3);
  assert.match(data.algorithmCommit, /^[0-9a-f]{7,40}$|^unknown$/);
  assert.ok(data.datasets.length >= 8);
  assert.ok(data.datasets.some((item) => item.evidenceGrade === "A"));
  assert.ok(data.datasets.some((item) => item.evidenceGrade === "D"));
  assert.ok(data.datasets.some((item) => item.evidenceGrade === "E"));
  for (const item of data.datasets) {
    assert.match(item.sourcePath, /\.json$/);
    assert.match(item.sourceSha256, /^[0-9a-f]{64}$/);
    assert.ok(Array.isArray(item.limitations));
  }
});

test("curve data is finite, aligned, and timestamp-monotonic", async () => {
  const data = JSON.parse(await readFile(dataUrl, "utf8"));
  assert.equal(data.curves.length, 2);
  for (const curve of data.curves) {
    assert.ok(curve.samples > 0);
    assert.ok(curve.storedSamples > 0);
    assert.equal(curve.rows.length, curve.storedSamples);
    assert.ok(curve.storedSamples <= curve.samples);
    assert.ok(curve.storedSamples <= 4000);
    assert.ok(Math.abs(curve.rows[0].t) < 1e-9);
    assert.ok(Math.abs(curve.rows.at(-1).t - curve.duration) < 0.001);
    assert.ok(curve.samplePeriodMs > 0);
    assert.match(curve.beforeSource.csvSha256, /^[0-9a-f]{64}$/);
    assert.match(curve.afterSource.csvSha256, /^[0-9a-f]{64}$/);
    for (let index = 0; index < curve.rows.length; index += 1) {
      const row = curve.rows[index];
      assert.ok(Number.isFinite(row.t));
      assert.ok(row.roll.length === 3 && row.pitch.length === 3 && row.yaw.length === 3 && row.geo.length === 3);
      for (const value of [...row.roll, ...row.pitch, ...row.yaw, ...row.geo]) assert.ok(Number.isFinite(value));
      if (index > 0) assert.ok(row.t > curve.rows[index - 1].t);
    }
  }
});

test("barometer campaign entry matches its compact public evidence", async () => {
  const data = JSON.parse(await readFile(dataUrl, "utf8"));
  const entry = data.datasets.find((item) => item.id === "baro-outage-v1");
  assert.ok(entry);
  const source = await readFile(
    new URL("../../../validation/public/barometer_outage_campaign_v1.json", import.meta.url),
  );
  assert.equal(entry.sourceSha256, createHash("sha256").update(source).digest("hex"));
  assert.equal(entry.samples, 600);
  assert.equal(entry.evidenceGrade, "E");
});

test("G0 rate sensitivity entry matches its compact public evidence", async () => {
  const data = JSON.parse(await readFile(dataUrl, "utf8"));
  const entry = data.datasets.find((item) => item.id === "g0-rate-sensitivity");
  assert.ok(entry);
  const source = await readFile(
    new URL("../../../validation/public/g0_rate_sensitivity.json", import.meta.url),
  );
  assert.equal(entry.sourceSha256, createHash("sha256").update(source).digest("hex"));
  assert.equal(entry.samples, 12);
  assert.equal(entry.evidenceGrade, "E");
  assert.equal(entry.highlights.healthyRatio, 1);
});

test("G0 trajectory screen and gate characterization entries match evidence", async () => {
  const data = JSON.parse(await readFile(dataUrl, "utf8"));
  for (const [id, file, samples] of [
    ["g0-trajectory-screen", "g0_trajectory_screen.json", 4],
    ["g0-gate-characterization", "g0_gate_characterization.json", null],
  ]) {
    const entry = data.datasets.find((item) => item.id === id);
    assert.ok(entry);
    const source = await readFile(new URL(`../../../validation/public/${file}`, import.meta.url));
    const evidence = JSON.parse(source.toString("utf8"));
    assert.equal(entry.sourceSha256, createHash("sha256").update(source).digest("hex"));
    assert.equal(entry.evidenceGrade, "E");
    assert.equal(entry.samples, samples ?? evidence.static_null.total_cases);
  }
});
