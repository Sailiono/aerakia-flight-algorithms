import fs from "node:fs/promises";
import path from "node:path";

const repoRoot = "/home/clark/DevFiles/OneDev/aerakia-flight-algorithms";
const output = new URL("../public/data/curve-data.json", import.meta.url);

const pairs = [
  {
    id: "v103",
    label: "V1_03 difficult · raw IMU -> bias corrected",
    dataset: "EuRoC MAV V1_03_difficult",
    before: "build/public-dataset-suite-final/V1_03_difficult__raw_imu_trusted_attitude_init/results.csv",
    after: "build/public-dataset-suite-final/V1_03_difficult__reference_bias_corrected_trusted_attitude_init/results.csv",
  },
  {
    id: "mh01",
    label: "MH_01 easy · raw IMU -> bias corrected",
    dataset: "EuRoC MAV MH_01_easy",
    before: "build/public-dataset-suite-final/MH_01_easy__raw_imu_trusted_attitude_init/results.csv",
    after: "build/public-dataset-suite-final/MH_01_easy__reference_bias_corrected_trusted_attitude_init/results.csv",
  },
];

async function readRows(relativePath) {
  const source = await fs.readFile(path.join(repoRoot, relativePath), "utf8");
  const [header, ...lines] = source.trim().split("\n");
  const index = Object.fromEntries(header.split(",").map((name, i) => [name, i]));
  return lines.map((line) => {
    const values = line.split(",");
    return {
      t: Number(values[index.ts_us]) / 1_000_000,
      roll: Number(values[index.eskf_roll_deg]),
      pitch: Number(values[index.eskf_pitch_deg]),
      yaw: Number(values[index.eskf_yaw_deg]),
      truthRoll: Number(values[index.truth_roll_deg]),
      truthPitch: Number(values[index.truth_pitch_deg]),
      truthYaw: Number(values[index.truth_yaw_deg]),
      truthQ: ["w", "x", "y", "z"].map((axis) => Number(values[index[`truth_q_${axis}`]])),
      eskfQ: ["w", "x", "y", "z"].map((axis) => Number(values[index[`eskf_q_${axis}`]])),
    };
  });
}

function wrapAngleDeg(value) {
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function circularRmse(rows, estimate, truth) {
  const sum = rows.reduce((total, row) => total + wrapAngleDeg(row[estimate] - row[truth]) ** 2, 0);
  return Math.sqrt(sum / rows.length);
}

function normalizeQuaternion(q) {
  const norm = Math.hypot(...q);
  return norm > 0 ? q.map((value) => value / norm) : [1, 0, 0, 0];
}

function geodesicErrorDeg(estimate, truth) {
  const normalizedTruth = normalizeQuaternion(truth);
  const normalizedEstimate = normalizeQuaternion(estimate);
  const dot = Math.min(1, Math.max(-1, Math.abs(normalizedTruth.reduce((acc, value, i) => acc + value * normalizedEstimate[i], 0))));
  return (2 * Math.acos(dot) * 180) / Math.PI;
}

function geodesicRmse(rows) {
  const sum = rows.reduce((total, row) => {
    return total + geodesicErrorDeg(row.eskfQ, row.truthQ) ** 2;
  }, 0);
  return Math.sqrt(sum / rows.length);
}

const curves = [];
for (const pair of pairs) {
  const [before, after] = await Promise.all([readRows(pair.before), readRows(pair.after)]);
  const rows = before.map((row, index) => ({
    t: Number(row.t.toFixed(3)),
    roll: [row.truthRoll, row.roll, after[index].roll].map((value) => Number(value.toFixed(5))),
    pitch: [row.truthPitch, row.pitch, after[index].pitch].map((value) => Number(value.toFixed(5))),
    yaw: [row.truthYaw, row.yaw, after[index].yaw].map((value) => Number(value.toFixed(5))),
    geo: [0, geodesicErrorDeg(row.eskfQ, row.truthQ), geodesicErrorDeg(after[index].eskfQ, row.truthQ)].map((value) => Number(value.toFixed(5))),
  }));
  const axisRmse = Object.fromEntries(
    [
      ["roll", "truthRoll"],
      ["pitch", "truthPitch"],
      ["yaw", "truthYaw"],
    ].map(([axis, truth]) => {
      const beforeValue = circularRmse(before, axis, truth);
      const afterValue = circularRmse(after, axis, truth);
      return [axis, {
        before: Number(beforeValue.toFixed(4)),
        after: Number(afterValue.toFixed(4)),
        reductionPct: Number((((beforeValue - afterValue) / beforeValue) * 100).toFixed(2)),
      }];
    }),
  );
  const beforeEulerRmse = Math.sqrt(Object.values(axisRmse).reduce((sum, metric) => sum + metric.before ** 2, 0) / 3);
  const afterEulerRmse = Math.sqrt(Object.values(axisRmse).reduce((sum, metric) => sum + metric.after ** 2, 0) / 3);
  const geodesicBefore = geodesicRmse(before);
  const geodesicAfter = geodesicRmse(after);
  curves.push({
    ...pair,
    duration: Number(rows.at(-1).t.toFixed(3)),
    samplePeriodMs: 5,
    samples: rows.length,
    axisRmse,
    attitudeEulerRmse: {
      before: Number(beforeEulerRmse.toFixed(4)),
      after: Number(afterEulerRmse.toFixed(4)),
      reductionPct: Number((((beforeEulerRmse - afterEulerRmse) / beforeEulerRmse) * 100).toFixed(2)),
    },
    attitudeGeodesicRmse: {
      before: Number(geodesicBefore.toFixed(4)),
      after: Number(geodesicAfter.toFixed(4)),
      reductionPct: Number((((geodesicBefore - geodesicAfter) / geodesicBefore) * 100).toFixed(2)),
    },
    rows,
  });
}

await fs.mkdir(new URL("../public/data/", import.meta.url), { recursive: true });
await fs.writeFile(output, JSON.stringify({ generatedAt: new Date().toISOString(), curves }));
