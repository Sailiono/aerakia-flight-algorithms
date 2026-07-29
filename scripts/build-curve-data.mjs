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
    };
  });
}

function rmse(rows, fields) {
  const values = rows.map((row) =>
    fields.reduce((sum, [estimate, truth]) => sum + (row[estimate] - row[truth]) ** 2, 0),
  );
  return Math.sqrt(values.reduce((sum, value) => sum + value, 0) / values.length / fields.length);
}

const curves = [];
for (const pair of pairs) {
  const [before, after] = await Promise.all([readRows(pair.before), readRows(pair.after)]);
  const rows = before.map((row, index) => ({
    t: Number(row.t.toFixed(3)),
    roll: [row.truthRoll, row.roll, after[index].roll].map((value) => Number(value.toFixed(5))),
    pitch: [row.truthPitch, row.pitch, after[index].pitch].map((value) => Number(value.toFixed(5))),
    yaw: [row.truthYaw, row.yaw, after[index].yaw].map((value) => Number(value.toFixed(5))),
  }));
  const beforeRmse = rmse(before, [
    ["roll", "truthRoll"],
    ["pitch", "truthPitch"],
    ["yaw", "truthYaw"],
  ]);
  const afterRmse = rmse(after, [
    ["roll", "truthRoll"],
    ["pitch", "truthPitch"],
    ["yaw", "truthYaw"],
  ]);
  curves.push({
    ...pair,
    duration: Number(rows.at(-1).t.toFixed(3)),
    samplePeriodMs: 5,
    samples: rows.length,
    attitudeEulerRmse: {
      before: Number(beforeRmse.toFixed(4)),
      after: Number(afterRmse.toFixed(4)),
      reductionPct: Number((((beforeRmse - afterRmse) / beforeRmse) * 100).toFixed(2)),
    },
    rows,
  });
}

await fs.mkdir(new URL("../public/data/", import.meta.url), { recursive: true });
await fs.writeFile(output, JSON.stringify({ generatedAt: new Date().toISOString(), curves }));
