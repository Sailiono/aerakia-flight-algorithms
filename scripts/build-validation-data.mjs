import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { createHash } from "node:crypto";

const scriptDir = path.dirname(new URL(import.meta.url).pathname);
const defaultRepoRoot = path.resolve(scriptDir, "../../..");
const repoRoot = path.resolve(process.env.AERAKIA_REPO_ROOT ?? defaultRepoRoot);
const outputPath = path.resolve(scriptDir, "../public/data/validation-data.json");

const curvePairs = [
  {
    id: "mh01",
    label: "MH_01 easy · raw IMU -> reference-bias diagnostic",
    dataset: "EuRoC MAV / MH_01_easy",
    before: "build/public-dataset-suite-final/MH_01_easy__raw_imu_trusted_attitude_init",
    after: "build/public-dataset-suite-final/MH_01_easy__reference_bias_corrected_trusted_attitude_init",
    referenceKind: "independent_truth",
    evidenceGrade: "A",
  },
  {
    id: "v103",
    label: "V1_03 difficult · raw IMU -> reference-bias diagnostic",
    dataset: "EuRoC MAV / V1_03_difficult",
    before: "build/public-dataset-suite-final/V1_03_difficult__raw_imu_trusted_attitude_init",
    after: "build/public-dataset-suite-final/V1_03_difficult__reference_bias_corrected_trusted_attitude_init",
    referenceKind: "independent_truth",
    evidenceGrade: "A",
  },
];

const catalogSpecs = [
  { id: "euroc-mh01", label: "EuRoC · MH_01_easy", grade: "A", kind: "independent_truth", file: "validation/public/euroc_mh01_summary.json", role: "independent attitude/position truth" },
  { id: "euroc-v103", label: "EuRoC · V1_03_difficult", grade: "A", kind: "independent_truth", file: "validation/public/euroc_v103_summary.json", role: "independent Vicon attitude truth" },
  { id: "blackbird-nyc", label: "Blackbird · NYC Subway Winter", grade: "A", kind: "independent_truth", file: "validation/public/blackbird_nyc_subway_winter_summary.json", role: "aggressive motion-capture stress" },
  { id: "urbannav-hk", label: "UrbanNav · HK Medium Urban 1", grade: "B", kind: "external_reference", file: "validation/public/urbannav_hk_medium_urban_1_summary.json", role: "recorded GNSS outage and SPAN reference" },
  { id: "uav-electrical", label: "Electrical survey UAV · voo_3", grade: "B", kind: "external_reference_with_shared_onboard_attitude", file: "validation/public/uav_electrical_voo3_summary.json", role: "RTK navigation reference" },
  { id: "insane-indoor", label: "INSANE · indoor_1", grade: "A-candidate", kind: "physical_heading_local_datum", file: "validation/public/insane_indoor_heading_study.json", role: "adverse physical magnetometer study" },
  { id: "insane-outdoor", label: "INSANE · outdoor_1_sensors", grade: "C", kind: "shared_physical_source", file: "validation/public/insane_outdoor1_heading_summary.json", role: "dual-RTK heading input, shared reference" },
  { id: "idf-ds", label: "IDF-DS · PX4 fixed-wing corpus", grade: "D", kind: "onboard_estimator_reference", file: "validation/public/idf_ds_summary.json", role: "large PX4 compatibility and stress corpus" },
];

function relative(file) {
  return path.relative(repoRoot, file).split(path.sep).join("/");
}

function sha256(buffer) {
  return createHash("sha256").update(buffer).digest("hex");
}

async function readJson(relativePath) {
  const file = path.join(repoRoot, relativePath);
  const buffer = await fs.readFile(file);
  return { value: JSON.parse(buffer.toString("utf8")), sha256: sha256(buffer) };
}

async function readGitCommit() {
  let gitDir = path.join(repoRoot, ".git");
  try {
    const marker = await fs.readFile(gitDir, "utf8");
    const match = marker.match(/^gitdir:\s*(.+)\s*$/m);
    if (match) gitDir = path.resolve(repoRoot, match[1]);
  } catch {
    // Normal repositories use a .git directory; worktrees use the marker above.
  }
  try {
    const head = (await fs.readFile(path.join(gitDir, "HEAD"), "utf8")).trim();
    if (!head.startsWith("ref: ")) return head;
    const ref = head.slice(5);
    try { return (await fs.readFile(path.join(gitDir, ref), "utf8")).trim(); } catch { return "unknown"; }
  } catch {
    return "unknown";
  }
}

function parseCsvLine(line) {
  // Validation outputs are numeric CSV without quoted commas. Rejecting malformed
  // rows is preferable to silently shifting a metric column.
  const values = line.split(",");
  if (values.some((value) => value.includes('"'))) throw new Error("quoted CSV fields are not supported");
  return values;
}

function finite(value, label) {
  const number = Number(value);
  if (!Number.isFinite(number)) throw new Error(`${label} is not finite: ${value}`);
  return number;
}

async function readResults(relativeDir) {
  const dir = path.join(repoRoot, relativeDir);
  const [csvBuffer, metrics, source] = await Promise.all([
    fs.readFile(path.join(dir, "results.csv")),
    readJson(path.join(relativeDir, "report/metrics.json")),
    readJson(path.join(relativeDir, "source.json")),
  ]);
  const lines = csvBuffer.toString("utf8").trim().split(/\r?\n/).filter(Boolean);
  if (lines.length < 2) throw new Error(`${relativeDir}/results.csv has no data rows`);
  const header = parseCsvLine(lines[0]);
  const index = Object.fromEntries(header.map((name, i) => [name, i]));
  const required = [
    "ts_us", "truth_roll_deg", "truth_pitch_deg", "truth_yaw_deg",
    "eskf_roll_deg", "eskf_pitch_deg", "eskf_yaw_deg",
    "truth_q_w", "truth_q_x", "truth_q_y", "truth_q_z",
    "eskf_q_w", "eskf_q_x", "eskf_q_y", "eskf_q_z",
  ];
  for (const name of required) if (index[name] === undefined) throw new Error(`${relativeDir}/results.csv missing ${name}`);
  const rows = lines.slice(1).map((line, rowIndex) => {
    const values = parseCsvLine(line);
    if (values.length !== header.length) throw new Error(`${relativeDir}/results.csv row ${rowIndex + 2} has ${values.length} columns, expected ${header.length}`);
    const get = (name) => finite(values[index[name]], `${relativeDir} row ${rowIndex + 2} ${name}`);
    return {
      t: get("ts_us") / 1_000_000,
      roll: get("eskf_roll_deg"),
      pitch: get("eskf_pitch_deg"),
      yaw: get("eskf_yaw_deg"),
      truthRoll: get("truth_roll_deg"),
      truthPitch: get("truth_pitch_deg"),
      truthYaw: get("truth_yaw_deg"),
      truthQ: ["w", "x", "y", "z"].map((axis) => get(`truth_q_${axis}`)),
      eskfQ: ["w", "x", "y", "z"].map((axis) => get(`eskf_q_${axis}`)),
    };
  });
  for (let i = 1; i < rows.length; i += 1) {
    if (!(rows[i].t > rows[i - 1].t)) throw new Error(`${relativeDir}/results.csv timestamps are not strictly increasing at row ${i + 2}`);
  }
  return {
    rows,
    metrics: metrics.value,
    metricsSha256: metrics.sha256,
    source: source.value,
    sourceSha256: source.sha256,
    csvSha256: sha256(csvBuffer),
    relativeDir,
  };
}

function wrapAngleDeg(value) {
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function normalizeQuaternion(q) {
  const norm = Math.hypot(...q);
  if (!(norm > 0)) throw new Error("zero-norm quaternion in results");
  return q.map((value) => value / norm);
}

function geodesicErrorDeg(estimate, truth) {
  const normalizedTruth = normalizeQuaternion(truth);
  const normalizedEstimate = normalizeQuaternion(estimate);
  const dot = Math.min(1, Math.max(-1, Math.abs(normalizedTruth.reduce((sum, value, i) => sum + value * normalizedEstimate[i], 0))));
  return (2 * Math.acos(dot) * 180) / Math.PI;
}

function rmse(rows, estimate, truth) {
  return Math.sqrt(rows.reduce((sum, row) => sum + wrapAngleDeg(row[estimate] - row[truth]) ** 2, 0) / rows.length);
}

function geodesicRmse(rows) {
  return Math.sqrt(rows.reduce((sum, row) => sum + geodesicErrorDeg(row.eskfQ, row.truthQ) ** 2, 0) / rows.length);
}

function round(value, digits = 4) {
  return Number(value.toFixed(digits));
}

function curveFromPair(pair, before, after) {
  if (before.rows.length !== after.rows.length) throw new Error(`${pair.id}: before/after sample counts differ`);
  const rows = before.rows.map((row, index) => {
    const corrected = after.rows[index];
    if (Math.abs(row.t - corrected.t) > 1e-9) throw new Error(`${pair.id}: before/after timestamps differ at sample ${index}`);
    return {
      t: round(row.t, 3),
      roll: [row.truthRoll, row.roll, corrected.roll].map((value) => round(value, 5)),
      pitch: [row.truthPitch, row.pitch, corrected.pitch].map((value) => round(value, 5)),
      yaw: [row.truthYaw, row.yaw, corrected.yaw].map((value) => round(value, 5)),
      geo: [0, geodesicErrorDeg(row.eskfQ, row.truthQ), geodesicErrorDeg(corrected.eskfQ, row.truthQ)].map((value) => round(value, 5)),
    };
  });
  const axes = ["roll", "pitch", "yaw"];
  const axisRmse = Object.fromEntries(axes.map((axis) => {
    const truth = `truth${axis[0].toUpperCase()}${axis.slice(1)}`;
    const beforeValue = rmse(before.rows, axis, truth);
    const afterValue = rmse(after.rows, axis, truth);
    return [axis, { before: round(beforeValue), after: round(afterValue), reductionPct: round(((beforeValue - afterValue) / beforeValue) * 100, 2) }];
  }));
  const beforeGeo = geodesicRmse(before.rows);
  const afterGeo = Math.sqrt(after.rows.reduce((sum, row, i) => sum + geodesicErrorDeg(row.eskfQ, before.rows[i].truthQ) ** 2, 0) / after.rows.length);
  const dt = rows.slice(1).map((row, i) => row.t - rows[i].t).sort((a, b) => a - b);
  return {
    id: pair.id,
    label: pair.label,
    dataset: pair.dataset,
    referenceKind: pair.referenceKind,
    evidenceGrade: pair.evidenceGrade,
    beforeSource: { path: before.relativeDir, csvSha256: before.csvSha256, metricsSha256: before.metricsSha256, sourceSha256: before.sourceSha256 },
    afterSource: { path: after.relativeDir, csvSha256: after.csvSha256, metricsSha256: after.metricsSha256, sourceSha256: after.sourceSha256 },
    duration: round(rows.at(-1).t, 3),
    samplePeriodMs: round((dt[Math.floor(dt.length / 2)] ?? 0) * 1000, 4),
    samples: rows.length,
    axisRmse,
    attitudeGeodesicRmse: { before: round(beforeGeo), after: round(afterGeo), reductionPct: round(((beforeGeo - afterGeo) / beforeGeo) * 100, 2) },
    integrity: {
      healthyRatio: before.metrics.eskf_integrity?.healthy_ratio ?? null,
      navigationRecoveries: before.metrics.eskf_integrity?.navigation_recoveries ?? null,
      positionNisMean: before.metrics.eskf_consistency?.position_nis?.mean ?? null,
      velocityNisMean: before.metrics.eskf_consistency?.velocity_nis?.mean ?? null,
      navigationNeesMean: before.metrics.eskf_consistency?.navigation_nees?.mean ?? null,
    },
    rows,
  };
}

function pick(value, paths) {
  for (const parts of paths) {
    let cursor = value;
    for (const part of parts.split(".")) cursor = cursor?.[part];
    if (cursor !== undefined && cursor !== null) return cursor;
  }
  return null;
}

function compactCatalogEntry(spec, value, sourceSha256) {
  return {
    id: spec.id,
    label: spec.label,
    evidenceGrade: spec.grade,
    referenceKind: spec.kind,
    role: spec.role,
    sourcePath: spec.file,
    sourceSha256,
    dataset: value.dataset ?? null,
    sequence: value.sequence ?? value.source?.sequence ?? null,
    samples: pick(value, ["samples", "recorded_imu_samples", "frozen_holdout_audit.imu_samples", "intake.dji_imu_samples", "corpus.imu_samples"]),
    durationS: pick(value, ["duration_s", "frozen_holdout_audit.duration_s", "replay.duration_s", "corpus.duration_s"]),
    coverage: {
      imuSamples: pick(value, ["samples", "recorded_imu_samples", "frozen_holdout_audit.imu_samples", "intake.dji_imu_samples", "corpus.imu_samples"]),
      headingUpdates: pick(value, ["recorded_dual_rtk_heading_updates", "frozen_holdout_audit.physical_magnetometer_updates", "paired_magnetometer_ab.physical_magnetometer_updates", "intake.rtk_velocity_samples"]),
      gnssUpdates: pick(value, ["synthetic_gnss.updates", "recorded_sensor_coverage.nmea_position_updates", "intake.dji_gps_position_samples", "corpus.gps_samples"]),
      outageS: pick(value, ["recorded_sensor_coverage.maximum_position_update_interval_s"]),
    },
    highlights: {
      eskfAttitudeRmseDeg: pick(value, ["tracks.raw_imu_trusted_attitude_init.eskf_attitude_rmse_deg", "tracks.static_reference_bias_corrected_trusted_attitude_init.eskf_attitude_rmse_deg", "tracks.recorded_position_reference_attitude_init.eskf_attitude_rmse_deg", "replay.eskf_geodesic_rmse_deg_vs_onboard_attitude", "frozen_holdout_tracking.algorithms.eskf.geodesic_rmse_deg"]),
      eskfYawRmseDeg: pick(value, ["tracks.raw_imu_trusted_attitude_init.eskf_yaw_rmse_deg", "tracks.static_reference_bias_corrected_trusted_attitude_init.eskf_yaw_rmse_deg", "tracks.recorded_position_reference_attitude_init.eskf_yaw_rmse_deg", "replay.eskf_yaw_rmse_deg_vs_onboard_attitude", "frozen_holdout_tracking.algorithms.eskf.yaw_rmse_deg", "cold_start.post_alignment_yaw_rmse_deg"]),
      tiltRmseDeg: pick(value, ["tracks.raw_imu_trusted_attitude_init.eskf_tilt_rmse_deg", "tracks.static_reference_bias_corrected_trusted_attitude_init.eskf_tilt_rmse_deg", "tracks.recorded_position_reference_attitude_init.eskf_tilt_rmse_deg", "replay.eskf_tilt_rmse_deg_vs_onboard_attitude", "frozen_holdout_tracking.algorithms.eskf.tilt_rmse_deg"]),
      positionRmseM: pick(value, ["tracks.recorded_position_reference_attitude_init.aided_position_rmse_m", "tracks.raw_imu_trusted_attitude_init.position_rmse_m", "replay.position_rmse_m_vs_rtk", "selected_replays.0.position_rmse_m"]),
      outagePeakErrorM: pick(value, ["tracks.recorded_position_reference_attitude_init.outage_peak_position_error_m"]),
      healthyRatio: pick(value, ["tracks.raw_imu_trusted_attitude_init.healthy_ratio", "tracks.recorded_position_reference_attitude_init.healthy_ratio", "replay.healthy_ratio", "frozen_holdout_tracking.algorithms.eskf.healthy_ratio"]),
      magnetometerOnYawRmseDeg: pick(value, ["paired_magnetometer_ab.magnetometer_on.eskf_yaw_rmse_deg"]),
      magnetometerOffYawRmseDeg: pick(value, ["paired_magnetometer_ab.magnetometer_off.eskf_yaw_rmse_deg"]),
      navigationNeesMean: pick(value, ["tracks.raw_imu_trusted_attitude_init.navigation_nees_mean", "tracks.recorded_position_reference_attitude_init.navigation_nees_mean", "frozen_holdout_tracking.algorithms.eskf.navigation_nees_mean"]),
    },
    limitations: value.limitations ?? value.claim?.not_claimed ?? [],
  };
}

export async function buildValidationData() {
  const curves = [];
  for (const pair of curvePairs) curves.push(curveFromPair(pair, await readResults(pair.before), await readResults(pair.after)));
  const datasets = [];
  for (const spec of catalogSpecs) {
    try {
      const source = await readJson(spec.file);
      datasets.push(compactCatalogEntry(spec, source.value, source.sha256));
    } catch (error) {
      if (error.code === "ENOENT") continue;
      throw error;
    }
  }
  const optional = [
    ["g0-monte-carlo", "build/g0-monte-carlo-clean-40000-v2/summary.json", "E", "synthetic_truth", "40,000-seed campaign · selected 1,000 trials", "synthetic navigation-outage consistency"],
    ["input-integrity", "build/g0-final-host-report/input-integrity-summary.json", "E", "synthetic_fault_campaign", "1.02M input-integrity attempts", "transport and numeric fault coverage"],
  ];
  for (const [id, file, grade, kind, label, role] of optional) {
    try {
      const source = await readJson(file);
      const value = source.value;
      datasets.push({
        id, label, evidenceGrade: grade, referenceKind: kind, role, sourcePath: file, sourceSha256: source.sha256,
        dataset: value.scope?.scenario ?? "host campaign", sequence: null,
        samples: value.scope?.completed_trials ?? value.attempted ?? null,
        durationS: value.scope?.duration_s ?? null,
        coverage: { imuSamples: value.attempted ?? null, headingUpdates: null, gnssUpdates: null, outageS: value.scope?.outage_duration_s ?? null },
        highlights: {
          eskfAttitudeRmseDeg: value.aggregates?.attitude_rmse_deg?.mean ?? null,
          eskfYawRmseDeg: null, tiltRmseDeg: null,
          positionRmseM: value.aggregates?.position_rmse_m?.mean ?? null,
          outagePeakErrorM: null, healthyRatio: value.aggregates?.healthy_ratio?.mean ?? null,
          magnetometerOnYawRmseDeg: null, magnetometerOffYawRmseDeg: null,
          navigationNeesMean: value.aggregates?.navigation_nees_mean?.mean ?? null,
        },
        limitations: value.scope?.not_covered ?? value.scope?.not_covered ?? [],
      });
    } catch (error) {
      if (error.code === "ENOENT") continue;
      throw error;
    }
  }
  const gitCommit = await readGitCommit();
  const dirty = null;
  return {
    schemaVersion: 2,
    generatedAt: new Date().toISOString(),
    generator: "reports/aerakia-optimization-explorer/scripts/build-validation-data.mjs",
    algorithmCommit: gitCommit,
    repositoryDirty: dirty,
    evidenceGrades: {
      A: "independent physical truth with audited timing/frames",
      B: "external reference with possible source/environment coupling",
      C: "shared physical source between input and scoring reference",
      D: "mature estimator or onboard engineering reference",
      E: "synthetic truth or deterministic fault campaign",
    },
    overview: {
      datasetCount: datasets.length,
      curveCount: curves.length,
      totalSamples: datasets.reduce((sum, item) => sum + (Number(item.samples) || 0), 0),
      independentTruthCount: datasets.filter((item) => item.evidenceGrade === "A").length,
      physicalHeadingCount: datasets.filter((item) => item.id.startsWith("insane-")).length,
      syntheticTrialCount: datasets.find((item) => item.id === "g0-monte-carlo")?.samples ?? null,
    },
    datasets,
    curves,
  };
}

export async function main() {
  const data = await buildValidationData();
  await fs.mkdir(path.dirname(outputPath), { recursive: true });
  await fs.writeFile(outputPath, `${JSON.stringify(data)}\n`);
  process.stdout.write(`wrote ${relative(outputPath)} (${data.curves.length} curves, ${data.datasets.length} dataset entries)\n`);
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) await main();
