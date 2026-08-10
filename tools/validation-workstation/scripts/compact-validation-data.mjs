import fs from "node:fs/promises";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { createHash } from "node:crypto";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const artifactPath = path.resolve(scriptDir, "../public/data/validation-data.json");
const repositoryRoot = path.resolve(scriptDir, "../../..");
const maximumRows = 4000;

function evenlySample(rows) {
  if (rows.length <= maximumRows) return rows;
  return Array.from({ length: maximumRows }, (_, index) => (
    rows[Math.round((index * (rows.length - 1)) / (maximumRows - 1))]
  ));
}

const artifact = JSON.parse(await fs.readFile(artifactPath, "utf8"));
if (!Array.isArray(artifact.curves)) throw new Error("validation artifact has no curve collection");
for (const curve of artifact.curves) {
  if (!Array.isArray(curve.rows) || curve.rows.length === 0) {
    throw new Error(`curve ${curve.id ?? "<unknown>"} has no rows`);
  }
  const originalCount = Number(curve.samples ?? curve.rows.length);
  if (originalCount < curve.rows.length) {
    throw new Error(`curve ${curve.id ?? "<unknown>"} has fewer declared than stored samples`);
  }
  curve.samples = originalCount;
  curve.rows = evenlySample(curve.rows);
  curve.storedSamples = curve.rows.length;
}
artifact.schemaVersion = 3;
artifact.displaySampling = {
  method: "uniform-index-decimation",
  maximumRowsPerCurve: maximumRows,
  note: "Full replay statistics and source hashes remain authoritative; rows are for interactive display only.",
};
const barometerPath = path.join(repositoryRoot, "validation/public/barometer_outage_campaign_v1.json");
try {
  const buffer = await fs.readFile(barometerPath);
  const evidence = JSON.parse(buffer.toString("utf8"));
  const sourcePath = "validation/public/barometer_outage_campaign_v1.json";
  const entry = {
    id: "baro-outage-v1",
    label: "Barometer outage campaign · v1",
    evidenceGrade: "E",
    referenceKind: "synthetic_fault_campaign",
    role: "relative-height source-supervision boundary",
    sourcePath,
    sourceSha256: createHash("sha256").update(buffer).digest("hex"),
    dataset: evidence.campaign?.id ?? null,
    sequence: null,
    samples: evidence.campaign?.completed_trials ?? null,
    durationS: null,
    coverage: { imuSamples: null, headingUpdates: null, gnssUpdates: null, outageS: null },
    highlights: {
      eskfAttitudeRmseDeg: null, eskfYawRmseDeg: null, tiltRmseDeg: null,
      positionRmseM: null, outagePeakErrorM: null,
      healthyRatio: evidence.result?.all_arm_healthy_ratio_min ?? null,
      magnetometerOnYawRmseDeg: null, magnetometerOffYawRmseDeg: null,
      navigationNeesMean: null,
    },
    limitations: evidence.limitations ?? [],
  };
  artifact.datasets = (artifact.datasets ?? []).filter((item) => item.id !== entry.id);
  artifact.datasets.push(entry);
} catch (error) {
  if (error.code !== "ENOENT") throw error;
}
const rateSensitivityPath = path.join(repositoryRoot, "validation/public/g0_rate_sensitivity.json");
try {
  const buffer = await fs.readFile(rateSensitivityPath);
  const evidence = JSON.parse(buffer.toString("utf8"));
  const sourcePath = "validation/public/g0_rate_sensitivity.json";
  const entry = {
    id: "g0-rate-sensitivity",
    label: "G0 · bias observability rate sensitivity",
    evidenceGrade: "E",
    referenceKind: "synthetic_contract_diagnostic",
    role: "rate/window/analyzer diagnostic; not an estimator gate",
    sourcePath,
    sourceSha256: createHash("sha256").update(buffer).digest("hex"),
    dataset: evidence.dataset ?? evidence.study_id ?? null,
    sequence: evidence.sequence ?? null,
    samples: evidence.samples ?? evidence.cases?.length ?? null,
    durationS: evidence.duration_s ?? null,
    coverage: { imuSamples: null, headingUpdates: null, gnssUpdates: null, outageS: null },
    highlights: {
      eskfAttitudeRmseDeg: null, eskfYawRmseDeg: null, tiltRmseDeg: null,
      positionRmseM: null, outagePeakErrorM: null,
      healthyRatio: evidence.healthy_ratio ?? null,
      magnetometerOnYawRmseDeg: null, magnetometerOffYawRmseDeg: null,
      navigationNeesMean: null,
    },
    limitations: evidence.limitations ?? [],
  };
  artifact.datasets = (artifact.datasets ?? []).filter((item) => item.id !== entry.id);
  artifact.datasets.push(entry);
} catch (error) {
  if (error.code !== "ENOENT") throw error;
}
for (const spec of [
  {
    id: "g0-trajectory-screen",
    label: "G0 · zero-noise trajectory screen",
    referenceKind: "synthetic_structural_diagnostic",
    role: "maneuver geometry candidate screen; not noisy convergence evidence",
    file: "validation/public/g0_trajectory_screen.json",
    samples: (evidence) => evidence.cases?.length ?? null,
  },
  {
    id: "g0-gate-characterization",
    label: "G0 · analyzer gate characterization",
    referenceKind: "synthetic_analyzer_error_rate",
    role: "stationary null and structural controls; not an estimator gate",
    file: "validation/public/g0_gate_characterization.json",
    samples: (evidence) => evidence.static_null?.total_cases ?? null,
  },
  {
    id: "g0-gate-null-split",
    label: "G0 · disjoint null split validation",
    referenceKind: "synthetic_analyzer_calibration",
    role: "held-out stationary score-threshold check; not an estimator gate",
    file: "validation/public/g0_gate_null_split_validation.json",
    samples: (evidence) => evidence.validation?.case_count ?? null,
  },
  {
    id: "g0-excitation-score",
    label: "G0 · noisy excitation score campaign",
    referenceKind: "synthetic_analyzer_calibration",
    role: "noisy maneuver score check; not bias-convergence evidence",
    file: "validation/public/g0_excitation_score_campaign.json",
    samples: (evidence) => evidence.cases?.length ?? null,
  },
]) {
  const evidencePath = path.join(repositoryRoot, spec.file);
  try {
    const buffer = await fs.readFile(evidencePath);
    const evidence = JSON.parse(buffer.toString("utf8"));
    const entry = {
      id: spec.id,
      label: spec.label,
      evidenceGrade: "E",
      referenceKind: spec.referenceKind,
      role: spec.role,
      sourcePath: spec.file,
      sourceSha256: createHash("sha256").update(buffer).digest("hex"),
      dataset: evidence.dataset ?? evidence.study_id ?? null,
      sequence: evidence.sequence ?? evidence.null_definition ?? null,
      samples: spec.samples(evidence),
      durationS: evidence.duration_s ?? null,
      coverage: { imuSamples: null, headingUpdates: null, gnssUpdates: null, outageS: null },
      highlights: {
        eskfAttitudeRmseDeg: null, eskfYawRmseDeg: null, tiltRmseDeg: null,
        positionRmseM: null, outagePeakErrorM: null,
        healthyRatio: evidence.healthy_ratio ?? null,
        magnetometerOnYawRmseDeg: null, magnetometerOffYawRmseDeg: null,
        navigationNeesMean: null,
      },
      limitations: evidence.limitations ?? evidence.interpretation ?? [],
    };
    artifact.datasets = (artifact.datasets ?? []).filter((item) => item.id !== entry.id);
    artifact.datasets.push(entry);
  } catch (error) {
    if (error.code !== "ENOENT") throw error;
  }
}
if (artifact.overview && Array.isArray(artifact.datasets)) {
  artifact.overview.datasetCount = artifact.datasets.length;
  artifact.overview.totalSamples = artifact.datasets.reduce(
    (sum, item) => sum + (Number(item.samples) || 0), 0,
  );
  artifact.overview.independentTruthCount = artifact.datasets.filter(
    (item) => item.evidenceGrade === "A",
  ).length;
}
await fs.writeFile(artifactPath, `${JSON.stringify(artifact)}\n`);
process.stdout.write(`compacted ${artifact.curves.length} validation curves to at most ${maximumRows} rows each\n`);
