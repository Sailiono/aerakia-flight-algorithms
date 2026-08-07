# Aerakia Validation Workstation

An auditable local viewer for Aerakia replay results. It is intentionally a
diagnostic workstation, not a flight-readiness dashboard: every dataset keeps
its evidence grade, source hash, truth boundary, and limitations.

## Prerequisites

- Node.js `>=22.13.0`

## Quick Start

```bash
npm install
npm run build
npm test
npm run dev
```

The committed `public/data/validation-data.json` is the portable display
artifact. `npm test` and `npm run build` deliberately use it without requiring
raw datasets or a parent repository `build/` directory.

Regenerate that artifact only after reproducing the parent repository's
validation suite and retaining the required compact replay outputs:

```bash
npm run data:build
npm run test:rebuild-data
```

`data:build` reads those local algorithm build outputs and fails closed if
required CSV columns, finite values, or strictly increasing timestamps are
missing. The raw source datasets, replay CSVs, and generated native-build
products remain outside this workstation and are safe to remove after their
compact summaries have been committed.

## Views

- `曲线诊断`: raw EuRoC sample curves, quaternion geodesic error, wrapped Euler
  diagnostics, RMSE and consistency metrics.
- `覆盖矩阵`: evidence grade, sample volume, truth/source class, heading/GNSS
  coverage, and explicit limitations for EuRoC, Blackbird, UrbanNav, INSANE,
  IDF-DS, electrical UAV, and synthetic campaigns.

The generated artifact records the algorithm commit and SHA-256 hashes of the
input summaries and replay result files. Raw ULogs, public dataset archives,
and generated replay CSVs remain outside the frontend.

## Useful Commands

- `npm run dev`: start local development
- `npm run data:build`: rebuild the compact validation data catalog
- `npm run build`: verify the vinext build output
- `npm test`: build the site and run rendering/data-integrity tests without raw datasets
