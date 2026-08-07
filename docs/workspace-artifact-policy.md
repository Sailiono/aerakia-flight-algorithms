# Workspace Artifact Policy

This repository is the durable source of the Aerakia algorithm and its
hardware-independent validation contracts. A temporary development machine
must not be treated as the source of truth for generated data.

## Retain in Git

- `src/`, `include/`, `tests/`, and `validation/`: portable code, contracts,
  fixtures, runners, thresholds, and deterministic test logic.
- `validation/public/`: compact, hashed evidence summaries and manifests. These
  do not contain raw ULogs, private coordinates, or replay CSVs.
- `docs/`: methods, decisions, limitations, execution logs, and the compact
  optimization showcase.
- `tools/validation-workstation/`: the local evidence viewer source and its
  committed `public/data/validation-data.json` display artifact.

## Do not retain in Git or on a temporary machine

- `build/`, `build-*`, `cmake-build-*`: compiler products, raw dataset intake,
  virtual environments, replay CSVs, plots, and downloaded source trees.
- `__pycache__/`, `.pytest_cache/`, `node_modules/`, `dist/`, `.wrangler/`, and
  other tool caches.
- Original ULogs, public dataset archives, private manifests, device IDs,
  absolute GPS data, and full-resolution replay outputs. Keep those in the
  private backup/archive workflow with SHA-256 manifests.

Generated files may be used for investigation, but a conclusion is durable
only after its reason, command, code identity, input identity, result summary,
and limitation are recorded in `docs/` or `validation/public/`.

## Rebuild after cleanup

```bash
cmake -S . -B build/host-clean -DCMAKE_BUILD_TYPE=Release
cmake --build build/host-clean --parallel
ctest --test-dir build/host-clean --output-on-failure
python3 -m unittest discover -s tests -p 'test_*.py'
```

The workstation can be tested immediately from its committed compact artifact:

```bash
cd tools/validation-workstation
npm ci
npm test
```

Regenerating that artifact is an explicit operation and requires restored EuRoC
replay outputs; ordinary workstation tests must never download or regenerate
raw validation data implicitly.

## Campaign cleanup rule

Resumable campaigns may keep shard summaries and logs under `build/` while they
run. After merging and reviewing them, commit only the compact result that is
needed to support a claim, then remove all shard directories. An incomplete
campaign is not evidence and must be reported as incomplete rather than
silently pooled with an older run.
