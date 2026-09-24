# Public development and release policy

This repository intentionally keeps reviewable algorithm evolution public while separating
development state from stable integration points.

## Branch roles

- `main` is the latest reviewed, CI-passing integration baseline. It may still be pre-1.0, but it
  must not knowingly contain a broken public API or an unexplained regression.
- Feature and audit work is developed on short-lived branches and presented through draft pull
  requests while evidence is incomplete.
- A pull request becomes ready only after its scope, test evidence, limitations, and public-data
  boundary have been reviewed.
- FCOne and other private consumers pin an exact commit or release tag; they never follow an
  unreviewed branch tip automatically.

## Releases

- Use semantic pre-1.0 tags such as `v0.3.0` for reviewed integration milestones.
- Release notes state the supported inputs, validated scenarios, known limitations, and any public
  API or state-model change.
- Experimental results do not become reliability claims merely because they appear in a release.
  Physical truth, target timing, HIL, and flight evidence retain their separate labels.

## History policy

Normal algorithm fixes, failed hypotheses, and test-driven corrections remain in branches and pull
requests because they provide useful engineering provenance. Mainline history may use squash merges
to stay readable, while the pull request preserves detailed review context.

Do not rewrite public history merely to hide ordinary development mistakes. History rewriting is
reserved for confirmed credential, personal-data, licensing, or confidential-IP exposure and must
be treated as an incident; already-fetched public objects cannot be assumed recoverable.

## Public boundary

Never commit raw private ULogs, absolute GNSS coordinates, hardware identifiers, credentials,
private board interfaces, customer/company material, or unreviewed third-party code. Public reports
may include explicitly approved, sanitized aggregates with their evidence limits. Hardware adapters,
CubeMX/HAL/RTOS integration, board configuration, and target evidence remain in private FCOne
repositories.
