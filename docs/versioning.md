# Naming and version boundaries

This project has several independent numbering systems. They must never be
used interchangeably.

| Scope | Canonical form | Meaning |
| --- | --- | --- |
| FCOne hardware | `FCOne-HW-v1`, `FCOne-HW-v2` | Physical FCOne board generations. This repository does not define or imply an FCOne hardware v3 or later. |
| FCOne firmware target | `fcone_v1`, `fcone_v2` | Private compile-time board targets corresponding to the two hardware generations. |
| Aerakia public API | `v0.3.0` | Semantic API release recorded in `include/aerakia/version.h`; it is not a board revision. |
| Validation study | Descriptive name plus `R<n>` | A host-side experiment revision, with no hardware or firmware-generation meaning. |

The TAS residual-persistence study has older frozen artifact names containing
`v4` through `v8`. Those names are retained because scripts, hashes, and prior
reports reference them. In new prose, refer to them as, for example,
"TAS residual persistence experiment R8". Use the historical `v8` spelling
only when a literal file, script, or artifact path must be identified.

The public Aerakia repository is hardware-neutral. FCOne board configuration,
CubeMX output, drivers, scheduler, selector policy, and flight authority live
in the private FCOne firmware repository.
