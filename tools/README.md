# Developer Tools

The tools directory contains optional development-facing utilities rather than
algorithm or flight software.

- [`validation-workstation`](validation-workstation): local evidence viewer
  for the compact, committed validation summaries.  Its checked-in display
  data is intentionally decimated; source hashes and compact JSON evidence are
  authoritative.

Do not put downloaded datasets, ULogs, compiler outputs, or one-off analysis
notebooks here.  Use the ignored `build/` workspace while investigating, then
retain only the script, protocol, compact result, and written conclusion.
