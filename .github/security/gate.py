"""Public Actions gate: emit counts only; detailed review stays private."""
import json
import os
import pathlib
import re
import sys
from scanlib import command, scan

def revision(value):
    if not re.fullmatch(r"[a-f0-9]{40}", value or ""):
        raise ValueError("Invalid Git revision")
    return value

def options(event, event_name):
    if event_name == "pull_request":
        base = revision(event["pull_request"]["base"]["sha"])
        head = revision(event["pull_request"]["head"]["sha"])
        return f"--full-history -m {base}..{head}"
    if event_name == "push":
        head = revision(event["after"])
        base = revision(event["before"])
        if base != "0" * 40:
            return f"--full-history -m {base}..{head}"
        return f"--full-history -m {head}"
    # Manual smoke run checks only the latest commit. Full history belongs to
    # the private monitor, including commits already deleted from the tip.
    return "--full-history -m -1 HEAD"

def main():
    event = json.loads(pathlib.Path(os.environ["GITHUB_EVENT_PATH"]).read_text())
    source = pathlib.Path("source").resolve()
    event_name = os.environ["GITHUB_EVENT_NAME"]
    opts = options(event, event_name)
    if event_name == "push" and event["before"] != "0" * 40:
        old_exists = command(["git", "-C", str(source), "cat-file", "-e", revision(event["before"])], allowed=(0, 1, 128))
        if old_exists.returncode:
            opts = "--full-history -m " + revision(event["after"])
    findings = scan(source, log_opts=opts)
    secrets = sum(x["classification"] == "secret-review" for x in findings.values())
    privacy = len(findings) - secrets
    summary = f"Sensitive content check: {secrets} secret candidates; {privacy} privacy candidates.\n"
    summary += "No sensitive values or locations are published in this log. The owner can review the private github-sensitive-monitor repository.\n"
    print(summary)
    with open(os.environ.get("GITHUB_STEP_SUMMARY", os.devnull), "a") as f:
        f.write(summary)
    if secrets:
        print("::error::Potential secret detected. Review privately and rotate confirmed credentials.")
        return 1
    if privacy:
        print("::warning::Personal information candidates require private review.")
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"::error::Sensitive content scan incomplete ({type(error).__name__}); details suppressed.")
        sys.exit(2)
