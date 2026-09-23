"""No target code is executed. Only explicitly selected metadata leaves the scanner."""
import hashlib
import json
import os
import pathlib
import re
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent

def command(args, *, cwd=None, timeout=1800, input=None, allowed=(0,)):
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GIT_CONFIG_NOSYSTEM="1",
               GIT_CONFIG_GLOBAL=os.devnull, GIT_LFS_SKIP_SMUDGE="1")
    # Scan subprocesses do not need credentials or Actions write tokens.
    for key in ("GH_TOKEN", "GITHUB_TOKEN", "ACTIONS_RUNTIME_TOKEN"):
        env.pop(key, None)
    p = subprocess.run(args, cwd=cwd, env=env, input=input, capture_output=True, timeout=timeout)
    if p.returncode not in allowed:
        raise RuntimeError(f"{pathlib.Path(args[0]).name} failed (exit {p.returncode}); output suppressed")
    return p

def safe_path(value):
    value = str(value).replace("\r", " ").replace("\n", " ")[:400]
    value = re.sub(r"[\w.+%-]+@[\w.-]+\.[A-Za-z]{2,}", "[email]", value)
    value = re.sub(r"(?<!\d)1[3-9]\d{9}(?!\d)", "[phone]", value)
    value = re.sub(r"[A-Za-z0-9_+=-]{36,}", "[long-value]", value)
    return value

def normalize(raw, source="history"):
    result = {}
    for finding in raw:
        rule = finding.get("RuleID", "unknown")
        if not re.fullmatch(r"[a-zA-Z0-9_.-]{1,100}", rule):
            rule = "unknown"
        commit = finding.get("Commit", "")
        if not re.fullmatch(r"[a-f0-9]{40,64}", commit):
            commit = ""
        # Hash the original location before path masking. Never retain Match, Secret,
        # author, email, message, or an unfiltered upstream fingerprint.
        location = [source, rule, finding.get("File", ""), finding.get("StartLine", 0), commit]
        identity = hashlib.sha256(json.dumps(location, ensure_ascii=True).encode()).hexdigest()
        result[identity] = {"id": identity, "rule": rule, "source": source,
            "path": safe_path(finding.get("File", "")), "line": finding.get("StartLine", 0),
            "commit": commit, "classification": "privacy-review" if rule.startswith("privacy-") else "secret-review"}
    return result

def scan(target, *, mode="git", log_opts="--all --full-history -m", source="history"):
    with tempfile.TemporaryDirectory() as td:
        report = pathlib.Path(td) / "raw.json"
        args = [str(ROOT / "bin/gitleaks"), mode, str(target), "--config", str(ROOT / "rules.toml"),
                "--redact=100", "--no-banner", "--no-color", "--log-level=error",
                "--report-format=json", "--report-path", str(report), "--exit-code=10",
                "--ignore-gitleaks-allow", "--gitleaks-ignore-path", os.devnull,
                "--max-decode-depth=2", "--max-archive-depth=2", "--timeout=1200"]
        if mode == "git":
            args += ["--log-opts=" + log_opts]
        command(args, timeout=1250, allowed=(0, 10))
        if not report.exists():
            raise RuntimeError("Scanner did not produce a report")
        return normalize(json.loads(report.read_text()) or [], source)
