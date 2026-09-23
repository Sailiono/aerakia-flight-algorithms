"""Install a pinned, SHA256-verified upstream binary without running archive code."""
import hashlib
import io
import os
import pathlib
import platform
import tarfile
import urllib.request

VERSION = "8.30.1"
DIGESTS = {
    ("Linux", "x86_64"): ("linux_x64", "551f6fc83ea457d62a0d98237cbad105af8d557003051f41f3e7ca7b3f2470eb"),
    ("Darwin", "arm64"): ("darwin_arm64", "b40ab0ae55c505963e365f271a8d3846efbc170aa17f2607f13df610a9aeb6a5"),
}

def install(destination="bin/gitleaks"):
    target = pathlib.Path(destination)
    system, digest = DIGESTS[(platform.system(), platform.machine())]
    url = f"https://github.com/gitleaks/gitleaks/releases/download/v{VERSION}/gitleaks_{VERSION}_{system}.tar.gz"
    with urllib.request.urlopen(url, timeout=120) as response:
        data = response.read(32 * 1024 * 1024)
    if hashlib.sha256(data).hexdigest() != digest:
        raise RuntimeError("Gitleaks archive checksum mismatch")
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        member = archive.getmember("gitleaks")
        if not member.isfile():
            raise RuntimeError("Unexpected executable archive member")
        binary = archive.extractfile(member).read()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(binary)
    target.chmod(0o700)
    print(f"Verified Gitleaks {VERSION}")
    return str(target.resolve())

if __name__ == "__main__":
    install()
