"""Check archive integrity and reproduce manuscript results using Python 3."""

import hashlib
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def check_files():
    count = 0
    for line in (ROOT / "manifest.sha256").read_text(encoding="utf-8").splitlines():
        digest, relative_path = line.split("  ", 1)
        path = ROOT / relative_path
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError(f"Checksum mismatch for {relative_path}")
        count += 1
    print(f"Verified {count} file checksums.", flush=True)


if __name__ == "__main__":
    check_files()
    for script in ("core/verify.py", "paired/verify_paired.py", "checks/verify.py"):
        print(f"\nRunning {script}", flush=True)
        subprocess.run([sys.executable, str(ROOT / script)], cwd=ROOT, check=True)
    print("\nAll supplementary data checks passed.")
