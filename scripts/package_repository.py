"""Refresh the checksum manifest and prepare a ZIP without Git or build artifacts."""

from pathlib import Path
import csv
import hashlib
import zipfile


ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {".git", "build_source", "private_inputs", "__pycache__", "smoke"}


def included(path):
    relative = path.relative_to(ROOT)
    return (
        path.is_file()
        and not any(part in EXCLUDED for part in relative.parts)
        and path.suffix.lower() not in {".db", ".exe", ".dll", ".obj", ".pyc"}
        and path.name != "manifest.csv"
    )


files = sorted(path for path in ROOT.rglob("*") if included(path))
with (ROOT / "manifest.csv").open("w", encoding="utf-8", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(["path", "bytes", "sha256"])
    for path in files:
        writer.writerow([
            path.relative_to(ROOT).as_posix(),
            path.stat().st_size,
            hashlib.sha256(path.read_bytes()).hexdigest(),
        ])

archive = ROOT.parent / "OE_data_repository.zip"
with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
    for path in [*files, ROOT / "manifest.csv"]:
        bundle.write(path, path.relative_to(ROOT).as_posix())
with zipfile.ZipFile(archive) as bundle:
    if bundle.testzip() is not None:
        raise ValueError("ZIP integrity check failed")
print(f"Prepared {len(files)} files plus manifest: {archive}")
