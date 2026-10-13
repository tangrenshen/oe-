"""Verify the supplied manifest and steady-field summaries without external packages."""

from pathlib import Path
import csv
import hashlib
import re


ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    0: (46.14, 105.85),
    1: (20.68, 67.58),
    2: (46.18, 106.60),
    3: (2.29, 15.48),
}


def verify():
    with (ROOT / "manifest.csv").open(encoding="utf-8-sig", newline="") as stream:
        entries = list(csv.DictReader(stream))
    seen = set()
    for entry in entries:
        relative = entry["path"]
        if relative in seen:
            raise ValueError(f"Duplicate manifest entry: {relative}")
        seen.add(relative)
        path = (ROOT / relative).resolve()
        if not path.is_relative_to(ROOT):
            raise ValueError(f"Invalid manifest path: {relative}")
        if not path.is_file() or path.stat().st_size != int(entry["bytes"]):
            raise ValueError(f"Missing file or size mismatch: {relative}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError(f"SHA-256 mismatch: {relative}")

    for group, expected in EXPECTED.items():
        folder = ROOT / "recovered_runs" / "steady" / f"g{group}"
        summaries = list(folder.glob("Summary_*.txt"))
        if len(summaries) != 1:
            raise ValueError(f"Expected one summary for G{group + 1}")
        values = dict(re.findall(r"(\w+)=([^\s]+)", summaries[0].read_text(encoding="utf-8")))
        actual = (float(values["rms_xte_m"]), float(values["max_abs_xte_m"]))
        if values.get("arrived") != "1" or actual != expected:
            raise ValueError(f"Steady-field summary mismatch for G{group + 1}: {actual}")
        print(f"G{group + 1}: RMS {actual[0]:.2f} m, maximum {actual[1]:.2f} m")
    print(f"Verified {len(entries)} files and four steady-field summaries.")


if __name__ == "__main__":
    verify()
