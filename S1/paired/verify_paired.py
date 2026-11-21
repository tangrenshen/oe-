"""Recompute emitted frame metrics and the comparisons in manuscript Section 4.4."""

import csv
from decimal import Decimal, ROUND_HALF_UP
import io
import math
from pathlib import Path
import zipfile


ROOT = Path(__file__).resolve().parent
SCENARIOS = ("nominal", "phase135", "phase225", "lag20", "gain075")
MODULES = {
    "full_2to5L_moment": ("adaptive_2to5L", 3e-6, 8),
    "no_moment_2to5L": ("adaptive_2to5L", 0.0, 9),
    "fixed4L_moment": ("fixed_4L", 3e-6, 8),
    "fixed4L_no_moment": ("fixed_4L", 0.0, 9),
    "fixed3L_no_moment": ("fixed_3L", 0.0, 9),
}


def keyed_csv(name):
    with (ROOT / name).open(encoding="utf-8-sig", newline="") as inp:
        rows = list(csv.DictReader(inp))
    keyed = {(r["scenario"], r["module_id"]): r for r in rows}
    expected = {(s, m) for s in SCENARIOS for m in MODULES}
    if set(keyed) != expected or len(rows) != len(expected):
        raise ValueError(f"{name}: expected one row for each of the 25 runs")
    return keyed


def check_equal(label, actual, expected):
    if actual != expected:
        raise ValueError(f"{label}: found {actual!r}, expected {expected!r}")


def check_close(label, actual, expected):
    if not math.isclose(actual, float(expected), rel_tol=1e-11, abs_tol=1e-11):
        raise ValueError(f"{label}: found {actual}, expected {expected}")


def check_frame_metrics(archive, scenario, module, row):
    name = f"{module}__{scenario}.csv"
    with archive.open(name) as binary, io.TextIOWrapper(binary, encoding="utf-8") as inp:
        frames = list(csv.DictReader(inp))
    n = len(frames)
    for i, frame in enumerate(frames, 1):
        check_equal(f"{name} frame index", int(frame["frame"]), i)
    xte = [float(f["xte_m"]) for f in frames]
    rudder = [abs(float(f["rudder_deg"])) for f in frames]
    metrics = {
        "rms_m": math.sqrt(math.fsum(e * e for e in xte) / n),
        "max_m": max(map(abs, xte)),
        "mean_abs_rudder_deg": math.fsum(rudder) / n,
        "near_limit_pct": 100 * sum(r >= 34.0 for r in rudder) / n,
        "transit_s": n * 0.5,
    }
    for metric, value in metrics.items():
        check_close(f"{name} {metric}", value, row[metric])


def comparison(on, reference, metric):
    return 100 * (float(on[metric]) / float(reference[metric]) - 1)


def main():
    data = keyed_csv("new_candidate_ablation.csv")
    tables = keyed_csv("manuscript_values.csv")
    summaries = keyed_csv("simulator_summaries.csv")
    with zipfile.ZipFile(ROOT / "paired_frame_series.zip") as archive:
        check_equal("frame series count", len(archive.namelist()), 25)
        for (s, m), row in data.items():
            run = f"{s}/{m}"
            summary, printed = summaries[(s, m)], tables[(s, m)]
            mode, kff, table = MODULES[m]
            check_equal(run + " lookahead", row["lookahead_mode"], mode)
            check_equal(run + " table", int(printed["table"]), table)
            check_equal(run + " reused", row["reused"], str(m == "full_2to5L_moment"))
            check_equal(run + " arrived", (row["arrived"], summary["arrived"]), ("1", "1"))
            check_equal(run + " split", summary["fsi_split"], "4x2")
            check_equal(run + " feedforward gain", float(row["kff"]), kff)
            controls = {"kff": kff, "kp": 12.0, "ti": 50.0, "td": 2.0,
                        "ff_bias_deg": 0.0, "ff_gain": 0.75 if s == "gain075" else 1.0,
                        "ff_lag_s": 20.0 if s == "lag20" else 0.0,
                        "backwater_beta": 0.4 if s.startswith("phase") else 0.0,
                        "backwater_phase_deg": float(s[5:]) if s.startswith("phase") else 0.0}
            for field, value in controls.items():
                check_equal(run + " " + field, float(summary[field]), value)
            for metric, digits in (("rms_m", 3), ("max_m", 2), ("rudder_tv_deg", 1)):
                rounded = Decimal(row[metric]).quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP)
                check_equal(run + " " + metric, str(rounded), printed[metric])
            check_equal(run + " simulator travel", summary["sum_abs_rudder_change_deg"], row["rudder_tv_deg"])
            check_close(run + " simulator transit", float(summary["sim_time_s"]), row["transit_s"])
            check_frame_metrics(archive, s, m, row)

    print("scenario  RMS reduction (%) / rudder travel increase (%) when moment feedforward is enabled;")
    print("          RMS reduction (%) / peak reduction (%) with adaptive rather than fixed 4L lookahead")
    effects = []
    for s in SCENARIOS:
        full = data[(s, "full_2to5L_moment")]
        off = data[(s, "no_moment_2to5L")]
        fixed = data[(s, "fixed4L_moment")]
        values = (-comparison(full, off, "rms_m"), comparison(full, off, "rudder_tv_deg"),
                  -comparison(full, fixed, "rms_m"), -comparison(full, fixed, "max_m"))
        effects.append(values)
        print(f"{s:<9s} " + " ".join(f"{v:8.2f}" for v in values))
    ranges = [(f"{min(col):.2f}", f"{max(col):.2f}") for col in zip(*effects)]
    check_equal("Section 4.4 percentage ranges", ranges,
                [("1.40", "21.34"), ("3.41", "7.83"), ("5.55", "27.95"), ("0.72", "26.95")])
    for s, expected in (("nominal", "0.042"), ("phase135", "0.351")):
        diff = float(data[s, "full_2to5L_moment"]["max_m"]) - float(data[s, "no_moment_2to5L"]["max_m"])
        check_equal(f"{s} peak increase (m)", f"{diff:.3f}", expected)
    for s, expected in (("phase135", "1.11"), ("gain075", "0.81")):
        diff = comparison(data[s, "fixed4L_moment"], data[s, "fixed4L_no_moment"], "rms_m")
        check_equal(f"{s} fixed 4L RMS increase (%)", f"{diff:.2f}", expected)
    for m, expected in (("no_moment_2to5L", "0.799"), ("full_2to5L_moment", "1.659")):
        check_equal(f"lag20 {m} near-limit (%)", f"{float(data['lag20', m]['near_limit_pct']):.3f}", expected)
    best = min(MODULES, key=lambda m: float(data["gain075", m]["rms_m"]))
    check_equal("speed factor 0.75 best RMS", best, "fixed3L_no_moment")
    print("Verified 25 runs, 75 reported table values and Section 4.4 contrasts.")


if __name__ == "__main__":
    main()
