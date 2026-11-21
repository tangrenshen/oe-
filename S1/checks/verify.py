"""Verify the earlier controller experiments and analytic-field checks with Python 3."""

import csv
import io
import json
import math
import statistics
import zipfile
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def rows(name):
    with (ROOT / name).open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def metadata(name):
    return json.loads((ROOT / name).read_text(encoding="utf-8"))


def require(condition, label):
    if not condition:
        raise ValueError(label)


def close(actual, expected, label):
    require(math.isclose(actual, float(expected), rel_tol=1e-10, abs_tol=1e-10),
            f"{label}: {actual} vs {expected}")


def check_fields():
    queries = defaultdict(list)
    anchors = defaultdict(list)
    for mapping, filename in ((queries, "analytic_queries.csv"), (anchors, "analytic_anchors.csv")):
        for row in rows(filename):
            mapping[row["field"], row["depth_mode"], row["n"]].append(row)
    published = [("0.0280", "0.00", "0.0015"), ("0.0603", "1.71", "0.0021"),
                 ("0.1608", "4.98", "0.0376")]
    keys = ["vector_rmse_mps", "speed_rmse_mps", "direction_rmse_deg",
            "hull_difference_rmse_mps", "true_hull_difference_rms_mps"]
    metrics = rows("analytic_field_metrics.csv")
    require(len(queries) == len(anchors) == len(metrics) == 18, "field case count")
    for metric in metrics:
        group = metric["field"], metric["depth_mode"], metric["anchors"]
        points = queries[group]
        require(len(points) == int(metric["queries"]) == 240, f"query count {group}")
        require(len(anchors[group]) == int(group[2]), f"anchor count {group}")
        squared = [[] for key in keys]
        for point in points:
            u, v, ue, ve, du, dv, due, dve = [float(point[k]) for k in
                ("true_u", "true_v", "estimated_u", "estimated_v", "true_du",
                 "true_dv", "estimated_du", "estimated_dv")]
            angle = math.degrees(math.atan2(ve, ue) - math.atan2(v, u))
            angle = (angle + 180.0) % 360.0 - 180.0
            errors = (math.hypot(ue-u, ve-v), math.hypot(ue, ve)-math.hypot(u, v),
                      angle, math.hypot(due-du, dve-dv), math.hypot(du, dv))
            for column, value in zip(squared, errors):
                column.append(value ** 2)
        rms = [math.sqrt(statistics.mean(column)) for column in squared]
        for key, value in zip(keys, rms):
            close(value, metric[key], f"{group} {key}")
        if group[1:] == ("0", "9"):
            require((f"{rms[0]:.4f}", f"{rms[2]:.2f}", f"{rms[3]:.4f}") ==
                    published[int(group[0])], "Table 10")


def check_controller_runs():
    nominal = rows("tradeoff_results.csv")
    heldout = rows("heldout_results.csv")
    mechanisms = rows("mechanism_results.csv")
    require((len(nominal), len(heldout), len(mechanisms)) == (24, 6, 12), "run counts")
    for choice in metadata("heldout_selection.json"):
        feasible = [row for row in nominal if row["mode"] == choice["mode"]
                    and float(row["rms_m"]) <= choice["threshold"]]
        best = min(feasible, key=lambda r: float(r["rudder_tv_deg"]))
        require(best["name"] == choice["name"], "nominal minimum travel under error cap")
    by_name = {row["name"]: row for row in nominal + heldout + mechanisms}
    published = rows("reported_rows.csv")
    require(len(published) == 21, "Table 7 and Table 12 row count")
    for report in published:
        source = by_name[report["name"]]
        for column, expected in report.items():
            if column not in ("name", "table") and expected:
                digits = len(expected.split(".")[1])
                # Table values can round exact midpoints in either direction.
                require(abs(float(source[column]) - float(expected)) <= 0.5 * 10**-digits + 1e-10,
                        f"Table {report['table']} {report['name']} {column}")
    with zipfile.ZipFile(ROOT / "figure16_series.zip") as archive:
        selected = [row for row in mechanisms if row["scenario"] == "gain075"]
        require({row["name"] + ".csv" for row in selected} == set(archive.namelist()),
                "Figure 16 series")
        for row in selected:
            with archive.open(row["name"] + ".csv") as data:
                series = list(csv.DictReader(io.TextIOWrapper(data, encoding="utf-8")))
            require(len(series) == int(row["samples"]), "Figure 16 sample count")
            def values(key):
                return [float(s[key]) for s in series]
            xt = values("xte_m")
            close(math.sqrt(statistics.mean(x*x for x in xt)), row["rms_m"], "Figure 16 RMS")
            close(max(map(abs, xt)), row["max_m"], "Figure 16 peak")
            for key, summary_key in [("outer_integral_state_m", "integral_max_abs"),
                                     ("heading_integral_rad_s", "heading_integral_max")]:
                close(max(map(abs, values(key))), row[summary_key], "Figure 16 " + key)


def check_mesh():
    records = rows("mesh_refinement.csv")
    reproduced = {"baseline": [1.406, 1.403, 1.404, 1.404],
                  "shared": [1.842, 1.838, 1.843, 1.838]}
    for mode, expected in reproduced.items():
        group = sorted((r for r in records if r["mode"] == mode), key=lambda r: int(r["split"]))
        require([int(r["split"]) for r in group] == [4, 8, 16, 32], "hull mesh")
        require([round(float(r["rms_m"]), 3) for r in group] == expected, "Figure 11 RMS")
        moments = [float(row["true_moment_rms_Nm"]) for row in group]
        require(abs(moments[-1] - moments[-2]) / moments[-1] < 0.005, "Figure 11 last refinement")


def check_randomized():
    data = rows("randomized_runs.csv")
    summary = metadata("randomized_summary.json")
    by_group = defaultdict(dict)
    for row in data:
        require(row["arrived"] == "1", "Table C1 completed passage")
        by_group[row["kind"], row["method"]][int(row["seed"])] = float(row["rms_m"])
    require(len(data) == 40 and len(by_group) == 4, "Table C1 run count")
    published = {("noise","ilos_crab"): (2.346,2.378,2.469),
                 ("noise","proposed"): (2.194,2.291,2.360),
                 ("derivatives","ilos_crab"): (1.432,1.502,1.728),
                 ("derivatives","proposed"): (1.868,1.923,2.182)}
    for result in summary["groups"]:
        key = result["kind"], result["method"]
        group = by_group[key]
        require(set(group) == set(range(101, 111)), "Table C1 seed IDs")
        quantiles = statistics.quantiles(group.values(), n=4, method="inclusive")
        require(tuple(round(q, 3) for q in quantiles) == published[key], "Table C1 quartiles")
        for value, k in zip(quantiles, ("q25_rms_m", "median_rms_m", "q75_rms_m")):
            close(value, result[k], k)
    for result in summary["paired"]:
        kind = result["kind"]
        diff = [by_group[kind, "proposed"][seed] - by_group[kind, "ilos_crab"][seed]
                for seed in range(101, 111)]
        close(statistics.mean(diff), result["mean_paired_difference_m"], "paired mean")
        close(statistics.median(diff), result["median_paired_difference_m"], "paired median")


if __name__ == "__main__":
    check_fields()
    check_controller_runs()
    check_mesh()
    check_randomized()
    print("Verified Tables 7, 10, 12 and C1, nominal mesh refinement and Figure 16 histories.")
