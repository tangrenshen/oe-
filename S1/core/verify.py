"""Verify the controller comparisons using only Python's standard library."""

import csv
import gzip
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean


HERE = Path(__file__).resolve().parent
B1 = "base_kp15.0_ki0p02"
B2 = "base_kp12.0_ki0p02"
PROPOSED = "prop_kp12.0_rate30.0"
SLOW = "prop_kp12.0_rate1.0"
NOISE = [f"noise2m_s{seed}" for seed in range(101, 106)]


def check(condition, message):
    if not condition:
        raise ValueError(message)


def csv_rows(name):
    with (HERE / name).open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def json_data(name):
    return json.loads((HERE / name).read_text(encoding="utf-8-sig"))


def printed(actual, string, label):
    digits = len(string.partition(".")[2])
    tolerance = 0.5 * 10 ** (-digits) + 1e-10
    check(abs(actual - float(string)) <= tolerance,
          f"{label}: calculated {actual}, reported {string}")


def validate_selection(training):
    groups = defaultdict(list)
    for row in training:
        groups[row["mode"], row["candidate_id"]].append(row)
    protocol = json_data("protocol.json")
    saved = json_data("frozen_selections.json")
    check(len(groups) == 16 and len(training) == 32, "Incomplete candidate grid")
    for name, mode in [("proposed_candidates", "shared"), ("baseline_candidates", "baseline")]:
        planned = {c["id"] for c in protocol[name]}
        observed = {c for m, c in groups if m == mode}
        check(planned == observed, f"Candidate set mismatch for {mode}")
    for budget in protocol["selection_budgets"]:
        winners = {}
        for mode in ["baseline", "shared"]:
            rankings = []
            for (arm, candidate), records in groups.items():
                if arm != mode:
                    continue
                check({r["scenario"] for r in records} == {"nominal", "gain085"},
                      f"Incomplete selection cases for {candidate}")
                rms = [float(r["rms_m"]) for r in records]
                travel = mean(float(r["rudder_tv_deg"]) for r in records)
                if all(r["arrived"] == "1" for r in records) and travel <= budget["max_mean_tv_deg"]:
                    rankings.append((max(rms), mean(rms), travel, candidate))
            worst, average, travel, candidate = min(rankings)
            archived = saved[budget["name"]][mode]
            check(candidate == archived["candidate_id"], "Selection mismatch")
            for value, key in [(worst, "worst_train_rms_m"), (average, "mean_train_rms_m"), (travel, "mean_train_tv_deg")]:
                check(math.isclose(value, archived[key], abs_tol=1e-10), f"Selection score {key}")
            winners[mode] = candidate
        print("Selection", budget["name"], winners["baseline"], winners["shared"])


def validate_settings():
    history = json_data("earlier_controller_settings.json")["selected"]
    newer = json_data("run_configuration_evidence.json")
    rows = csv_rows("reported_settings.csv")
    check(len(rows) == 6, "Table 4 count")
    for row in rows:
        candidate = row["controller"]
        if candidate in history:
            params = history[candidate]["params"]
            gain = params["ki"]
            bounds = [params["lookahead"]] * 2
            enabled = False
        else:
            record = newer[candidate]
            params = record["params"]
            enabled = record["mode"] == "shared"
            if enabled:
                gain = float(record["applied_environment"].get("OE_EXP_RATE", newer["default_proposed_integral_rate"]))
                bounds = record["lookahead_bounds_L"]
                check(params["kff"] > 0, f"Zero moment feedforward gain for {candidate}")
            else:
                gain = params["ki"]
                bounds = [params["lookahead"]] * 2
        actual = [params["kp"], params["ti"], params["td"], gain, *bounds]
        expected = [float(row[k]) for k in ["kp", "ti", "td", "outer_gain", "min_lookahead_L", "max_lookahead_L"]]
        check(actual == expected and str(enabled) == row["moment_feedforward"], f"Table 4 {candidate}")
    print("Table 4: all six settings match.")


def index_runs():
    training = csv_rows("training_results.csv")
    validate_selection(training)
    followup = csv_rows("targeted_followup_results.csv")
    extra = csv_rows("dir20_supplement_results.csv")
    protocol = json_data("followup_protocol.json")
    candidates = {c["id"] for c in protocol["frozen_configs"]}
    scenarios = {s["name"] for s in protocol["scenarios"]}
    for records, cases in [(followup, scenarios), (extra, {"dir20", "dirm20"})]:
        expected = {(c, s) for c in candidates for s in cases}
        observed = {(r["candidate_id"], r["scenario"]) for r in records}
        check(observed == expected and len(observed) == len(records), "Validation grid mismatch")
    data = {}
    for row in training + followup + extra:
        key = row["candidate_id"], row["scenario"]
        check(key not in data, f"Duplicate run: {key}")
        data[key] = row
    for row in csv_rows("earlier_passages.csv"):
        scenario = "nominal" if row["scenario"] == "dir0" else row["scenario"]
        key = row["method"], scenario
        check(key not in data, f"Duplicate earlier run: {key}")
        data[key] = row
    return data


def validate_tables(data):
    counts = Counter()
    for r in csv_rows("reported_values.csv"):
        table, candidate, scenario, metric, target = (r[k] for k in
            ["table", "controller", "scenario", "metric", "printed_value"])
        scenarios = NOISE if scenario == "noise_mean" else [scenario]
        actual = mean(float(data[candidate, s][metric]) for s in scenarios)
        printed(actual, target, f"Table {table} {candidate} {scenario} {metric}")
        counts[table] += 1
    check(counts == {"5": 36, "6": 60, "11": 96, "C2": 40, "D1": 104, "D2": 104}, "Table comparison counts")
    print("Values matching reported precision:", dict(counts))


def validate_nominal(data):
    raw_rms = {}
    for key, candidate in [("B1", B1), ("B2", B2), ("Proposed", PROPOSED)]:
        with gzip.open(HERE / f"nominal_{key}.csv.gz", "rt", encoding="ascii", newline="") as f:
            rows = list(csv.DictReader(f))
        xte = [float(row["xte_m"]) for row in rows]
        rudder = [float(row["rudder_deg"]) for row in rows]
        time = [float(row["sim_time"]) for row in rows]
        check(all(b - a == 0.5 for a, b in zip(time, time[1:])), f"Sampling continuity {key}")
        metrics = {"rms_m": math.sqrt(mean(v * v for v in xte)), "max_m": max(map(abs, xte)),
                   "mean_abs_rudder_deg": mean(map(abs, rudder)),
                   "near_limit_pct": mean(abs(v) >= 34 for v in rudder) * 100, "transit_s": len(rows) * 0.5}
        for metric, value in metrics.items():
            check(math.isclose(value, float(data[candidate, "nominal"][metric]), rel_tol=1e-12), f"Nominal {key}: {metric}")
        raw_rms[key] = metrics["rms_m"]
        print(f"Nominal {key}: {len(rows)} frames; RMS {metrics['rms_m']:.12f} m")
    for key, reported in [("B1", "6.2"), ("B2", "9.0")]:
        reduction = 100 * (1 - raw_rms["Proposed"] / raw_rms[key])
        printed(reduction, reported, f"Abstract reduction vs {key}")
        print(f"Nominal RMS reduction vs {key}: {reduction:.9f}% ({reported}%)")
    travel = [float(data[c, "nominal"]["rudder_tv_deg"]) for c in (B2, PROPOSED, B1)]
    check(travel[0] < travel[1] < travel[2], "Nominal rudder-travel ordering")


def validate_contrasts(data):
    for scenario in ["dirm20", "dir20"]:
        values = [float(data[candidate, scenario]["rms_m"]) for candidate in [B1, B2, PROPOSED, SLOW]]
        check(max(values[:2]) < values[2] < values[3], f"Direction-bias comparison {scenario}")
        print(f"{scenario} RMS (B1, B2, proposed, slow), m:", " / ".join(f"{v:.3f}" for v in values))
    def differences(comparator, metric):
        return [float(data[PROPOSED, seed][metric]) - float(data[comparator, seed][metric]) for seed in NOISE]
    for metric, expected in [("rms_m", ("-0.565", "-0.729", "-0.321")), ("rudder_tv_deg", ("-326.0", "-414.1", "-258.6"))]:
        values = differences(B2, metric)
        for value, target in zip([mean(values), min(values), max(values)], expected):
            printed(value, target, f"Paired noise vs B2: {metric}")
        print(f"Paired noise vs B2: {metric} mean {mean(values):.6f}, min {min(values):.6f}, max {max(values):.6f}")
    fast_minus_slow = differences(SLOW, "rms_m")
    printed(mean(fast_minus_slow), "-0.00167", "Fast vs slow update")
    check(sum(v < 0 for v in fast_minus_slow) == 3, "Direction of noise contrast")
    print(f"Paired noise vs slow update: RMS mean {mean(fast_minus_slow):.8f} m (3 of 5 lower).")


if __name__ == "__main__":
    validate_settings()
    data = index_runs()
    validate_tables(data)
    validate_nominal(data)
    validate_contrasts(data)
