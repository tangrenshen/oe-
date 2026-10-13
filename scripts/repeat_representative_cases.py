"""Repeat selected historical cases and retain outputs, commands, and checksums."""

from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from datetime import datetime, timezone, timedelta
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import os
import re
import subprocess
import time


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--executable", type=Path, required=True)
parser.add_argument("--chart", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--workdir", type=Path)
args = parser.parse_args()
ROOT = args.output.resolve()
EXE = args.executable.resolve()
CHART = args.chart.resolve()
PROJECT = args.workdir.resolve() if args.workdir else CHART.parent
os.environ["YINGONGZHOU_CHART_DB"] = str(CHART)
LOCAL_TIME = timezone(timedelta(hours=8))
CASES = [
    ("derivative_g4_seed7_q94_5", 3, ["--deriv-perturb", "0.20", "--deriv-seed", "7"], 11.78, 96.88),
    ("derivative_g2_seed7", 1, ["--deriv-perturb", "0.20", "--deriv-seed", "7"], 20.77, 68.28),
    ("derivative_g4_seed7_q100", 3, ["--deriv-perturb", "0.20", "--deriv-seed", "7", "--own-rpm", "100"], 10.63, 95.19),
    ("derivative_g4_seed7_q105", 3, ["--deriv-perturb", "0.20", "--deriv-seed", "7", "--own-rpm", "105"], 3.40, 26.82),
    ("prior_gain070", 3, ["--ff-gain", "0.7"], 12.49, 27.76),
    ("prior_gain130", 3, ["--ff-gain", "1.3"], 11.96, 30.12),
    ("prior_bias_p15", 3, ["--ff-bias", "15"], 16.53, 36.35),
    ("prior_bias_m15", 3, ["--ff-bias", "-15"], 17.11, 32.43),
    ("prior_lag60", 3, ["--ff-lag", "60"], 5.23, 26.81),
    ("prior_combined", 3, ["--ff-gain", "0.7", "--ff-bias", "15", "--ff-lag", "60", "--xte-noise", "2", "--noise-seed", "1"], 16.88, 53.12),
    *[(f"noise_seed{seed}", 3, ["--xte-noise", "2", "--noise-seed", str(seed)], rms, maximum)
      for seed, rms, maximum in [(1, 2.67, 12.91), (2, 2.96, 19.11), (3, 2.58, 12.41), (4, 2.56, 16.38), (5, 2.98, 15.67)]],
]


def timestamp():
    return datetime.now(LOCAL_TIME).isoformat(timespec="seconds")


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def dump_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def run_case(case):
    name, group, extra, expected_rms, expected_max = case
    folder = ROOT / "runs" / name
    folder.mkdir(parents=True, exist_ok=False)
    command = [str(EXE), "--batch", "--group", str(group), "--out", str(folder), "--max-time", "16000", *extra]
    record = {
        "case": name, "group": f"G{group + 1}", "command_argv": command,
        "working_directory": str(PROJECT), "started_at": timestamp(),
        "expected_historical_rms_m": expected_rms, "expected_historical_maximum_m": expected_max,
        "origin": "New simulation run; historical summary used only for comparison",
    }
    dump_json(folder / "run_metadata.json", record)
    start = time.monotonic()
    with (folder / "stdout.txt").open("wb") as stdout, (folder / "stderr.txt").open("wb") as stderr:
        process = subprocess.Popen(command, cwd=PROJECT, stdout=stdout, stderr=stderr)
        record["pid"] = process.pid
        dump_json(folder / "run_metadata.json", record)
        try:
            record["exit_code"] = process.wait(timeout=1800)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=15)
            record["exit_code"] = process.returncode
            record["error"] = "Wall-time limit reached (1800 s)"
    record.update(finished_at=timestamp(), wall_seconds=round(time.monotonic() - start, 3))
    if record["exit_code"] != 0:
        record.setdefault("error", "Simulation exited without successful arrival")
        dump_json(folder / "run_metadata.json", record)
        return record

    try:
        summaries = list(folder.glob("Summary_*.txt"))
        if len(summaries) != 1:
            raise ValueError("Expected one non-empty summary")
        summary = dict(re.findall(r"(\w+)=([^\s]+)", summaries[0].read_text(encoding="utf-8")))
        if summary["arrived"] != "1":
            raise ValueError("Arrival flag not set")
        record["summary"] = summary
        rms, maximum = float(summary["rms_xte_m"]), float(summary["max_abs_xte_m"])
        record["historical_match"] = abs(rms - expected_rms) < 0.005 and abs(maximum - expected_max) < 0.005
        state_file = next(folder.glob("Log_*.csv"))
        force_file = next(folder.glob("FSI_*.csv"))
        with state_file.open(encoding="utf-8", newline="") as stream:
            states = list(csv.DictReader(stream))
        with force_file.open(encoding="utf-8", newline="") as stream:
            forces = list(csv.DictReader(stream))
        if len(states) < 1000 or len(states) != len(forces):
            raise ValueError("State/force record count mismatch or incomplete route")
        previous = None
        intervals = []
        for state, force in zip(states, forces):
            if state["sim_time"] != force["sim_time"]:
                raise ValueError("State/force time mismatch")
            values = [float(value) for value in state.values()] + [float(value) for value in force.values()]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("Non-finite state or force value")
            moment = float(state["sim_time"])
            if previous is not None:
                intervals.append(moment - previous)
            previous = moment
        if not all(abs(dt - 0.5) < 1e-6 for dt in intervals):
            raise ValueError("Unexpected recording interval")
        record.update(
            state_records=len(states), force_records=len(forces), record_interval_s=0.5,
            rms_recomputed_from_csv_m=math.sqrt(sum(float(row["xte_m"]) ** 2 for row in states) / len(states)),
            maximum_recomputed_from_csv_m=max(abs(float(row["xte_m"])) for row in states),
            summary_sampling_s=0.5, integrator_step_s=0.1,
        )
    except (ValueError, KeyError, StopIteration) as error:
        record["error"] = str(error)
    record["files"] = {
        path.name: {"bytes": path.stat().st_size, "sha256": sha256(path)}
        for path in sorted(folder.iterdir()) if path.is_file() and path.name != "run_metadata.json"
    }
    dump_json(folder / "run_metadata.json", record)
    return record


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    if (ROOT / "run_status.json").exists():
        raise RuntimeError("This run directory already has a status file; choose a new directory to rerun")
    metadata = {
        "started_at": timestamp(), "executable": str(EXE), "executable_sha256": sha256(EXE),
        "private_chart_input": str(CHART), "chart_sha256": sha256(CHART),
        "workers": 4, "case_count": len(CASES),
        "control_step_s": 0.1, "state_force_and_summary_sampling_s": 0.5,
        "note": "Summary statistics are accumulated once per onTimerTick (five 0.1 s steps).",
    }
    dump_json(ROOT / "suite_metadata.json", metadata)
    lane_command = [str(EXE), "--batch", "--dump-lane", str(ROOT / "planned_route.csv")]
    with (ROOT / "lane_stdout.txt").open("wb") as stdout, (ROOT / "lane_stderr.txt").open("wb") as stderr:
        subprocess.run(lane_command, cwd=PROJECT, stdout=stdout, stderr=stderr, check=True, timeout=120)
    status = {"total": len(CASES), "completed": [], "running_or_queued": [case[0] for case in CASES]}
    dump_json(ROOT / "run_status.json", status)
    last_print = time.monotonic()
    print(f"Starting {len(CASES)} full-route cases with four workers.", flush=True)
    with ThreadPoolExecutor(max_workers=4) as executor:
        pending = {executor.submit(run_case, case): case[0] for case in CASES}
        while pending:
            completed, _ = wait(pending, timeout=5, return_when=FIRST_COMPLETED)
            for future in completed:
                name = pending.pop(future)
                try:
                    result = future.result()
                except Exception as error:
                    result = {"case": name, "error": repr(error)}
                status["completed"].append(result)
                status["running_or_queued"].remove(name)
                dump_json(ROOT / "run_status.json", status)
                values = result.get("summary", {})
                print(f"Completed {len(status['completed'])}/{len(CASES)}: {name}; RMS={values.get('rms_xte_m')}; maximum={values.get('max_abs_xte_m')}; match={result.get('historical_match')}; error={result.get('error')}", flush=True)
            if time.monotonic() - last_print >= 30:
                print(f"Progress: {len(status['completed'])}/{len(CASES)} completed; {len(pending)} running or queued.", flush=True)
                last_print = time.monotonic()
    status["finished_at"] = timestamp()
    status["all_passed"] = all(row.get("historical_match") and not row.get("error") for row in status["completed"])
    dump_json(ROOT / "run_status.json", status)
    with (ROOT / "rerun_comparison.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["case", "group", "historical_rms_m", "rerun_rms_m", "historical_maximum_m", "rerun_maximum_m", "matches", "exit_code", "state_records"])
        for row in sorted(status["completed"], key=lambda item: item["case"]):
            summary = row.get("summary", {})
            writer.writerow([row["case"], row.get("group"), row.get("expected_historical_rms_m"), summary.get("rms_xte_m"), row.get("expected_historical_maximum_m"), summary.get("max_abs_xte_m"), row.get("historical_match"), row.get("exit_code"), row.get("state_records")])
    print(f"All cases completed. Historical comparisons passed: {status['all_passed']}", flush=True)
    raise SystemExit(0 if status["all_passed"] else 1)


if __name__ == "__main__":
    main()
