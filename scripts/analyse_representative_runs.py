"""Check repeated runs and plot acute-bend and current-prior diagnostics."""

from pathlib import Path
import json
import math
import shutil
import statistics

import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator


REPOSITORY = Path(__file__).resolve().parents[1]
ROOT = REPOSITORY / "recovered_runs" / "representative"
NOMINAL = ROOT.parent / "steady" / "g3"
OUTPUT = REPOSITORY / "diagnostics"
MERC_SCALE = 0.8460443260071842
MERC_R = 6378137.0
KN_TO_MPS = 0.5144
COLORS = ["#253746", "#B5533C", "#2C6E9B", "#3F7E6E", "#8B6B9F"]
plt.rcParams.update({
    "font.family": "Times New Roman", "font.size": 10,
    "axes.labelsize": 10, "axes.titlesize": 11,
    "axes.linewidth": 0.7, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "legend.fontsize": 9, "pdf.fonttype": 42, "ps.fonttype": 42,
    "axes.spines.top": False, "axes.spines.right": False,
    "savefig.facecolor": "white",
})


def load_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8-sig")


LANE = load_csv(ROOT / "planned_route.csv")
POINTS = np.column_stack([LANE["x"], LANE["y"]])
SEGMENTS = np.diff(POINTS, axis=0)
LENGTHS = np.linalg.norm(SEGMENTS, axis=1)
CHAIN = np.r_[0.0, np.cumsum(LENGTHS)]
TREE = cKDTree(POINTS)


def chainage(states):
    longitude = np.deg2rad(states["lon"])
    latitude = np.deg2rad(states["lat"])
    positions = np.column_stack([
        MERC_SCALE * MERC_R * longitude,
        MERC_SCALE * MERC_R * np.log(np.tan(np.pi / 4 + latitude / 2)),
    ])
    _, vertices = TREE.query(positions, k=3)
    candidates = np.clip(np.concatenate([vertices, vertices - 1], axis=1), 0, len(SEGMENTS) - 1)
    start = POINTS[candidates]
    delta = SEGMENTS[candidates]
    fraction = np.clip(np.einsum("ijk,ijk->ij", positions[:, None, :] - start, delta) / (LENGTHS[candidates] ** 2), 0, 1)
    projected = start + fraction[:, :, None] * delta
    distances = np.einsum("ijk,ijk->ij", positions[:, None, :] - projected, positions[:, None, :] - projected)
    best = np.argmin(distances, axis=1)
    rows = np.arange(len(states))
    chosen = candidates[rows, best]
    return CHAIN[chosen] + fraction[rows, best] * LENGTHS[chosen]


def case_data(folder):
    states = load_csv(next(folder.glob("Log_*.csv")))
    forces = load_csv(next(folder.glob("FSI_*.csv")))
    s = chainage(states)
    dx = np.column_stack([forces[f"E{i}_dX"] for i in range(8)])
    dy = np.column_stack([forces[f"E{i}_dY"] for i in range(8)])
    x_local = np.repeat([60.0, 20.0, -20.0, -60.0], 2)
    y_local = np.tile([7.25, -7.25], 4)
    moment = dy @ x_local - dx @ y_local
    return states, s, dx.sum(axis=1), dy.sum(axis=1), moment


def save_figure(fig, name):
    fig.savefig(OUTPUT / f"{name}.png", dpi=300, bbox_inches="tight")
    fig.savefig(OUTPUT / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def report_metrics(name, states, s, force_x, force_y, moment):
    xte = states["xte_m"]
    rudder = states["rudder_deg"]
    peak = int(np.argmax(np.abs(xte)))
    bend = (s >= 35300) & (s <= 37400)
    large_error = np.abs(xte) > 20
    return {
        "case": name, "samples": len(states),
        "rms_from_record_m": float(np.sqrt(np.mean(xte ** 2))),
        "maximum_from_record_m": float(np.max(np.abs(xte))),
        "peak_chainage_km": float(s[peak] / 1000),
        "median_absolute_error_m": float(np.median(np.abs(xte))),
        "fraction_above_20m": float(np.mean(np.abs(xte) > 20)),
        "rudder_total_variation_from_rounded_csv_deg": float(np.sum(np.abs(np.diff(rudder)))),
        "full_route_rudder_ge_34_deg_fraction": float(np.mean(np.abs(rudder) >= 34)),
        "full_route_water_speed_median_mps": float(np.median(states["stw_kn"]) * KN_TO_MPS),
        "full_route_maximum_abs_recorded_crab_angle_deg": float(np.max(np.abs(states["crab_angle_deg"]))),
        "above_20m_rudder_ge_34_deg_fraction": float(np.mean(np.abs(rudder[large_error]) >= 34)) if large_error.any() else None,
        "above_20m_water_speed_median_mps": float(np.median(states["stw_kn"][large_error]) * KN_TO_MPS) if large_error.any() else None,
        "above_20m_chainage_range_km": [float(np.min(s[large_error]) / 1000), float(np.max(s[large_error]) / 1000)] if large_error.any() else None,
        "acute_bend_chainage_km": [35.3, 37.4],
        "acute_bend_rudder_ge_34_deg_fraction": float(np.mean(np.abs(rudder[bend]) >= 34)),
        "acute_bend_water_speed_median_mps": float(np.median(states["stw_kn"][bend]) * KN_TO_MPS),
        "acute_bend_maximum_abs_recorded_crab_angle_deg": float(np.max(np.abs(states["crab_angle_deg"][bend]))),
        "maximum_abs_distributed_surge_correction_N": float(np.max(np.abs(force_x))),
        "maximum_abs_distributed_sway_correction_N": float(np.max(np.abs(force_y))),
        "maximum_abs_distributed_yaw_correction_Nm": float(np.max(np.abs(moment))),
    }


def main():
    status = json.loads((ROOT / "run_status.json").read_text(encoding="utf-8"))
    if not status.get("all_passed") or len(status["completed"]) != 15:
        raise RuntimeError("All fifteen runs must finish and match historical summaries before analysis")
    data = {}
    metrics = []
    OUTPUT.mkdir(exist_ok=True)
    derived = OUTPUT / "derived"
    derived.mkdir(exist_ok=True)
    for record in sorted(status["completed"], key=lambda item: item["case"]):
        name = record["case"]
        states, s, force_x, force_y, moment = case_data(ROOT / "runs" / name)
        data[name] = (states, s)
        metric = report_metrics(name, states, s, force_x, force_y, moment)
        summary = record["summary"]
        metric["summary_rms_m"] = float(summary["rms_xte_m"])
        metric["summary_maximum_m"] = float(summary["max_abs_xte_m"])
        metric["summary_rudder_total_variation_deg"] = float(summary["sum_abs_rudder_change_deg"])
        metric["csv_summary_rms_difference_m"] = metric["rms_from_record_m"] - metric["summary_rms_m"]
        metrics.append(metric)
        np.savetxt(
            derived / f"{name}_route_and_loads.csv",
            np.column_stack([states["sim_time"], s, force_x, force_y, moment]),
            delimiter=",", fmt="%.6f", comments="",
            header="sim_time,route_chainage_m,distributed_surge_N,distributed_sway_N,distributed_yaw_Nm",
        )
    nominal_states, nominal_s, nominal_x, nominal_y, nominal_n = case_data(NOMINAL)
    data["nominal"] = (nominal_states, nominal_s)
    metrics.append(report_metrics("nominal", nominal_states, nominal_s, nominal_x, nominal_y, nominal_n))
    (OUTPUT / "diagnostic_metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.2), sharex=True, layout="constrained")
    bend_cases = [
        ("nominal", "Nominal, command 94.5", COLORS[0], "--"),
        ("derivative_g4_seed7_q94_5", "Seed 7, command 94.5", COLORS[1], "-"),
        ("derivative_g4_seed7_q100", "Seed 7, command 100", COLORS[2], "-"),
        ("derivative_g4_seed7_q105", "Seed 7, command 105", COLORS[3], "-"),
    ]
    for name, label, color, linestyle in bend_cases:
        states, s = data[name]
        select = (s >= 33500) & (s <= 39000)
        for axis, values in zip(axes, [states["xte_m"], states["rudder_deg"], states["stw_kn"] * KN_TO_MPS]):
            axis.plot(s[select] / 1000, values[select], color=color, linestyle=linestyle, linewidth=1.2, label=label)
    for axis, title, ylabel in zip(axes, ["(a) True cross-track error", "(b) Rudder demand", "(c) Water-relative speed"], ["Cross-track error (m)", "Rudder angle (deg)", "Water speed (m/s)"]):
        axis.axvspan(35.3, 37.4, color="#E8ECEF", zorder=-1)
        axis.set_title(title, loc="left", fontweight="normal")
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", color="#DEE3E7", linewidth=0.5)
        axis.set_xlim(33.5, 39)
    axes[0].legend(loc="upper left", ncol=2, frameon=False)
    for bound in [-35, 35]:
        axes[1].axhline(bound, color="#7B8794", linestyle=":", linewidth=0.8)
    axes[1].set_ylim(-40, 40)
    axes[2].set_xlabel("Route chainage (km)")
    save_figure(fig, "seed7_acute_bend")

    fig, axes = plt.subplots(3, 1, figsize=(8.0, 7.2), sharex=True, layout="constrained")
    comparisons = [
        [("prior_gain070", "Current magnitude -30%", COLORS[1]), ("prior_gain130", "Current magnitude +30%", COLORS[2])],
        [("prior_bias_p15", "Direction bias +15 deg", COLORS[1]), ("prior_bias_m15", "Direction bias -15 deg", COLORS[2])],
        [("prior_lag60", "Prior age 60 s", COLORS[2]), ("prior_combined", "Combined errors", COLORS[1])],
    ]
    for axis, cases, title in zip(axes, comparisons, ["(a) Current magnitude", "(b) Current direction", "(c) Prior age and combined errors"]):
        axis.plot(nominal_s[::4] / 1000, nominal_states["xte_m"][::4], color=COLORS[0], linestyle="--", linewidth=1.0, label="Exact current prior")
        for name, label, color in cases:
            states, s = data[name]
            axis.plot(s[::4] / 1000, states["xte_m"][::4], color=color, linewidth=1.1, label=label)
        axis.axhline(0, color="#B7C0C9", linewidth=0.6, zorder=-1)
        axis.set_title(title, loc="left", fontweight="normal")
        axis.set_ylabel("Cross-track error (m)")
        axis.grid(axis="y", color="#DEE3E7", linewidth=0.5)
        axis.legend(loc="upper right", ncol=3, frameon=False)
        axis.set_xlim(0, CHAIN[-1] / 1000)
        axis.set_ylim(-65, 65)
    axes[-1].set_xlabel("Route chainage (km)")
    save_figure(fig, "current_prior_comparison")

    by_name = {row["case"]: row for row in metrics}
    seed = by_name["derivative_g4_seed7_q94_5"]
    raised = by_name["derivative_g4_seed7_q105"]
    comparison_rows = []
    for record in sorted(status["completed"], key=lambda item: item["case"]):
        comparison_rows.append(f"| {record['case']} | {record['summary']['rms_xte_m']} | {record['summary']['max_abs_xte_m']} | 一致 |")
    noise_mean = statistics.mean(float(row["summary"]["rms_xte_m"]) for row in status["completed"] if row["case"].startswith("noise_seed"))
    report = f"""# 代表性仿真复核结果

本轮于 2026 年 10 月 13 日使用留存可执行程序和海图输入重新完成 15 个全航程工况。全部抵达终点，进程退出码均为 0，RMS 与最大偏差均在原报告精度下匹配历史结果。每个工况保存了命令、程序与输入校验信息、标准输出、状态 CSV 和八单元力 CSV。

## 与历史结果对照

| 工况 | RMS（m） | 最大偏差（m） | 历史对照 |
|---|---:|---:|---|
{chr(10).join(comparison_rows)}

五个量测噪声种子的 RMS 均值为 {noise_mean:.2f} m，与论文一致。新运行记录与从会话恢复的历史摘要分开存储；不能把本轮文件写成原始历史文件。

## 急弯诊断

种子 7、推进命令 94.5 的最大偏差出现在里程 {seed['peak_chainage_km']:.2f} km。全程绝对偏差中位数为 {seed['median_absolute_error_m']:.2f} m，超过 20 m 的记录占 {seed['fraction_above_20m']*100:.2f}%。

在 35.3–37.4 km 区间，命令 94.5 时记录舵角达到或超过 34° 的占比为 {seed['acute_bend_rudder_ge_34_deg_fraction']*100:.1f}%，对水航速中位数为 {seed['acute_bend_water_speed_median_mps']:.2f} m/s；命令 105 时相应为 {raised['acute_bend_rudder_ge_34_deg_fraction']*100:.1f}% 和 {raised['acute_bend_water_speed_median_mps']:.2f} m/s。两种推进命令下的导数扰动相同。曲线支持急弯中舵角饱和与航速下降同时出现，且提高该推进命令能在这一案例中改善偏差；不据此断言唯一因果或一般稳定性保证。

历史分析还使用了全程绝对偏差超过 20 m 的条件样本。这些样本中的近限舵角占比，命令 94.5 与 105 分别为 {seed['above_20m_rudder_ge_34_deg_fraction']*100:.1f}% 与 {raised['above_20m_rudder_ge_34_deg_fraction']*100:.1f}%，对水航速中位数分别为 {seed['above_20m_water_speed_median_mps']:.2f} 与 {raised['above_20m_water_speed_median_mps']:.2f} m/s。该条件样本不等于固定的急弯区间，且两种命令的样本位置不完全相同。全程最大记录蟹行角分别为 {seed['full_route_maximum_abs_recorded_crab_angle_deg']:.2f}° 与 {raised['full_route_maximum_abs_recorded_crab_angle_deg']:.2f}°。论文中将该角度称为“要求的前馈蟹行角”、将条件样本占比称为“急弯区间占比”均需校正。

`seed7_acute_bend.png/pdf` 展示偏差、舵角与对水航速；`current_prior_comparison.png/pdf` 展示六种先验退化情况。阴影区间为上述急弯核查区间。推进命令数值不是实际桨轴 rpm。记录中的蟹行角为船首向减航迹向，不等于控制器要求的前馈蟹行角。

## 数据字段与统计口径

物理积分步长为 0.1 s，每帧包含五个子步；状态和单元力 CSV 每 0.5 s 记录。`batch_runner.cpp` 的 RMS、最大偏差和舵角总变化也在每帧结束时采样，因此其统计间隔为 0.5 s，而非此前文字写的 0.1 s。到达终点的最后一帧写入 CSV 后被排除于汇总循环，且舵角 CSV 保留两位小数，因此精确重算需注意末帧和舍入差异。此处是文字口径校正，不重新定义或更改既有实验指标。

`derived/` 内的航线里程来自记录经纬度在计划折线上的最近投影。总扰动力由八单元力求和，总扰动首摇力矩按单元局部坐标计算 Σ(xᵢ dYᵢ − yᵢ dXᵢ)，这些是由记录派生的数据，并非另一次水动力求解。全部采用 4×2 划分，八单元日志完整覆盖该划分；本次没有用固定八槽日志声称记录更细划分的全部单元。

## 公开数据完整性

本次补齐种子 7 配对与命令余量、六个先验误差工况、五个噪声种子的过程日志。其他 15 个扰动种子的配对过程、四组四潮时、单元细化、增益与命令扫描的全部时间序列仍未补齐。本轮不应称为论文全部实验的完整时间序列数据集。
"""
    (OUTPUT / "复核结果说明.md").write_text(report, encoding="utf-8")
    print(f"15 repeated cases verified; five-seed mean RMS {noise_mean:.2f} m")
    print(f"Seed 7 acute bend: near-limit rudder {seed['acute_bend_rudder_ge_34_deg_fraction']*100:.1f}%; median water speed {seed['acute_bend_water_speed_median_mps']:.2f} m/s")


if __name__ == "__main__":
    main()
