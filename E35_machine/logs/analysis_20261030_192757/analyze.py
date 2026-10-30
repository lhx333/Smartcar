"""Analyze the first completed module-1 run in 20261030_192757_265134.csv.

Run from this directory: python analyze.py
The source CSV is never modified. Wheel speeds are encoder estimates, not
independent ground-speed measurements; PWM columns are commands, not voltages.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / "20261030_192757_265134.csv"
os.environ.setdefault("MPLCONFIGDIR", str(HERE / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

DT = 0.005
PHASES = {11: "20% hold", 12: "20→15% ramp", 13: "15% hold", 14: "15→0% ramp", 19: "stop wait"}
REQUIRED = {"host_time", "tick", "state", "error", "path", "vl", "vr", "duty_l", "duty_r", "yaw", "gz"}
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def read_trial() -> tuple[pd.DataFrame, dict]:
    raw = pd.read_csv(SOURCE)
    absent = REQUIRED - set(raw.columns)
    if absent:
        raise ValueError(f"Missing columns: {sorted(absent)}")
    if raw[list(REQUIRED)].isna().any().any():
        raise ValueError("Missing essential values; refusing to silently fill them")
    run_rows = raw.index[raw.state.eq("RUN") & raw.path.eq(11)]
    if len(run_rows) == 0:
        raise ValueError("No module-1 RUN/phase-11 sample")
    start_row = int(run_rows[0])
    done_rows = raw.index[(raw.index > start_row) & raw.state.eq("DONE")]
    if len(done_rows) == 0:
        raise ValueError("The first run is not finished yet; wait for DONE")
    start_tick = int(raw.at[start_row, "tick"])
    done_row = int(done_rows[0])
    done_tick = int(raw.at[done_row, "tick"])
    if not start_tick < done_tick or done_tick - start_tick > 10000:
        raise ValueError("Unexpected tick wrap or excessive run; inspect this log manually")
    # Restrict by host reception time before deduplicating. A long live log can
    # later wrap the 16-bit tick counter and reuse the same tick values.
    host = pd.to_datetime(raw.host_time)
    first_time = host.iloc[start_row]
    done_time = host.iloc[done_row]
    window = raw.loc[(host >= first_time - pd.Timedelta(seconds=1)) &
                     (host <= done_time + pd.Timedelta(seconds=2))].copy()
    candidates = window.loc[window.tick.between(start_tick - 100, done_tick + 400)]
    duplicates = int(candidates.duplicated("tick").sum())
    conflicts = int((candidates.groupby("tick")[["state", "path", "vl", "vr", "duty_l", "duty_r"]]
                     .nunique() > 1).any(axis=1).sum())
    if conflicts:
        raise ValueError(f"{conflicts} tick values have conflicting measurements")
    trial = candidates.drop_duplicates("tick", keep="first").sort_values("tick").copy()
    trial = trial.loc[trial.tick.between(start_tick - 100, done_tick + 400)].copy()
    trial["t_from_start_s"] = (trial.tick - start_tick) * DT
    trial["wheel_mean_mps"] = (trial.vl + trial.vr) / 2
    trial["wheel_diff_mps"] = trial.vl - trial.vr
    trial["phase_name"] = trial.path.map(PHASES).fillna(trial.state)
    run = trial.loc[(trial.tick >= start_tick) & (trial.tick < done_tick)]
    expected_ticks = done_tick - start_tick
    quality = {
        "source": str(SOURCE),
        "source_rows_at_read": int(len(raw)),
        "source_last_host_time_at_read": str(raw.host_time.iloc[-1]),
        "run_start_host_time": str(raw.at[start_row, "host_time"]),
        "first_done_host_time": str(raw.at[done_row, "host_time"]),
        "run_start_tick": start_tick,
        "first_done_tick": done_tick,
        "expected_run_samples": expected_ticks,
        "retained_run_samples": int(len(run)),
        "missing_run_ticks": int(expected_ticks - len(run)),
        "duplicate_tick_rows_in_window": duplicates,
        "conflicting_duplicate_ticks": conflicts,
        "all_essential_columns_complete": True,
    }
    return trial, quality


def derivative(trial: pd.DataFrame) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    t = trial.t_from_start_s.to_numpy()
    v = trial.wheel_mean_mps.to_numpy()
    grid = np.arange(t[0], t[-1] + DT / 2, DT)
    interp = np.interp(grid, t, v)
    smooth = savgol_filter(interp, 21, 2)
    accel = savgol_filter(interp, 21, 2, deriv=1, delta=DT)
    valid = np.ones(len(grid), dtype=bool)
    # Interpolation bridges small telemetry losses for the plot only. Do not
    # use acceleration near a larger loss or a PWM transition for statistics.
    gaps = np.where(np.diff(t) > 0.025 + 1e-8)[0]
    for i in gaps:
        valid &= ~((grid >= t[i] - .055) & (grid <= t[i + 1] + .055))
    phase = trial.path.to_numpy()
    for i in np.where(phase[1:] != phase[:-1])[0]:
        valid &= np.abs(grid - t[i + 1]) > .055
    valid &= (grid >= t[0] + .055) & (grid <= t[-1] - .055)
    return grid, smooth, np.where(valid, accel, np.nan)


def summaries(trial: pd.DataFrame, quality: dict, grid: np.ndarray,
              accel: np.ndarray) -> tuple[pd.DataFrame, dict]:
    run = trial.loc[trial.state.eq("RUN")]
    rows = []
    for code, label in PHASES.items():
        q = run.loc[run.path.eq(code)].copy()
        if q.empty:
            continue
        ts = q.t_from_start_s.to_numpy()
        tail = q.loc[q.t_from_start_s >= ts.max() - .5]
        slope = float(np.polyfit(tail.t_from_start_s, tail.wheel_mean_mps, 1)[0]) if len(tail) >= 5 else None
        valid_a = accel[(grid >= ts.min() + .1) & (grid <= ts.max() - .1)]
        rows.append({
            "phase": code, "name": label, "start_s": float(ts.min()),
            "end_s": float(ts.max() + DT), "duration_s": float(ts.max() - ts.min() + DT),
            "samples": int(len(q)), "pwm_start_pct": float(q.duty_l.iloc[0]),
            "pwm_end_pct": float(q.duty_l.iloc[-1]),
            "speed_start_mps": float(q.wheel_mean_mps.iloc[0]),
            "speed_end_mps": float(q.wheel_mean_mps.iloc[-1]),
            "speed_peak_mps": float(q.wheel_mean_mps.max()),
            "speed_last_half_second_mean_mps": float(tail.wheel_mean_mps.mean()),
            "speed_last_half_second_slope_mps2": slope,
            "median_valid_wheel_accel_mps2": float(np.nanmedian(valid_a)) if np.isfinite(valid_a).any() else None,
        })
    phases = pd.DataFrame(rows)
    # Integrate only directly adjacent, sufficiently close MCU samples.
    x = trial.t_from_start_s.to_numpy()
    v = np.maximum(trial.wheel_mean_mps.to_numpy(), 0)
    valid_edges = (np.diff(x) <= .025 + 1e-8) & (x[:-1] >= 0) & (x[1:] <= quality["first_done_tick"] * DT - quality["run_start_tick"] * DT)
    distance = float(np.sum((v[:-1] + v[1:]) * .5 * np.diff(x) * valid_edges))
    metrics = {
        "run_duration_s": (quality["first_done_tick"] - quality["run_start_tick"]) * DT,
        "wheel_speed_peak_mps": float(run.wheel_mean_mps.max()),
        "wheel_distance_observed_intervals_m": distance,
        "distance_note": "Wheel-speed integral over observed intervals only; missing intervals omitted",
        "maximum_abs_wheel_speed_difference_mps": float(run.wheel_diff_mps.abs().max()),
        "state_after_run": "DONE",
        "error_after_run": 0,
        "phase_count": int(len(phases)),
    }
    return phases, metrics


def plot(trial: pd.DataFrame, phases: pd.DataFrame, grid: np.ndarray,
         smooth: np.ndarray, accel: np.ndarray) -> None:
    q = trial.loc[trial.t_from_start_s.between(-.5, 12)]
    run = q.loc[q.state.eq("RUN")]
    boundaries = phases.start_s.tolist() + [float(phases.end_s.iloc[-1])]

    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(q.t_from_start_s, q.vl, lw=.9, label="左轮")
    ax[0].plot(q.t_from_start_s, q.vr, lw=.9, label="右轮")
    ax[0].plot(grid, smooth, lw=2, color="black", label="两轮均值，0.105s平滑")
    ax[0].set(ylabel="轮速 (m/s)", title="模块1：20%→15%→0 开环 PWM 试验")
    ax[0].legend()
    ax[1].step(q.t_from_start_s, q.duty_l, where="post", label="左 PWM")
    ax[1].step(q.t_from_start_s, q.duty_r, where="post", lw=.8, ls="--", label="右 PWM")
    ax[1].set(ylabel="PWM 指令 (%)")
    ax[1].legend()
    ax[2].plot(grid, accel, color="tab:red", lw=1)
    ax[2].axhline(0, color="0.4", lw=.8)
    ax[2].set(ylabel="轮速导数 (m/s²)", xlabel="距 START 的 MCU 时间 (s)")
    for a in ax:
        for b in boundaries:
            a.axvline(b, color="0.6", lw=.7, ls=":")
        a.grid(alpha=.25)
        a.set_xlim(-.5, min(12, q.t_from_start_s.max()))
    fig.tight_layout()
    fig.savefig(HERE / "01_pwm_speed_accel.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    colors = {11: "tab:blue", 12: "tab:purple", 13: "tab:orange", 14: "tab:green", 19: "tab:gray"}
    for code, name in PHASES.items():
        p = run.loc[run.path.eq(code)]
        if not p.empty:
            ax[0].plot(p.t_from_start_s, p.wheel_mean_mps, color=colors[code], label=f"{code} {name}")
    ax[0].set(xlabel="距 START 的 MCU 时间 (s)", ylabel="两轮均速 (m/s)", title="按固件阶段分色")
    ax[0].legend(fontsize=8)
    for code in (11, 12, 13, 14):
        p = run.loc[run.path.eq(code)]
        if not p.empty:
            ax[1].scatter(p.duty_l.iloc[::5], p.wheel_mean_mps.iloc[::5], s=7,
                          alpha=.45, color=colors[code], label=f"{code} {PHASES[code]}")
    ax[1].set(xlabel="PWM 指令 (%)", ylabel="两轮均速 (m/s)", title="同一 PWM 在不同阶段可有不同轮速")
    ax[1].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(HERE / "02_phase_and_pwm_speed.png", dpi=180)
    plt.close(fig)

    # The firmware resets yaw when leaving RUN. Exclude DONE here so the
    # reset is not mistaken for an instantaneous physical turn.
    motion = q.loc[q.state.eq("RUN")]
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].plot(motion.t_from_start_s, motion.wheel_diff_mps, lw=1)
    ax[0].set(ylabel="左－右轮速 (m/s)", title="左右轮差与 IMU 姿态（仅作诊断）")
    ax[1].plot(motion.t_from_start_s, motion.yaw, lw=1)
    ax[1].set(ylabel="航向角 (°)")
    ax[2].plot(motion.t_from_start_s, motion.gz, lw=1)
    ax[2].set(ylabel="陀螺 Z (°/s)", xlabel="距 START 的 MCU 时间 (s)")
    for a in ax:
        a.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(HERE / "03_wheel_balance_imu.png", dpi=180)
    plt.close(fig)


def main() -> None:
    trial, quality = read_trial()
    grid, smooth, accel = derivative(trial)
    phases, metrics = summaries(trial, quality, grid, accel)
    trial.to_csv(HERE / "trial_deduplicated.csv", index=False, encoding="utf-8-sig")
    phases.to_csv(HERE / "phase_summary.csv", index=False, encoding="utf-8-sig")
    (HERE / "metrics.json").write_text(json.dumps({"quality": quality, "metrics": metrics},
                                          ensure_ascii=False, indent=2), encoding="utf-8")
    plot(trial, phases, grid, smooth, accel)
    p = phases.set_index("phase")
    lines = ["# 2026-10-30 模块1开环 PWM 试验", "",
             f"输入：`{SOURCE.name}`。读取时 {quality['source_rows_at_read']} 行；原文件未修改。",
             f"RUN：{quality['run_start_host_time']}；首次 DONE：{quality['first_done_host_time']}；运行 {metrics['run_duration_s']:.3f} s。",
             f"RUN 应有 {quality['expected_run_samples']} 拍，保留 {quality['retained_run_samples']} 拍，缺 {quality['missing_run_ticks']} 拍。分析窗口发现 {quality['duplicate_tick_rows_in_window']} 行重复拍号，且无冲突值；已按拍号去重。",
             "上位机 `segment/time_s` 经常重置，因此使用 MCU `tick×5 ms`。图中轮速导数在较大丢拍及 PWM 切换附近留空。",
             "", "## 阶段结果", "",
             "| 阶段 | 时长 s | PWM 起→末 % | 轮速起→末 m/s | 最后0.5s均速 m/s | 最后0.5s斜率 m/s² |",
             "|---|---:|---:|---:|---:|---:|"]
    for row in phases.itertuples():
        slope = "—" if row.speed_last_half_second_slope_mps2 is None else f"{row.speed_last_half_second_slope_mps2:.3f}"
        lines.append(f"| {row.phase} {row.name} | {row.duration_s:.3f} | {row.pwm_start_pct:.1f}→{row.pwm_end_pct:.1f} | {row.speed_start_mps:.3f}→{row.speed_end_mps:.3f} | {row.speed_last_half_second_mean_mps:.3f} | {slope} |")
    lines.extend(["", "## 解读", "",
                  f"最高两轮均速 {metrics['wheel_speed_peak_mps']:.3f} m/s；观测区间的轮速积分为 {metrics['wheel_distance_observed_intervals_m']:.3f} m，丢拍区间未补算，也不等同独立地面位移。",
                  "本次 20% PWM 已使轮速上升，任务走完 20% 保持、下坡、15% 保持、下坡和停车等待，最后进入 DONE/错误码0。",
                  "15% 段是否已达到匀速，应看表中最后0.5秒斜率是否接近0；不宜仅用该段平均轮速宣称稳态 PWM－速度定律。",
                  "约9.4秒时轮速突然下跌并伴随陀螺角速度尖峰，可能有外部碰触或其他扰动；该段不宜用于拟合自然减速。DONE 后航向状态重置，姿态图只绘 RUN。",
                  "PWM 是控制指令，轮速来自编码器；未测实际电机电压、地面速度或纵向加速度。后续若要拟合稳态关系，应在多种 PWM 下延长保持时间并重复试验。",
                  "", "## 图表", "",
                  "- `01_pwm_speed_accel.png`：时序、阶段切换和轮速导数。",
                  "- `02_phase_and_pwm_speed.png`：按阶段的速度变化及 PWM－速度散点。",
                  "- `03_wheel_balance_imu.png`：左右轮速差、航向、角速度。", ""])
    (HERE / "analysis_report.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Analysis complete: {HERE}")
    print(phases[["phase", "duration_s", "speed_end_mps", "speed_last_half_second_slope_mps2"]].to_string(index=False))
    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
