"""Analyze 30%/1s pulse tests. Run: python analyze_pwm_motion.py

Original CSVs are read-only. Outputs go to motion_analysis/.
Wheel-derived speed is not independently measured ground speed.
"""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parent / 'motion_analysis' / '.mplconfig'))
import json
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'motion_analysis'
DT = .005
STOP_SPEED = .03
STOP_HOLD = .30
SMOOTH_WINDOW = .105
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def sustained(mask, count):
    x = np.convolve(np.asarray(mask, int), np.ones(count, int), 'valid')
    hits = np.flatnonzero(x == count)
    return int(hits[0]) if len(hits) else None


def analyze(path):
    raw = pd.read_csv(path)
    required = ['time_s', 'vl', 'vr', 'duty_l', 'duty_r', 'yaw', 'gz']
    if raw[required].isna().any().any():
        raise ValueError(f'{path.name}: missing essential values; do not fill blindly')
    if (np.diff(raw.time_s) <= 0).any():
        raise ValueError(f'{path.name}: nonmonotonic time / reset; split first')
    active = (raw.duty_l.abs() + raw.duty_r.abs()) > 1
    starts = np.flatnonzero(active & ~active.shift(fill_value=False))
    if len(starts) != 1:
        raise ValueError(f'{path.name}: expected exactly one PWM pulse')
    first = starts[0]
    off = np.flatnonzero((raw.index > first) & ~active)[0]
    origin = raw.time_s.iloc[first]
    end_drive = float(raw.time_s.iloc[off] - origin)
    raw['t'] = raw.time_s - origin
    raw['v'] = (raw.vl + raw.vr) / 2
    # Uniform MCU-time grid; never use bursty host arrival time for derivatives.
    t = np.arange(-.5, float(raw.t.iloc[-1]), DT)
    frame = pd.DataFrame({'t': t})
    for c in ['vl', 'vr', 'v', 'yaw', 'gz']:
        frame[c] = np.interp(t, raw.t, raw[c])
    for c in ['duty_l', 'duty_r']:
        pos = np.clip(np.searchsorted(raw.t, t, side='right')-1, 0, len(raw)-1)
        frame[c] = raw[c].to_numpy()[pos]
    good = np.ones(len(t), bool)
    gaps = np.flatnonzero(np.diff(raw.t) > .025)
    for g in gaps:
        good &= ~((t >= raw.t.iloc[g]-.055) & (t <= raw.t.iloc[g+1]+.055))
    # First rest after power-off; both wheels must stay slow for 0.30 s.
    rest = (abs(frame.vl) < STOP_SPEED) & (abs(frame.vr) < STOP_SPEED) & (t > end_drive+.1)
    stop_index = sustained(rest, round(STOP_HOLD/DT))
    if stop_index is None:
        raise ValueError(f'{path.name}: no confirmed rest; requires manual crop')
    stop_t = t[stop_index]
    nwin = round(SMOOTH_WINDOW/DT) | 1
    frame['v_smooth'] = savgol_filter(frame.v, nwin, 2)
    frame['accel'] = savgol_filter(frame.v, nwin, 2, deriv=1, delta=DT)
    frame['valid_derivative'] = good & (t >= .055) & (t <= stop_t-.055) & (abs(t-end_drive) > .055)
    frame['retained'] = (t >= 0) & (t <= stop_t)
    # Subsequent motion is flagged, but ALL samples after first stop are excluded.
    later = (np.maximum(abs(frame.vl), abs(frame.vr)) > .15) & (t > stop_t+STOP_HOLD)
    human_index = sustained(later, round(.10/DT))
    a = frame[(t >= 0) & (t < end_drive)]
    coast = frame[(t >= end_drive) & (t <= stop_t)]
    valid_coast = frame[frame.valid_derivative & (t > end_drive+.10) & (frame.v_smooth > .10)]
    bins = []
    for lo in np.arange(.1, 2.81, .2):
        rows = valid_coast[(valid_coast.v_smooth >= lo) & (valid_coast.v_smooth < lo+.2)]
        if len(rows) >= 5:
            bins.append(dict(run=path.stem, speed_mps=rows.v_smooth.mean(),
                             decel_mps2=-rows.accel.mean(), samples=len(rows)))
    def v_at(q):
        return float(np.interp(q, t, frame.v_smooth))
    row = dict(run=path.stem, raw_rows=len(raw), duration_s=float(raw.t.iloc[-1]-raw.t.iloc[0]),
        pulse_s=end_drive, pwm_l_percent=float(a.duty_l.median()), pwm_r_percent=float(a.duty_r.median()),
        v_off_mps=v_at(end_drive), v_peak_mps=float(a.v_smooth.max()),
        v_before_off_mps=float(raw.loc[(raw.t>=end_drive-.025)&(raw.t<end_drive),'v'].median()),
        v_coast_010_mps=v_at(end_drive+.10),
        coast_after_010_distance_m=float(np.trapezoid(coast.loc[coast.t>=end_drive+.10,'v'].clip(lower=0),coast.loc[coast.t>=end_drive+.10,'t'])),
        v_010_mps=v_at(.1), v_025_mps=v_at(.25), v_050_mps=v_at(.5), v_075_mps=v_at(.75),
        accel_010_040_mps2=(v_at(.4)-v_at(.1))/.3,
        accel_040_090_mps2=(v_at(.9)-v_at(.4))/.5,
        accel_000_010_mps2=(v_at(.1)-v_at(0))/.1,
        drive_distance_m=float(np.trapezoid(a.v.clip(lower=0), a.t)),
        coast_distance_m=float(np.trapezoid(coast.v.clip(lower=0), coast.t)),
        coast_time_s=float(stop_t-end_drive), stop_t_s=float(stop_t),
        average_coast_decel_mps2=v_at(end_drive)/(stop_t-end_drive),
        yaw_change_drive_deg=float(np.interp(end_drive, t, frame.yaw)-np.interp(0, t, frame.yaw)),
        wheel_speed_diff_drive_mps=float(np.mean(abs(a.vl-a.vr))),
        gaps_over_25ms=int(len(gaps)), maximum_gap_s=float(np.diff(raw.time_s).max()),
        missing_ticks=int(np.maximum(np.diff(raw.tick)-1, 0).sum()),
        largest_gap_retained_s=float(np.diff(raw.loc[(raw.t>=0)&(raw.t<=stop_t), 'time_s']).max()),
        excluded_after_stop_rows=int((raw.t > stop_t).sum()),
        later_motion_t_s=float(t[human_index]) if human_index is not None else None)
    frame.to_csv(OUT / (path.stem+'_processed.csv'), index=False)
    return raw, frame, row, bins


def main():
    OUT.mkdir(exist_ok=True)
    runs = [analyze(p) for p in sorted(ROOT.glob('*.csv'))]
    stats = pd.DataFrame([r[2] for r in runs])
    bins = pd.DataFrame([b for r in runs for b in r[3]])
    # One mean per run/speed bin avoids weighting slow speeds by dwell time.
    def fit(data):
        return least_squares(lambda c: c[0]+c[1]*data.speed_mps-data.decel_mps2,
                             [.2, .6], bounds=(0, np.inf), loss='soft_l1', f_scale=.15).x
    b0,b1 = fit(bins)
    const = bins.decel_mps2.mean()
    def distance(v):
        return v/b1-b0/b1**2*np.log1p(b1*v/b0)
    def time_to_stop(v):
        return np.log1p(b1*v/b0)/b1
    stats['model_coast_time_s'] = time_to_stop(stats.v_off_mps)
    stats['model_coast_distance_m'] = distance(stats.v_off_mps)
    stats.to_csv(OUT/'run_summary.csv', index=False, encoding='utf-8-sig')
    bins.to_csv(OUT/'coast_bins.csv', index=False, encoding='utf-8-sig')
    rng=np.random.default_rng(20261029)
    names=stats.run.to_numpy()
    boot=np.array([fit(pd.concat([bins[bins.run==n] for n in rng.choice(names,len(names))])) for _ in range(300)])
    # Held-out run fit errors: independent repeats, not individual correlated samples.
    cv=[]
    for n in names:
        c=fit(bins[bins.run!=n]); test=bins[bins.run==n]
        cv.append(float(np.sqrt(np.mean((c[0]+c[1]*test.speed_mps-test.decel_mps2)**2))))
    stats['model_after_010_distance_m'] = distance(stats.v_coast_010_mps)-distance(STOP_SPEED)
    stats.to_csv(OUT/'run_summary.csv', index=False, encoding='utf-8-sig')
    table=pd.DataFrame({'speed_mps':[.3,.5,1.,1.5,2.]})
    table['coast_decel_mps2']=b0+b1*table.speed_mps
    table['stop_time_s']=time_to_stop(table.speed_mps)
    table['stop_distance_m']=distance(table.speed_mps)
    table.to_csv(OUT/'coast_reference.csv',index=False,encoding='utf-8-sig')
    summary={'coast_model':'dv/dt=-(b0+b1*v), v>0; clamp to zero at stop',
        'b0_mps2':b0,'b1_per_s':b1,'bootstrap_95pct_b0_b1':np.percentile(boot,[2.5,97.5],axis=0).tolist(),
        'fit_rmse_mps2':float(np.sqrt(np.mean((b0+b1*bins.speed_mps-bins.decel_mps2)**2))),
        'constant_decel_fit_mps2':const,
        'constant_fit_rmse_mps2':float(np.sqrt(np.mean((const-bins.decel_mps2)**2))),
        'leave_one_run_out_rmse_mps2':float(np.mean(cv)),
        'coast_distance_prediction_mae_m':float(abs(stats.coast_distance_m-stats.model_coast_distance_m).mean()),
        'after_010_distance_prediction_mae_m':float(abs(stats.coast_after_010_distance_m-stats.model_after_010_distance_m).mean()),
        'fitted_speed_range_mps':[float(bins.speed_mps.min()),float(bins.speed_mps.max())],
        'threshold_stop_definition_mps':STOP_SPEED,'threshold_hold_s':STOP_HOLD,
        'savgol_window_s':SMOOTH_WINDOW,'savgol_order':2,
        'run_count':len(runs),'total_raw_rows':int(stats.raw_rows.sum()),
        'metrics':stats.select_dtypes('number').agg(['mean','std','min','max']).to_dict()}
    (OUT/'fit_results.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    colors=plt.cm.tab10(np.linspace(0,1,len(runs)))
    fig,ax=plt.subplots(3,1,figsize=(11,10),sharex=True)
    for j,(_,f,r,_) in enumerate(runs):
        q=f[f.retained]; color=colors[j]
        ax[0].plot(q.t,q.v_smooth,color=color,label=f'T{j+1}')
        ax[1].plot(q.t,q.accel.where(q.valid_derivative),color=color,alpha=.7)
        ax[2].step(q.t,(q.duty_l+q.duty_r)/2,color=color,where='post')
    for a in ax: a.grid(alpha=.25); a.axvline(1,color='black',ls='--',lw=1)
    ax[0].set(ylabel='轮速均值 (m/s)',title='10组30% PWM脉冲：按通电时刻对齐，首次确认停车后全部剔除')
    ax[0].legend(ncol=5);ax[1].set(ylabel='轮速导数 (m/s²)');ax[2].set(ylabel='PWM (%)',xlabel='距通电时刻 (s)')
    fig.tight_layout();fig.savefig(OUT/'01_aligned_response.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(5,2,figsize=(14,17))
    for j,(raw,f,r,_) in enumerate(runs):
        a=axs.flat[j];a.plot(raw.t,raw.v,color='0.75',lw=.7,label='原始全段')
        q=f[f.retained];a.plot(q.t,q.v_smooth,color=colors[j],label='保留试验段')
        a.axvspan(r['stop_t_s'],raw.t.iloc[-1],color='orange',alpha=.12)
        a.axvline(0,color='green',ls=':');a.axvline(r['pulse_s'],color='red',ls=':')
        a.set(title=f'T{j+1} {r["run"]}',xlabel='距通电时刻 (s)',ylabel='轮速 (m/s)');a.grid(alpha=.2)
    axs.flat[0].legend();fig.tight_layout();fig.savefig(OUT/'02_crop_audit.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(13,5))
    for j,n in enumerate(names):
        b=bins[bins.run==n]; ax[0].scatter(b.speed_mps,b.decel_mps2,color=colors[j],s=15,label=f'T{j+1}')
    vv=np.linspace(.1,2.2,100);ax[0].plot(vv,b0+b1*vv,'k',label=f'b={b0:.3f}+{b1:.3f}v')
    ax[0].axhline(const,color='gray',ls='--',label='常减速度拟合')
    ax[0].set(xlabel='轮速 (m/s)',ylabel='减速度大小 (m/s²)',title='PWM=0的速度相关减速度');ax[0].legend(ncol=3,fontsize=8)
    ax[1].plot(vv,distance(vv),label='速度相关模型');ax[1].scatter(stats.v_coast_010_mps,stats.coast_after_010_distance_m,label='断电0.1秒后剩余实测距离')
    ax[1].set(xlabel='断电初速度 (m/s)',ylabel='滑行停车距离 (m)',title='同一地面/驱动零输出方式下的参考');ax[1].legend()
    for a in ax:a.grid(alpha=.25)
    fig.tight_layout();fig.savefig(OUT/'03_coast_model.png',dpi=180);plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(12,4.5))
    for j,(_,f,r,_) in enumerate(runs):
        q=f[(f.t>=0)&(f.t<r['pulse_s'])];ax[0].plot(q.t,q.v_smooth,color=colors[j],label=f'T{j+1}')
    ax[0].set(xlabel='通电时间 (s)',ylabel='轮速 (m/s)',title='30% PWM未达到稳定速度')
    ax[1].bar(np.arange(len(runs))-.18,stats.drive_distance_m,width=.36,label='通电阶段')
    ax[1].bar(np.arange(len(runs))+.18,stats.coast_distance_m,width=.36,label='断电滑行')
    ax[1].set(xticks=np.arange(len(runs)),xticklabels=[f'T{i+1}' for i in range(len(runs))],ylabel='轮速积分距离 (m)',title='各次试验距离');ax[1].legend()
    for a in ax:a.grid(alpha=.25)
    fig.tight_layout();fig.savefig(OUT/'04_drive_and_distance.png',dpi=180);plt.close(fig)
    def span(key):
        z=stats[key]; return f'{z.mean():.3f} ± {z.std():.3f}（范围 {z.min():.3f}–{z.max():.3f}）'
    report = f'''# 30% PWM、约1秒脉冲的十组试验分析

## 数据与裁剪

输入为本目录10个原始CSV，共{len(runs)}组、{int(stats.raw_rows.sum())}行，关键字段无空值。
按MCU time_s对齐；v=(vl+vr)/2。两轮PWM均为30%，持续0.995秒后归零。
PWM是日志指令值，不是实测电机端电压。加速度为轮速导数，没有纵向加速度传感器数据。
首次两轮绝对速度均低于0.03 m/s且持续0.30秒，定义为确认停车；停车后的数据全部剔除。
02_crop_audit.png保留灰色完整日志和橙色排除区，供复核人为拿取；T8在起步后约5.965秒再次明显运动。
0.025秒以上采样间隙两侧各0.055秒不参与导数分析，积分距离使用保留区轮速。
导数使用0.105秒窗口、二阶Savitzky–Golay平滑；PWM切换附近±0.055秒不参与导数。
每组精确裁剪点、缺帧和最大间隙见run_summary.csv；逐点标记见各*_processed.csv。

## 实测统计：均值 ± 样本标准差

|指标|结果|
|---|---|
|断电前25ms轮速中位数，m/s|{span('v_before_off_mps')}|
|通电0.1秒轮速，m/s|{span('v_010_mps')}|
|通电0.5秒轮速，m/s|{span('v_050_mps')}|
|通电0–0.1秒平均轮速增长率，m/s²|{span('accel_000_010_mps2')}|
|通电0.1–0.4秒平均轮速增长率，m/s²|{span('accel_010_040_mps2')}|
|通电0.4–0.9秒平均轮速增长率，m/s²|{span('accel_040_090_mps2')}|
|通电阶段轮速积分距离，m|{span('drive_distance_m')}|
|断电到确认停车，s|{span('coast_time_s')}|
|断电滑行轮速积分距离，m|{span('coast_distance_m')}|
|通电期间航向变化，度|{span('yaw_change_drive_deg')}|

轮速在约0.2–0.5秒出现平台，随后继续上升，1秒内没有达到稳态；单一一阶速度模型不适合整个通电过程。
0–0.1秒轮速增长很快，可能含打滑、传动瞬态和测速滤波影响；不能直接当作车身加速度标定。
十组均为同一PWM档，只能给出v(t | PWM=30%)的重复试验参考，不能辨识PWM→稳态速度曲线、最小起动PWM、20%或25%加速度、质量/摩擦系数或可靠PID增益。

## PWM=0的滑行参考模型

忽略断电后前0.10秒的快速瞬态，在约{bins.speed_mps.min():.2f}–{bins.speed_mps.max():.2f} m/s范围，
按每组0.2 m/s速度箱平均减速度后，使用非负稳健拟合：

dv/dt = -(b0 + b1*v)，其中 b0={b0:.4f} m/s²，b1={b1:.4f} s⁻¹。

只适用于前进且v>0，停车后速度钳为0。系数是经验阻力项，不能直接解释为轮胎摩擦系数。
减速度拟合RMSE={summary['fit_rmse_mps2']:.3f} m/s²；常减速度模型RMSE={summary['constant_fit_rmse_mps2']:.3f} m/s²。
每次留出一整组试验验证，平均RMSE={np.mean(cv):.3f} m/s²。
从断电0.1秒后的速度预测剩余停车距离，平均绝对误差={summary['after_010_distance_prediction_mae_m']:.3f} m。
若将模型硬套在断电瞬间，距离误差约{summary['coast_distance_prediction_mae_m']:.3f} m，因此不能覆盖断电最初瞬态。
Bootstrap区间在fit_results.json中，只反映这十组重复性，不覆盖编码器标定误差或换地面的变化。

由该模型到零速度的参考关系：
t_stop=ln(1+b1*v/b0)/b1
s_stop=v/b1-b0*ln(1+b1*v/b0)/b1²

    {table.to_string(index=False, float_format=lambda x: f'{x:.3f}')}

表内低速端到零速度的末尾小段是外推；实际停车门限为0.03 m/s。
规划器若只支持单个brake，必须指定工作速度范围并保守选取，不能把整段平均减速度用于低速停车。
本次没有自动修改vehicle_config.json，也没有把该经验模型应用到固件。
日志速度和距离依赖编码器比例；若打滑或标定不准，真实车速和真实停车距离都会不同。
默认保留所有十次试验，后几次通电期间航向变化达约6度，属于近似直线试验而非严格无转弯。

## 复现

在本目录运行 python analyze_pwm_motion.py；依赖numpy、pandas、scipy、matplotlib。
原始CSV不修改，所有结果写入motion_analysis/。
'''
    (OUT/'analysis_report.md').write_text(report,encoding='utf-8')
    print(stats[['run','pulse_s','v_off_mps','coast_time_s','coast_distance_m','later_motion_t_s','largest_gap_retained_s']].to_string(index=False))
    print(json.dumps({k:v for k,v in summary.items() if k!='metrics'},ensure_ascii=False,indent=2))
    print(table.to_string(index=False))


if __name__=='__main__':
    main()
