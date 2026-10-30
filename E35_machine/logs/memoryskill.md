# Short-term Workbench

- 2026-10-29：完成本目录十组30% PWM、0.995秒脉冲试验分析。
- 10个原始CSV、71999行；time_s单调，关键字段无空值；保留段最大采样间隙15–30ms。
- 脚本 analyze_pwm_motion.py；原始日志不修改；结果 motion_analysis/analysis_report.md。
- 首次两轮绝对轮速均<0.03m/s且持续0.30s后全部剔除；T8可见后续拿取运动。
- 图：motion_analysis/01_aligned_response.png、02_crop_audit.png、03_coast_model.png、04_drive_and_distance.png。
- 断电前轮速2.633±0.037m/s；滑行时间2.402±0.059s；滑行轮速积分距离2.258±0.048m。
- 断电0.1s后，约0.18–2.19m/s范围：dv/dt=-(0.3865+0.6527v)，减速度留一组验证RMSE约0.119m/s²。
- 轮速为编码器估计，不能等同独立地速；只有30%一档，不能标定完整PWM曲线。未更改规划器或固件配置。
- 2026-10-30：分析 `20261030_192757_265134.csv` 的模块1完整试验。分析时源文件仍在记录；读取快照115092行、关键列无缺失。上位机时间段反复重置，按MCU tick去重/对齐，RUN理论2060拍、保留1977拍、缺83拍；窗口重复拍号1914行、重复值无冲突。
- 独立脚本 `analysis_20261030_192757/analyze.py`，报告 `analysis_report.md`，图 `01_pwm_speed_accel.png`、`02_phase_and_pwm_speed.png`、`03_wheel_balance_imu.png`，另有 `phase_summary.csv`、`trial_deduplicated.csv`、`metrics.json`。原始CSV不改。
- 20%保持3s后轮速均值约1.639m/s，末0.5s仍以约0.333m/s²上升；15%保持3s后约1.435m/s，末0.5s斜率约-0.010m/s²；10.3s完成任务进入DONE。约9.4s轮速骤降且角速度尖峰，疑似外部扰动，不用于自然减速拟合。轮速是编码器估计、PWM是指令值。
