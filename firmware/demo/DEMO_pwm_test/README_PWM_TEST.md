# PWM 直线测速测试：30% × 1 秒

本固件用于测量固定 PWM 对应的实际轮速。上电电机保持 0%，在 E35 上位机执行 IMU 零偏标定，等待 ARMED 后点击 START；左右电机立即输出固定 **30%**，舵机保持 **99° 中位**。持续 **1000 ms** 后直接将两路 PWM 写为 **0%**，状态进入 DONE。没有减速斜坡、速度闭环、起步增益或路径转向；机械滑行是否停止由车辆实际阻力决定，PWM 归零不代表车轮瞬间停转。固定舵机中位是直线开环指令，不能保证轮胎/连杆不对称时没有偏移。

1. Keil 打开 `mdk/rt1064.uvprojx`，选择 `nor_sdram_zf_dtcm`，编译并下载。
2. 连接原 E35 上位机，保持车体静止，点击 IMU 零偏标定。
3. 等待 ARMED，点击 START。上位机记录 30% PWM 下的左右轮速。
4. 1 秒后固件自动置零 PWM；DONE 后继续回传编码器/IMU 及滑行速度。
5. 下一次测试先点击 STOP 回到 ARMED，等待车轮静止后再点击 START。RUN 中重复 START 不延长脉冲。STOP 可以随时提前结束测试。

配置位于 `user/inc/pwm_test_config.h`：`PWM_TEST_DUTY_MILLI_PERCENT=30000`（30%），`PWM_TEST_DURATION_MS=1000`（1 秒）。`vehicle_params.h` 将本测试上限设为 30%，因此不会被导航版本的 25% 限幅截断。此版本不读取或修改 `path_planner/vehicle_config.json`，编译也不会重新生成路径。舵机最大行程仍限制为 ±15°，本测试只使用中位。

脉冲终点由 5 ms PIT 定时中断触发高优先级 PendSV，计时基于本机节拍，不依赖无线链路或电脑计时；时间分辨率为 5 ms。保留 STOP、IMU 有效性检查、发车静止检查、控制任务超时保护。DEBUG 被拒绝，JOY 不控制电机/舵机。因本测试为 1 秒固定开环脉冲，不按轮速修改占空比，也不触发原 2 秒堵转判断；故障仍可提前切断输出。

上位机遥测 200 Hz；CSV 的 `duty_l/duty_r` 为输出指令百分比，`vl/vr` 为滤波后的编码器轮速（m/s），`time_s/tick` 可定位 30% 上升沿到 0% 下降沿。`speed_target=0` 代表此测试没有速度目标，不代表实测轮速为 0。诊断帧还包含累计编码器计数，可用于复核换算比例。波形和真实车速仍需实车测量。

输出为 `mdk/Objects/rt1064_pwm_test.axf`、`.hex`、`.bin`。BIN 后处理需要 `fromelf.exe` 在 PATH 中；在本机也自动查找 `E:/CARcar/keil` 对应相对目录。下载前按所接调试器设置 Keil Debug/Utilities。

`verification/run_pwm_test.ps1` 使用 GCC 编译实际状态机/调度器/里程计 C 代码，验证恒定 PWM、定时归零、急停、重复命令、零编码器及滑行遥测。另有 `verification/actuator_pwm.c` 捕获实际电机输出层 PWM 调用，验证 30% 对应 3000/10000，避免百分比与归一化量混用。

2026-10-26：以上回归检查通过，Keil MDK / ArmClang 6.21 编译为 0 errors / 0 warnings，已生成 AXF/HEX/BIN。未执行板上下载或实车测速。
