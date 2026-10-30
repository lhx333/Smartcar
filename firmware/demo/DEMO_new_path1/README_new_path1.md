# 新路径固件 DEMO_new_path1

本版本使用 `path_planner2/barrels_input.csv` 的 8 个桶坐标规划绕桶、掉头和回程，不再使用原 3 m + U 型弯 + 3 m 路径。当前轨迹为 2187 点，长约 54.557 m，停车参考点为发车局部坐标 (-0.6, 2.0) m。

Python 在电脑上搜索几何路径、平滑并规划速度/前视；完整结果导出为 C 常量表。小车运行时由 C 程序执行编码器/IMU 航迹推算、动态前视 Pure Pursuit 转向、角速度阻尼、轨迹速度插值和每轮 PI 调速，不需要连接 Python。

## 参数与重新编译

参数唯一输入为 `path_planner2/vehicle_config.json`：

| 键 | 当前值 | 含义 |
| --- | --- | --- |
| `pwm_start_percent` | 13.0 | 发车首拍及 DEBUG 起步的实际 PWM 百分比 |
| `pwm_max_percent` | 25.0 | 两个电机实际 PWM 百分比上限 |
| `servo_max_deg` | 15.0 | 相对 99° 中位的最大舵机指令偏角；AUTO、DEBUG、自检共用 |
| `wheel_angle_deg` | 15.0 | 满归一化 turn 对应的实际前轮角度假设，用于几何控制，需独立标定 |

当前舵机指令范围是 84°～114°。`servo_max_deg` 是舵机摇臂指令角，`wheel_angle_deg` 是前轮角，两者通过连杆传动；当前均设为 15° 是待实测校准的假设，并不证明连杆传动比为 1。更改前者后应同时复核后者。

起步 13% 是首拍值；自动模式在某轮尚未达到 0.03 m/s 前，以每秒 12 个百分点提高该轮起步 PWM 下限，最高不超过配置的 25%，以克服静摩擦。检测到该轮前进后结束本次起步辅助，速度环可向下调节到 0%。爬升率位于 `my_common.h` 的 `LAUNCH_PWM_RAMP_PERCENT_S`。原 2 秒低轮速/反向轮速保护不变，达到上限仍不动会停车。最大 PWM 支持不超过 100%，起步须大于 0 且不超过最大值。舵机参数须大于 0 且不超过保留的机械标定窗口 ±20°；三个配置最多支持三位小数。

1. 修改 JSON，或修改桶坐标 CSV。
2. 用 Keil 打开本目录 `mdk\rt1064.uvprojx`，目标为 `nor_sdram_zf_dtcm`，ArmClang 6.21。
3. 执行 Build/Rebuild。编译前会自动调用 `prepare_firmware.bat`；输入改变时重新规划，未改变时复用经 SHA256 核对的轨迹，再更新 `vehicle_params.h` 和 `robust_trajectory.h`。
4. 配置不合法、规划失败或生成文件失配会阻止构建。修改 JSON 后必须重新编译、下载，运行中的 MCU 不会直接读取电脑上的 JSON。

也可在终端先运行：

```powershell
python -B path_planner2/prepare_firmware.py --firmware firmware/demo/DEMO_new_path1
```

## 源码与输出

| 文件 | 用途 |
| --- | --- |
| `user/src/navigation.c` | 位姿递推、调用新跟踪器、速度/转向接口和到达判断 |
| `user/inc/robust_control.h` | 局部弧长投影、动态前视、几何转向、阻尼和误差降速 |
| `user/src/state_machine.c` | 新速度表、每轮 PI、起步、停止、超时等控制调度 |
| `user/inc/motor_launch.h` | 自动模式一次性的受限起步 PWM 爬升 |
| `user/src/servo_control.c` | 全部模式共用的最终舵机角度限幅 |
| `user/inc/vehicle_params.h` | 从 JSON 自动生成的执行器参数，勿直接编辑 |
| `user/inc/robust_trajectory.h` | 自动生成的 XY/弧长/曲率/速度/前视完整轨迹表 |
| `firmware_manifest.json` | 输入和导出文件 SHA256、参数与轨迹摘要 |

Keil 输出到 `mdk\Objects\`：`rt1064_new_path1.axf`、`.hex`、`.bin`。AXF 用于 Keil 的下载/调试；BIN 由编译后脚本使用 fromelf 生成，基于原 NOR 链接布局。工程文件保留芯片、启动文件、链接脚本与现有硬件引脚配置。

## 验证与下一步下载

本机 Keil MDK 5.39 / ArmClang 6.21 已编译通过。C 跟踪器的简化自行车模型名义案例约 108.4 s 完成，最大估计横向误差约 2.14 cm；7 项现有回归通过。这些是软件检查，模型没有辨识真实 PWM、电机、轮胎和转向连杆。

当前保留 120 s 运行超时、IMU 超时、堵转停止、STOP 命令和原 E35 协议；没有执行硬件下载。Keil 工程未携带已验证的调试器/Flash 下载算法设置（保留原工程的 `InvalidFlash=1` 状态），实际下载前需在 Debug/Utilities 中选择所接探针和 RT1064 对应 Flash 算法。生成的 AXF/HEX/BIN 不等于已经完成板上下载验证。

上电后电机保持停止，状态为 INIT。E35 上位机连接后先静止执行 `CALIBRATE`；固件完成 IMU 零偏标定并确认传感器数据有效后才进入 ARMED。只有此后收到 `START` 才进入 RUN、输出电机 PWM。提前或条件不满足时发来的 `START` 会收到 `CMD,REJECTED_START,<state>,<error>`，不会排队等标定后自动启动；上位机 START 按钮也仅在 ARMED 时开放。原 `DEMO` 目录保留供原路线使用。

## 2026-10-26 error 4 排查与修复

`SM_ERR_MOTOR_STALL=4` 指运行后某轮持续低速或反向，不是未标定/拒绝 START。上位机日志中 10:58:51、11:00:13、11:02:03 三次均先进入 RUN，约 2 秒后才进入 ERROR/4。期间 PWM 为 13%～14.4%，轮速近零。起点轨迹目标速度仅约 0.089 m/s，未移动时路径进度不变，原 PI 的 PWM 增长过慢，可能尚未克服静摩擦就达到保护期限。此次增加起步辅助，保留首拍、最大 PWM 和停机期限。日志不能单独证明真实引脚存在 PWM 或电机供电正常，实车仍需下载新固件确认。

`verification/run_startup.ps1` 使用 GCC 运行实际状态机、调度器与导航 C 代码，检查未标定不启动、首拍 PWM、无反馈爬升、2 秒堵转停车、前进后结束辅助、STOP 以及同批 READY/START 不提前启动。回归通过；本次 Keil ArmClang 6.21 构建为 0 errors / 0 warnings，日志见 `verification/startup_keil_build.log`，AXF/HEX/BIN 已更新，尚未进行板上下载。
