# 修改依据、实车边界与使用说明

控制补丁及轨迹现在位于 `E:\CARcar\demo1\DEMO_new_path1`。`vehicle_config.json` 中的执行器、轨迹及 AUTO 速度 PID 参数通过该工程的编译前脚本同步。详细使用步骤及实际下载限制见固件目录 `README_new_path1.md`。

## 1. 原实现的关键问题

- 原 DP 按路径长度和横向偏移惩罚选优，倾向贴桶；无解会把半径降至约 0.55 m。
- 圆弧与直线仅切线连续，曲率跳变。舵机限速只是延迟实际转向，不能保证车辆还在规划曲线上。
- 固定 0.40 m 前视，固定 1.00 m/s 巡航。原速度环是每侧增量 I：P=0、I=1、D=0，单位为占空比百分点/(m/s*s)。
- 原航向控制 `0.030*角度误差 - 0.004*角速度` 在稳态弯道中也持续用 D 抵消转向；不是完整的几何 Pure Pursuit 转角计算。
- 最多 240 点的导出会使长路线的弯道描述变稀；只换 XY 数组不能实现速度规划。
- 惯导位置由编码器与航向积分产生。PID 不会校正真实世界中的航向零偏和位置漂移；转向 I 也不能把这个不可观测误差消掉。

## 2. 本次实现

默认从请求半径向上搜索三个候选，拒绝缩小半径、环路、漏隙、错误侧别和非外侧回程。起步预留 0.5 m 直线。为平滑预留 3% 半径及 0.12 m 距离，对等弧长点进行高斯平滑，再从实际 XY 重新计算曲率、距离和弧长。平滑后重新检查每段到桶的距离、最小半径、侧别与每个间隙恰好一次穿越。

这是离散平滑和采样曲率约束，不声称生成了严格解析 G2/clothoid 曲线。最终导出全部约 2.5 cm 间距点；原始候选通过点只是搜索参考，平滑路线不必精确穿过该点。

速度约束：

- 横向加速度 `a_y=v²*abs(kappa)`；同时受 `mu*g*friction_use` 限制。给横纵各保留摩擦预算，组合加速度不超过采用的摩擦上限。
- 车轮转角 `delta=atan(wheelbase*kappa)`，限制车轮转角变化率；它不是舵机摇臂角。
- 在 `vmax*delay` 距离内提前取低速限制；反向遍历保证刹车距离，正向遍历限制加速。
- 平滑速度平方，再做保守缩放以满足加速度及横纵 jerk。横向 jerk 包含 `2*v*a*kappa + v³*dkappa/ds` 的离散效果。
- 终点向计时线后延长 0.60 m 再停车。这个长度必须覆盖真实车长、参考点位置和滑行距离；地图中也必须有空间。

曲率或转向变化密集的短直道不会立即全速。默认整个路线可以保持较低速度，避免反复急加减速。算法在有限的可行候选中比较估计用时、转向总变化与净空惩罚，并非全局最优求解器；几何内层仍是原来的 DP，未枚举所有曲线族。

罚则按截图：轻碰 +1 s、撞倒/明显位移 +3 s、漏隙每个 +30 s；停止超过 10 s、冲出场地无法返回为失败。我们把漏隙、碰撞包络和已知场地边界当硬约束；不允许靠罚时换取违规路径。软净空惩罚用 1+3 的权重增强避桶倾向，但不是概率校准的预期罚时。没有碰撞动力学及事故统计，不能捏造“碰倒概率”或认定一次接触必然只罚 1 s。

## 3. 默认参数（均需实测确认）

| 参数 | 默认 | 意义 |
|---|---:|---|
| vmax | 1.00 m/s | 规划允许上限，最终速度更低 |
| accel / brake | 0.35 / 0.45 m/s² | 驱动/制动能力假设 |
| alat | 0.65 m/s² | 横向加速度上限 |
| mu / friction_use | 0.35 / 0.55 | 摩擦系数及采用比例 |
| wheelbase | 0.26 m | 轴距假设 |
| wheel_angle_deg | 15° | 归一化 turn=1 对应的实际车轮角假设，需独立于舵机摇臂标定 |
| pwm_start_percent / pwm_max_percent | 13% / 25% | 发车首拍与驱动电机输出上限 |
| servo_max_deg | 15° | 相对中位的舵机指令偏角上限，所有模式共用 |
| steer_rate_deg_s | 35°/s | 实际车轮转动速度假设 |
| lon_jerk / lat_jerk | 1.0 / 1.2 m/s³ | 加速度变化率 |
| delay | 0.35 s | 提前降速窗口使用的响应时间 |
| body_radius / cone_radius | 0.25 / 0.10 m | 整车与桶的包络半径 |
| tracking_margin | 0.20 m | 跟踪误差余量 |
| initial_position_m | 0.03 m | 发车坐标误差假设 |
| initial_heading_deg | 0.10° | 发车方向误差假设 |
| residual_bias_dps | 0.005°/s | 静态校准后的持续零偏上界假设，未证明 IMU 达到此值 |
| bounds | null | 场地尺寸未知，尚不能认证边界合规 |

质量未单独设定：侧向力 `F=m*v²/R`，摩擦力约 `mu*m*g`，平地简化模型里质量消去。质心高度/轮距决定的侧翻边界、载荷转移、坡度、松动连接和轮胎非线性未建模；需要试车确定 alat，不能据此声称绝不甩出。

当前驱动层只有正向 PWM 调节，没有新增主动反向制动。`brake` 必须用实际松油/制动测试标定；如果实际只能滑行减速，就必须按更低实测减速度重算。

## 4. 控制补丁

补丁基于 `E:\CARcar\step31\Smartcar-main\firmware\demo\DEMO` 的 SHA256 快照，放在本文件夹 `firmware_patch`。现已按请求接入同级 `DEMO_new_path1`；原 `DEMO` 保留原路线。车体模型参数仍有待实测。

- 保持现有 5 ms 周期和状态机的停止/超时保护。
- 弧长局部投影，按距离搜索而非固定六个点，并约束单拍进度，降低误跳回程路径的概率。
- 前视由速度和曲率共同限制在 0.35–0.60 m，变化率限制 0.30 m/s。
- 几何转向 `k_pp=2*y_body/L_chord²`，`turn_geom=atan(Lwheel*k_pp)/wheel_angle_at_turn1`。
- 阻尼改为 `turn=turn_geom + 0.004*(v*k_pp - filtered_gyro)*180/pi`，角速度低通时间常数 30 ms。正常弯道需要的角速度不再被当作扰动一律抑制。
- 不再叠加原 `KP_YAW=0.030` 的航向 P，也不添加航向 I。几何 PP 本身已经提供航向/横向反馈；原增益不能直接与新控制器等同。
- 自动模式每轮使用增量 PID，起调值 P=6、I=3、D=0，从 `vehicle_config.json` 生成到固件。P、I、D 的单位分别为占空比百分点/(m/s)、占空比百分点/(m/s*s)、占空比百分点*s/(m/s)。饱和输出作为下一拍积分状态；保留原轮间占空比差限制。手动调试模式保持原逻辑。
- 横向误差过大或转向逼近饱和时降低速度；超过 0.35 m 跟踪误差或约 70° 航向偏差触发现有故障停止。复用 CONTROL_OVERRUN 故障码，应结合遥测判断，不表示每次都是计算超时。

PID 值是待标定的起调值，未通过真实电机模型辨识。仿真里的速度一阶模型没有模拟电机 PWM、静摩擦和编码器量化，因此不能当作 PID 整定验收。D 项使用误差二阶差分，实车开启前应检查测速噪声。两轮仍用同一中心速度参考，尚未加入轮距对应的内外轮差速分配。

当前工程已经完成 C 控制器集成，后续更新配置或轨迹使用：

```powershell
python -B prepare_firmware.py --firmware "E:\CARcar\demo1\DEMO_new_path1"
```

`prepare_firmware.py` 会核对输入和导出文件 SHA256，并在需要时重算路径；它同步生成头文件，不覆盖手写 C 源码。`apply_firmware_patch.py` 保留为原始基线的历史应用工具，不应重复用于已集成版本。只复制 `path_array.txt` 不会启用速度表。全表约 53 KB 常量数据，已通过新版 Keil 链接；尚未做板上试跑。

## 5. 零偏与鲁棒性验证

误差预算采用 `initial_position + integral((initial_yaw + bias*t) ds)`，对每条路径边检查剩余净空。若配置实际残余零偏为 0.05°/s，当前路线会被拒绝，不通过盲目增大 PID 来伪装解决。轮径比例误差、打滑、震动导致的时变零偏尚需额外预算。

桌面测试使用实际 portable C 控制器（GCC 编译），车辆为简化自行车模型、0.25 s 速度一阶响应、0.10 s 转向响应及可调指令延迟。它不是实车数字孪生。

当前输入的名义路径：长约 54.56 m，最小半径约 1.229 m，最近桶中心距离约 0.761 m，峰值速度约 0.594 m/s；过计时线估计约 101.37 s，全程到停车参考点约 104.13 s。相比原始 51.04 m 路径更长，目标是提高容错；不能把原先恒定 1 m/s 的理想时间与新约束下的时间当作公平的速度提升证明。

更新为满 turn 对应前轮 15° 后，无零偏简化模型完成约 108.4 s，最大估计横向误差约 2.14 cm。完整扰动案例以重新生成的 simulation_report.json 为准；0.05°/s 的既有失败检查仍保留。仿真现在从导出轨迹的 dynamics 读取前轮角、轴距、加减速和转向速率，避免沿用旧的 22° 硬编码。不能据此推广为任意地面都安全。

长路线仅用编码器+陀螺积分，无法从路径误差独立识别真实零偏；静止校准、舵机中位和轮径标定是基本条件。若残余偏置预算无法满足，需要视觉/外部航向/路标等独立观测，或缩短定位重校准间隔。没有传感器接口和观测数据，本补丁没有虚构传感器融合。

## 6. 复现测试

```powershell
python -B path_planner.py --input barrels_input.csv --out out_robust
gcc -std=c99 -O2 -Wall -Wextra -Werror -shared -o firmware_patch/controller_test.dll firmware_patch/host_bridge.c -I "E:/CARcar/demo1/DEMO_new_path1/user/inc" -lm
python -B simulate_tracking.py
python -B -m unittest -v test_robust_planner
python -B plot_results.py
```

7 项回归测试覆盖动态限制、零偏预算拒绝、非法参数、漏隙相关输入/同侧绕行拒绝、三种不等间距布局，以及实际 C 跟踪器正常/失败案例。完整场地仍缺尺寸、车体包络、轮地模型与实时控制板验证；没有声称全赛道扰动都已通过。

## 7. 参考资料

- Coulter, Implementation of the Pure Pursuit Path Tracking Algorithm, CMU-RI-TR-92-01: https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm
- Nav2 Regulated Pure Pursuit 官方实现：速度相关前视、曲率及接近障碍降速。https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md
- Macenski et al., Regulated Pure Pursuit for Robot Path Tracking：https://arxiv.org/abs/2305.20026

本实现借鉴这些原则，没有直接移植 ROS 控制器，也没有把文献实验结果当成本车性能。
