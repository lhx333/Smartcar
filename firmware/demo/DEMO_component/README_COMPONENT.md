# 五模块调参固件

## 打开、编译和下载

Keil 打开 `E:\CARcar\demo1\DEMO_component\mdk\rt1064.uvprojx`，目标 `nor_sdram_zf_dtcm`，ArmClang 6.21。只打开这一个工程，无需分别打开五份工程。

1. 修改 `E:\CARcar\path_planner\component_tests.json` 的 `module` 为1~5，再改对应试验参数。
2. 共用参数修改 `E:\CARcar\path_planner\vehicle_config.json`：wheelbase=0.20m、body_radius=0.269m、speed_kp/ki/kd、pwm_start_percent、pwm_max_percent、servo_max_deg等。
3. Build/Rebuild 自动调用本目录 prepare_firmware.bat，再调用 path_planner/prepare_component.py。配置非法或路径失败会阻止构建。检查日志 `COMPONENT READY: module N` 与0 Error(s)。
4. 本工程的独立输出名是 `mdk/Objects/rt1064_component.axf`、`.hex`、`.bin`。旧复制来的 rt1064_new_path1 文件不用于本工程下载。
5. Debug/Utilities 选择实际探针及RT1064 Flash算法。配置并不由本工程自动识别，需按实际硬件设置。烧录新生成的component固件。
6. 上电连接E35上位机，保持静止，CALIBRATE完成后等待ARMED，再START。STOP随时中止。DEBUG/JOY不用于此试验固件。重新试验先STOP，待车停稳后START。

启动门禁已用实际状态机C代码对五个模块验证：上电INIT、标定中、标定后ARMED均无电机PWM；提前START会被拒绝且不会排队，须ARMED后重新START。READY与START同批到达也不会启动；STOP与START同批到达时STOP优先。静态模块3在START后仍保持电机零输出，仅开始舵机标定。
本固件禁止编译启用CHASSIS_BRINGUP_MODE或IMU_USE_PRECALIBRATED_DEFAULTS。回归结果见verification/startup_gate_result.txt。传感器算法和硬件输出边界使用桌面模拟，尚未做板上下载验证。
配套上位机为E:\CARcar\step31\Smartcar-main\E35_machine；其发车按钮只在ARMED启用，发送函数也会再次检查ARMED。更新上位机后应重启Python程序。

JSON在电脑上生成C头文件，修改后仍须编译、下载，运行中的MCU不直接读取JSON。JSON保存后不需要手动复制头文件。
也可以先在终端运行 `python E:\CARcar\path_planner\prepare_component.py` 检查和生成，但这不能替代Keil编译。
模块1~4不求解路径，因此不会因全赛道规划失败而无法编译；模块5才生成两桶路径。

## 共用配置及遥测

vehicle_config.json的PID只用于速度闭环模块2、4、5；模块1直接给PWM、模块3电机恒为零。
模块1默认25%在试验JSON中写25.0，不能写0.25。模块1有独立的10–40%开环PWM范围；其他闭环模块仍受共用pwm_max_percent=25%限制。
转向值为相对舵机中位99°的指令偏角，正值对应当前接线下左转。实际前轮角必须用量角/视频测量。
steering.rate_deg_s是舵机指令角的变化率；模块1~5最终输出共用此限速。模块5速度规划同时按wheel_angle_deg/servo_max_deg比例保守限制前轮转速假设。
速度闭环使用既有每轮PID、起步辅助与PWM限幅；两轮共用中心速度目标，没有轮距对应的内外轮速度分配。
左右PWM差限幅等既有参数仍在user/inc/my_common.h。编码器比例、电机方向、舵机中位等硬件标定也在此文件。
保留STOP、IMU失效、任务超时、调度超时保护；静态模块有意跳过电机堵转检测，其他模块保留低速/反向堵转检测。

E35协议保持兼容，CSV记录time_s/tick、vl/vr、duty_l/r、yaw、gz、turn、servo_offset、x/y、speed_target。
模块1~4的path字段为 `模块号×10+阶段号`；阶段9是PWM归零后的停车等待。模块5的path字段为轨迹点索引（现有8位遥测可能在255饱和），progress仍为弧长。
模块1和3没有速度目标，speed_target=0不表示轮速为零；DONE后仍回传滑行/静止遥测。模块1、2、4在两轮都低于0.03m/s持续settle_hold_s后DONE；超出settle_timeout_s仍不停则ERROR。
模块5到末端附近先将PWM归零，等待同一停车条件；这不是精确地面位置停车控制，需分析最终位置误差。
每次编译保存component_manifest.json，包括模块、完整输入配置与生成头文件哈希。保存日志时一起保存该manifest，区分不同烧录版本。
日志不含真实前轮角或地面独立位置；角速度来自IMU，纵向加速度需由轮速求导或外部测量。

## 模块1：开环PWM

文件code/test_pwm.h。默认阶段：11=25%保持5秒；5秒到时PWM直接归零；19=等待两轮停稳。归零是指令阶跃，不是制动加速度指令。
上位机只有一个“发车 START”按钮，旁边的“发车模式”默认选择“模块1：按PWM/时间”。在ARMED后选择整数PWM 10–40%和保持2–5秒，点击后先发送`CONFIG,25,5`（例）。固件保持ARMED，保存参数并回传`CONFIG,OK,25,5`；上位机逐项核对后才发送`START`，固件此时进入RUN。确认超时或数值不符不会发车。运行模块2–5时需先把发车模式改为“其他模块：普通START”。
模块1必须先收到CONFIG参数才能接受START。JSON中的pwm.high_percent和pwm.hold_high_s保留为任务生成的默认配置；当前上位机发车时用界面选值。纠偏增益改JSON后需重新Build并下载固件。
模块1只有电机PWM开环；转向是部分闭环。发车时IMU航向归零，运行中舵机根据航向误差、陀螺角速度及左右编码器速度差纠偏。电机两侧仍等PWM，速度不闭环；编码器差只作为转向阻尼，不等于独立位置测量。代码在code/straight_control.h；增益及最大舵机纠偏角分别在JSON的pwm.heading_kp_per_deg、yaw_rate_kd_per_dps、wheel_diff_k_per_mps、max_steer_deg。舵机输出还受共用steering.rate_deg_s和实际行程限幅。修改增益需要重新编译烧录。舵机符号/前轮实际角度需先用低速试车确认。此控制器维持初始航向，不能感知或纠正整车相对地面直线的横向平移。
先维持默认进行重复试验，对比起步延迟、轮速、轮速导数、左右差。若5秒末轮速仍在明显上升，就不能把该末段当成匀速样本，应继续加长hold_high_s。不同PWM起点还需分别测试，不能把单次试验当成完整PWM－速度关系。

## 模块2：闭环梯形速度

文件code/test_speed.h。默认0→0.5m/s（0.2m/s²）→保持3秒→以0.3m/s²目标减速到零→等待停车。
阶段21/22/23分别为加速/恒速/减速。speed中的target_mps、hold_s、accel_mps2、decel_mps2分别控制这些段。
先较低目标速度测试P，观察响应、超调与PWM饱和，再调整I消除稳态误差。D默认0，确认测速噪声后再尝试小值。PID改共用vehicle_config.json。
速度斜坡是目标，不保证真实车辆实现相同加速度。记录目标/实测速度、输出PWM和起步辅助影响。先定PID，再逐步提高加速度目标。

## 模块3：静态开环转向

文件code/test_static_steering.h。电机恒零，默认偏角序列0,+5,+10,+15,0,-5,-10,-15,0度，按10度/秒变化，到位后保持2秒。
path=31起，每个目标点阶段加1。调整steering.offsets_deg、hold_s、rate_deg_s；最后一个偏角必须是0，所有偏角受servo_max_deg限制。
在每个保持点人工测左右前轮角；分别记录从左/右回中误差。用视频记录指令扫角时的真实前轮响应，才能标定指令速率和实际速率。
先标SERVO_MOTOR_M，再标角度映射，最后标动态速率。软件不会把遥测servo_offset当作真实前轮角。

## 模块4：闭环速度、按 IMU 航向角结束转弯

在上位机选“模块4：按航向角/速度”，设目标航向变化角 ±5°–±90°、舵机指令偏角幅值2–15°、转弯速度和回正直线速度各0.2–1.0m/s（步长0.1）。上位机发送例如`C4,+045,08,04,06`，固件回传`C4,OK,+045,08,04,06`且仍处ARMED；核对一致后上位机才发`START`。
文件code/test_circle.h。先加速至转弯速度并直行circle.straight_s，再以界面指定的舵机偏角转向；IMU航向达到目标角后舵机回中，按第二速度直行circle.hold2_s，最后减速停车。circle.hold1_s现是到达目标角的最长等待时间（默认15s），超时进入ERROR。阶段41~46为起步、直线、转弯、回中变速、直线、减速；49为停车等待。
速度由编码器闭环；转弯结束角由IMU航向闭环判定。舵机实际前轮角仍无传感器反馈。目标航向角与舵机指令角是不同量，物理转弯半径需另测。改变circle.turn_deg、加减速率或阶段时长需改JSON并重新编译；上位机的三项参数每次发车可改。

## 模块5：两桶前进S路径

调用现有path_planner.py的Dubins候选搜索与robust_planner.py的平滑、速度约束及Pure Pursuit跟踪。默认桶坐标(3,0)、(6,0)m，首桶左绕、第二桶右绕，出弯后接1m切向直线停车。没有掉头与回程。
prepare_component.py先取得现有有限候选规划结果的去程，再接切向直线，重新检查净空、半径、边界（如提供）、漂移预算和速度表。不是全局最优解证明。
生成器要求存在曲率绝对值>0.05/m且规划加速度>0.01m/s²的样点，保证速度表包含弯道加速；实际加速度需上车验证。
调整path.cones_m、vmax_mps、radius_m、exit_straight_m；前视、摩擦、accel/brake、alat、jerk、转向能力等改共用vehicle_config.json。
先低速验证坐标与方向，再调前视距离，之后逐项增加vmax/alat/转向速率，比较横向误差、角速度、实际曲率和饱和情况。不要同时更改多类参数。
generated_path/component_path.png和component_trajectory.csv是最终执行路径/速度；component_summary.json给出长度、时间和弯道加速点数。
generated_path/path.svg、path.json是中间完整往返候选，仅用于搜索诊断，不能据它布置最终试验。
摆桶坐标以发车后轴中心为原点，X前、Y左，单位米；轴距已共用0.20m，车体包络半径0.269m。

## 代码组织与范围

五种模式所有代码同时参与编译，由生成的C_MODULE选择执行分支；每次改module后重新编译和下载。
code/component.c负责计时/停车等待，code/test_*.h为独立任务模块；user/src/state_machine.c负责状态和电机输出，user/src/navigation.c只在模块5调用跟踪器。
本目录基于DEMO_new_path1建立，底层引脚/IMU/编码器接口继承原工程，实际硬件标定需保持一致。
新工程与DEMO_new_path1的配置生成互不覆盖；前者由prepare_component.py生成，后者仍用prepare_firmware.py。
