# 智能车鲁棒路径与速度规划

运行：

```powershell
python -B path_planner.py --input barrels_input.csv --out out_robust
```

或双击 `生成我的路径.bat`。修改桶坐标后必须重新生成。`out_my` 保留原下载结果供比较，新的默认批处理输出是 `out_robust`。

先阅读 [ROBUSTNESS.md](ROBUSTNESS.md)。车辆尺寸、轮胎抓地力、制动能力、车轮转角和残余零偏均尚未实测；当前数值是可复现的调试假设。

| 文件 | 用途 |
|---|---|
| path_planner.py | 原 Dubins 搜索几何内核；命令行默认调用鲁棒规划 |
| robust_planner.py | 平滑、硬约束复查、多半径候选比较、速度/前视/漂移预算 |
| vehicle_config.json | SI 单位的车体、动态、定位误差假设和场地边界 |
| out_robust/path.svg | 新路径俯视图 |
| out_robust/trajectory.csv | 弧长、位置、曲率、速度、时间、前视、加速度 |
| out_robust/robust_trajectory.h | 固件所需的完整轨迹表，含速度与前视 |
| out_robust/candidate_report.json | 成功及被拒绝的候选 |
| firmware_patch/ | 与 step31 基线配套的导航/控制补丁；尚未修改实际固件 |
| apply_firmware_patch.py | 校验原文件 SHA256、备份后应用补丁 |
| test_robust_planner.py | 几何、动态、坏输入、零偏与跟踪测试 |
| simulate_tracking.py | 运行实际 C 跟踪器的简化车辆模型测试 |

默认请求半径 1.20 m；只搜索 1.20/1.40/1.60 m，不自动缩小半径。默认 `--min-radius 1.0` 只用于额外限制，实际硬下限取它与 `--radius` 的较大者。无解时明确报错；禁止用小半径自动兜底。

默认中心到桶中心的安全距离为 0.60 m，并增加 0.12 m 几何平滑预留。车体使用包络圆，`body_radius` 应不小于以定位参考点为中心包住整车的半径。场地边界在发车局部坐标系中填写 `bounds: [xmin,xmax,ymin,ymax]`；留空意味着未验证出界风险。

坐标单位米、X 前、Y 左。桶必须按通过顺序输入、局部 X 递增，侧别严格交替。重复、过密、非有限坐标、错误侧别会拒绝。当前不支持任意二维乱序桶阵。

只复制 `path_array.txt` **不会启用速度表与新控制器**。补丁用法、参数单位、测试命令及限制见 ROBUSTNESS.md。
