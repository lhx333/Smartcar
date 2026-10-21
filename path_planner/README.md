# 智能车绕桶路径规划器

从 [barrels_input.csv](barrels_input.csv) 读取按行进顺序排列的桶坐标，规划出：

1. 从起点出发，左右交替绕过每一个桶；
2. 绕过最后一个桶后，向最后一桶通过侧的**相反侧**做 180° 掉头：
   - 最后一桶从左绕 → 从右侧掉头，回程在右侧；
   - 最后一桶从右绕 → 从左侧掉头，回程在左侧；
3. 沿桶列外侧的回程车道，回到发车坐标系 `x = 0` 的位置。

代码里**没有写死任何桶坐标**。每次运行都会重新读取 `barrels_input.csv`，
你只需要改这个 CSV。

---

## 文件夹说明

| 文件 | 用途 |
|---|---|
| `barrels_input.csv` | **唯一输入**。每次把本次桶坐标写到这里 |
| `生成我的路径.bat` | 双击运行，读取上面的 CSV，生成 `out_my` |
| `path_planner.py` | 路径规划核心代码 |
| `README.md` | 本说明 |
| `out_my/` | 运行后自动生成的结果目录 |

`out_my/` 每次运行会覆盖。如果想保留某一次结果，先把它改名或复制走。

---

## 1. 输入桶坐标

用记事本或 Excel 打开 [barrels_input.csv](barrels_input.csv)。

第一行保留：

```csv
x,y
```

下面每行一个桶，按小车通过的先后顺序：

```csv
x,y
<x1>,<y1>
<x2>,<y2>
<x3>,<y3>
```

规则：

- 单位是 **m**；
- `x` 向前，`y` 向左；
- 起点是 `(0, 0)`，发车朝向 `+x`；
- 一行一个桶，顺序不要打乱；
- 空行会被跳过；
- 用 Excel 修改后要“另存为 → CSV UTF-8”，不要保存成 `.xlsx`。

默认第 1 个桶从左侧绕、第 2 个右侧、第 3 个左侧，依次交替。
如果某一轮想固定从同一侧绕，可以在命令行加 `--sides`，例如：

```powershell
python -B path_planner.py --input barrels_input.csv --out out_my --sides "L,L,L"
```

---

## 2. 生成路径

### 一键运行

双击：

[生成我的路径.bat](生成我的路径.bat)

它会：

1. 读取 `barrels_input.csv`；
2. 计算路径；
3. 输出到 `out_my/`；
4. 自动打开 `out_my/path.svg`。

窗口会停在结果处，不会一闪而过。

### 命令行运行

在 `path_planner` 目录下打开 PowerShell：

```powershell
python -B path_planner.py --input barrels_input.csv --out out_my
```

如果只想临时给点、不写 CSV，也可以：

```powershell
python -B path_planner.py --points "x1,y1 x2,y2 x3,y3" --out out_my
```

### 双击 bat 仍然闪退

在 `path_planner` 文件夹的地址栏输入 `cmd` 回车，然后运行：

```bat
生成我的路径.bat
```

窗口不会自动关闭，可以把最后几行报错发出来。

---

## 3. 常用参数

| 参数 | 默认 | 含义 |
|---|---:|---|
| `--clearance` | `0.45` | 路径中心到桶中心的最小安全距离，m |
| `--max-offset` | `1.20` | 通过点离桶中心的最大横向距离，m |
| `--corridor` | `1.60` | 绕桶段离“起点—桶列”折线的最大距离，m；`0` 关闭 |
| `--radius` | `1.00` | 目标最小转弯半径，m |
| `--min-radius` | `0.55` | 自动降半径的下限，m |
| `--uturn-radius` | `1.20` | 最后 180° 掉头半径，m |
| `--exit` | `0.80` | 最后一桶到掉头起点的前向距离，m |
| `--first-side` | `left` | 第一个桶从左侧还是右侧绕 |
| `--sides` | 自动 | 逐个指定通过侧，如 `"L,R,L,R"` |
| `--heading-step` | `10` | 通过点切线角搜索步长，度 |
| `--heading-span` | `50` | 通过点切线角搜索半宽，度 |
| `--max-array-points` | `240` | 输出的 C 数组最多点数 |

想让转弯更缓，可以试：

```powershell
python -B path_planner.py --input barrels_input.csv --out out_my `
  --radius 1.2 --uturn-radius 1.4
```

桶距很密时，目标半径可能几何上不可行，程序会自动降低半径并打印警告。
这时应加大桶距或降低车速。

---

## 4. 输出文件

运行后 `out_my/` 里会有：

### `path_array.txt`

可直接替换固件 `navigation.c` 里的 `PATH_PTS` 数组：

```c
static const float PATH_PTS[][2] =
{
    {  0.0000f,  0.0000f },  /*   0 */
    ...
};
```

`PATH_N` 由 `sizeof` 自动计算，不用改。

### `path.svg`

俯视图：

- 红色桶和安全圆；
- 紫色菱形：规划通过点；
- 蓝色：绕桶段；
- 橙色：180° 掉头；
- 绿色：回程；
- 起点 S、终点 E；
- 本次的长度、最小半径、最小桶距等统计。

### `path.csv`

密集路径点，包含：

```text
index,x_m,y_m,heading_deg,s_m,curvature_1pm,segment
```

`segment` 为 `outbound`、`uturn` 或 `return`。

### `path.json`

本次输入的桶坐标、通过点、完整路径和统计，便于回放或二次处理。

---

## 5. 算法简述

- 每个桶用相邻桶坐标估计局部行进方向；
- 在桶的左侧或右侧取通过点，横向距离在 `clearance` 和 `max_offset` 之间搜索；
- 通过点切线角在局部方向 ±`heading_span` 内搜索；
- 相邻候选之间用 **Dubins 圆弧—直线—圆弧** 连接，圆弧半径不小于 `--radius`；
- 用动态规划选总长度最短的组合；
- 每一步都检查到所有桶的安全距离，并限制绕桶段离桶列的横向偏离；
- 最后从最后一桶向前 `--exit` 米，再按“相反侧”规则做 180° 掉头并返回。

---

## 6. 注意

- `clearance` 是路径中心到桶中心的距离，应包含桶半径、车身半宽和控制误差；
- 上车先低速，路径几何可行不等于实车能完全跟上；
- 回程终点是 `(0, 回程车道 y)`，不是精确回到 `(0,0)`；
- 路径切线连续，但圆弧和直线交接处曲率会跳变，实车舵机斜率限制会平滑它；
- 桶距过密时可能无解，这是几何限制，不是调参能完全解决的。
