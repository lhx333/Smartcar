#ifndef _navigation_h_
#define _navigation_h_

/* ==================================================================================
 *  navigation.h —— 二维航迹推算 + Pure Pursuit 前视跟踪（纯状态，不碰硬件）
 *
 *  坐标系：车体系 / 导航系都是 X 前、Y 左，发车瞬间重合；yaw 逆时针为正，车头指向导航 +X 时为 0。
 *  单位：本层是**弧度**；控制层（state_machine.c）的 yaw_deg / yaw_err 是**度**，换算只在两处交汇。
 *  ★ 航向解算不在本层 —— 在 imu.c，全工程只有它碰陀螺，本层拿到的 yaw / yaw_mid 已经解算好。
 * ================================================================================== */

#include "my_common.h"

/* ==================== 状态 ====================
 * 位置状态集中在这个结构体里；航向在 imu.c 的 yaw / yaw_cont 里，main.c 直接用于串口遥测。
 * 出问题先看 (x, y) 和 imu.c 的 yaw_cont。 */
typedef struct
{
    /* ---------- 位姿：航迹推算的输出 ---------- */
    float x, y;             /* m。导航系：X 前 / Y 左，发车点为原点 */
    float speed;            /* m/s，实测两轮平均速度；速度环分别使用各轮测速。 */
    float progress_m;       /* m，路径投影的单调前进弧长。 */
    float remaining_m;      /* m，剩余路径弧长，用于限制终点判定。 */
    float distance;         /* m，累计 |ds|（总路程，只增不减）。★ 只用于显示，
                             * 停车判据用的是 dist_to_goal，不是它。 */

    /* ---------- Pure Pursuit ---------- */
    int   path_index;       /* 当前投影路径段的起点下标。用于局部搜索和调试 ——
                             * 它应该单调地一路涨到 path_count-2，卡住不动说明位置推算偏了。 */
    int   path_count;       /* 路径点总数（= navigation.c 里那个数组的长度） */
    float dist_to_goal;     /* m，到**路径终点**（最后一个点）的距离。
                             * ★ 终点判据同时检查剩余路径弧长与终点线。 */
    float look_x, look_y;   /* 前视点的导航系坐标。出问题时也可检查这个值：
                             *   「车往哪打」完全由它决定，车跑歪了先确认它没跑歪。 */
    float bearing;          /* rad，从车位置看前视点的方位角（就是目标航向）。
                             * 控制层读它当目标航向，所以它必须每拍都跟着位置更新。 */
} Navigation_t;

extern Navigation_t g_nav;

/* ==================== 对外接口 ==================== */

/* 状态清零（路径是 navigation.c 里的 static const 手写数组，改路径就是改那个数组）。 */
void Navigation_Init(void);

/* 发车那一拍调用：把 (x, y, distance) 归零，最近点索引回到 0，
 * 并转调 imu.c 的 imu_yaw_reset() 把 yaw / yaw_cont 一起清零 —— 归零之后导航系就由
 * 车头朝向定义，路径写在「相对发车朝向」的坐标里，所以没有 yaw0 基准角这个量。 */
void Navigation_Reset(void);

/* 每个控制周期调用一次，★ 必须在 imu_update() 之后（它要用本拍的 yaw_mid）。
 *   ds_m : 本拍走过的距离（m），左右两轮平均值，控制层按 METERS_PER_COUNT 算好传进来
 *   dt   : 控制周期（s） */
void NavigationUpdate(float ds_m, float dt);
uint8 Navigation_Arrived(void);
float Navigation_SpeedTarget(void);
float Navigation_TurnCommand(void);
uint8 Navigation_TrackingValid(void);

#endif
