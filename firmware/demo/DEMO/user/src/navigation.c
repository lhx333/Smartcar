/* ==================================================================================
 *  navigation.c —— 二维航迹推算 + Pure Pursuit 前视跟踪
 *
 *  每拍两件事，顺序固定：用本拍位移推位置（中点航向取自 imu.c）→ 重算前视点与目标航向。
 *  ★ 航向不在这里积分 —— 那件事在 imu.c，本层一行陀螺代码都没有。
 * ================================================================================== */

#include "navigation.h"

#include "imu.h"                /* yaw / yaw_mid / imu_yaw_reset() */

#include <math.h>
#include <string.h>             /* memset */

/* ==================================================================================
 *  路径：直接写点的数组
 *
 *  一个点 = 导航系（X 前 / Y 左，发车点为原点，发车朝向为 +X）里的一组坐标，单位 m。
 *  这份是任务路径：直行 3m → 左转半径 1m 的 U 型弯 → 反向直行 3m，共 61 点、9.14m。
 *  相邻点间距 0.15m（弯里约 0.157m）。改路径就是改这个数组，三条规则：
 *    1. 点数必须 ≥ 2（编译期检查见下面的 typedef）
 *    2. 点要足够密，间距必须远小于 PP_LOOKAHEAD_M（0.40m），否则前视点一跳一跳走、方向跟着抖
 *    3. 使用局部向前路径段搜索；路径进度不回退，前视点沿路径弧长插值
 * ================================================================================== */
static const float PATH_PTS[][2] =
{
    /* ---- 直行段 1：(0,0) → (3,0) ---- */
    {  0.00f,  0.00f },     /* 0  ← 发车点，也是导航系原点 */
    {  0.15f,  0.00f },
    {  0.30f,  0.00f },
    {  0.45f,  0.00f },
    {  0.60f,  0.00f },
    {  0.75f,  0.00f },
    {  0.90f,  0.00f },
    {  1.05f,  0.00f },
    {  1.20f,  0.00f },
    {  1.35f,  0.00f },
    {  1.50f,  0.00f },
    {  1.65f,  0.00f },
    {  1.80f,  0.00f },
    {  1.95f,  0.00f },
    {  2.10f,  0.00f },
    {  2.25f,  0.00f },
    {  2.40f,  0.00f },
    {  2.55f,  0.00f },
    {  2.70f,  0.00f },
    {  2.85f,  0.00f },
    {  3.00f,  0.00f },     /* 20 ← 直行段终点，也是 U 型弯起点 */

    /* ---- U 型弯：圆心 (3,1)、半径 1m、左转 180° ---- */
    {  3.16f,  0.01f },     /* 21 */
    {  3.31f,  0.05f },     /* 22 */
    {  3.45f,  0.11f },     /* 23 */
    {  3.59f,  0.19f },     /* 24 */
    {  3.71f,  0.29f },     /* 25 */
    {  3.81f,  0.41f },     /* 26 */
    {  3.89f,  0.55f },     /* 27 */
    {  3.95f,  0.69f },     /* 28 */
    {  3.99f,  0.84f },     /* 29 */
    {  4.00f,  1.00f },     /* 30 ← 弯的最外侧一点 */
    {  3.99f,  1.16f },     /* 31 */
    {  3.95f,  1.31f },     /* 32 */
    {  3.89f,  1.45f },     /* 33 */
    {  3.81f,  1.59f },     /* 34 */
    {  3.71f,  1.71f },     /* 35 */
    {  3.59f,  1.81f },     /* 36 */
    {  3.45f,  1.89f },     /* 37 */
    {  3.31f,  1.95f },     /* 38 */
    {  3.16f,  1.99f },     /* 39 */
    {  3.00f,  2.00f },     /* 40 ← 出弯 */

    /* ---- 直行段 2：(3,2) → (0,2) ---- */
    {  2.85f,  2.00f },     /* 41 */
    {  2.70f,  2.00f },
    {  2.55f,  2.00f },
    {  2.40f,  2.00f },
    {  2.25f,  2.00f },
    {  2.10f,  2.00f },
    {  1.95f,  2.00f },
    {  1.80f,  2.00f },
    {  1.65f,  2.00f },
    {  1.50f,  2.00f },
    {  1.35f,  2.00f },
    {  1.20f,  2.00f },
    {  1.05f,  2.00f },
    {  0.90f,  2.00f },
    {  0.75f,  2.00f },
    {  0.60f,  2.00f },
    {  0.45f,  2.00f },
    {  0.30f,  2.00f },
    {  0.15f,  2.00f },
    {  0.00f,  2.00f },     /* 60 ← 路径终点，停车判据就是「离它 ≤ NAV_ARRIVE_R_M」 */
};

#define PATH_N      ((int)(sizeof(PATH_PTS) / sizeof(PATH_PTS[0])))

/* 编译期检查点数：sizeof 不能出现在预处理条件里，所以用 typedef。
 * ★ 算法要求至少 2 个点，只有 1 个点时「找最近点」和「找前视点」会退化成同一个点。 */
typedef char path_point_count_check[(PATH_N >= 2) ? 1 : -1];

Navigation_t g_nav;

/* Spatial tracking: project onto nearby forward segments, then interpolate
 * the target a fixed arc length ahead. No speed target or timed path schedule. */
static float s_path_s[PATH_N];

static void update_target(void)
{
    int i, best, last, look;
    float best_d2 = 1.0e30f, progress = g_nav.progress_m;
    float target_s, dx, dy, t;
    best = g_nav.path_index;
    if (best >= PATH_N - 1) best = PATH_N - 2;
    last = best + 6;
    if (last > PATH_N - 2) last = PATH_N - 2;
    /* A forward local search prevents selecting the return leg of the U. */
    for (i = best; i <= last; ++i)
    {
        float vx = PATH_PTS[i+1][0] - PATH_PTS[i][0];
        float vy = PATH_PTS[i+1][1] - PATH_PTS[i][1];
        float length2 = vx*vx + vy*vy;
        float px, py, d2, candidate;
        if (length2 < 1.0e-12f) continue;
        t = ((g_nav.x - PATH_PTS[i][0])*vx +
             (g_nav.y - PATH_PTS[i][1])*vy) / length2;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        candidate = s_path_s[i] + t*(s_path_s[i+1] - s_path_s[i]);
        if (candidate < g_nav.progress_m)
        {
            if (s_path_s[i+1] < g_nav.progress_m) continue;
            candidate = g_nav.progress_m;
            t = (candidate-s_path_s[i]) / (s_path_s[i+1]-s_path_s[i]);
        }
        px = PATH_PTS[i][0] + t*vx;
        py = PATH_PTS[i][1] + t*vy;
        d2 = (g_nav.x-px)*(g_nav.x-px) + (g_nav.y-py)*(g_nav.y-py);
        if (d2 < best_d2)
        {
            best_d2 = d2; best = i; progress = candidate;
        }
    }
    g_nav.path_index = best;
    g_nav.progress_m = progress;
    g_nav.remaining_m = s_path_s[PATH_N-1] - progress;
    target_s = progress + PP_LOOKAHEAD_M;
    if (target_s > s_path_s[PATH_N-1]) target_s = s_path_s[PATH_N-1];
    look = best;
    while (look < PATH_N-2 && s_path_s[look+1] < target_s) ++look;
    dx = PATH_PTS[look+1][0] - PATH_PTS[look][0];
    dy = PATH_PTS[look+1][1] - PATH_PTS[look][1];
    t = s_path_s[look+1] - s_path_s[look];
    t = (t > 1.0e-6f) ? (target_s-s_path_s[look])/t : 0.0f;
    g_nav.look_x = PATH_PTS[look][0] + t*dx;
    g_nav.look_y = PATH_PTS[look][1] + t*dy;
    dx = g_nav.look_x - g_nav.x;
    dy = g_nav.look_y - g_nav.y;
    g_nav.bearing = (dx*dx + dy*dy < 1.0e-6f) ? yaw : atan2f(dy, dx);
    dx = PATH_PTS[PATH_N-1][0] - g_nav.x;
    dy = PATH_PTS[PATH_N-1][1] - g_nav.y;
    g_nav.dist_to_goal = sqrtf(dx*dx + dy*dy);
}

uint8 Navigation_Arrived(void)
{
    float dx, dy, vx, vy, length;
    if (g_nav.remaining_m > PP_LOOKAHEAD_M) return 0;
    if (g_nav.dist_to_goal <= NAV_ARRIVE_R_M) return 1;
    /* Stop if a sample crosses the finish line within the lateral tolerance.
     * Avoid steering back toward a missed endpoint at fixed motor duty. */
    dx = g_nav.x - PATH_PTS[PATH_N-1][0];
    dy = g_nav.y - PATH_PTS[PATH_N-1][1];
    vx = PATH_PTS[PATH_N-1][0] - PATH_PTS[PATH_N-2][0];
    vy = PATH_PTS[PATH_N-1][1] - PATH_PTS[PATH_N-2][1];
    length = sqrtf(vx*vx + vy*vy);
    return length > 1.0e-6f && dx*vx + dy*vy >= 0.0f &&
           fabsf(dx*vy - dy*vx)/length <= NAV_ARRIVE_R_M;
}

/* ==================== 初始化 / 复位 ==================== */

/* 状态清零，并把路径点数告诉显示层。 */
void Navigation_Init(void)
{
    int i;
    memset(&g_nav, 0, sizeof(g_nav));
    s_path_s[0] = 0.0f;
    for (i = 1; i < PATH_N; ++i)
    {
        float dx = PATH_PTS[i][0] - PATH_PTS[i-1][0];
        float dy = PATH_PTS[i][1] - PATH_PTS[i-1][1];
        s_path_s[i] = s_path_s[i-1] + sqrtf(dx*dx + dy*dy);
    }

    g_nav.path_count = PATH_N;

    update_target();
}

/* 发车那一拍调用：位置 / 路程归零，最近点回到 0。
 * 航向状态不归本层管，转交给 imu.c 的 imu_yaw_reset()。 */
void Navigation_Reset(void)
{
    g_nav.x         = 0.0f;
    g_nav.y         = 0.0f;
    g_nav.speed     = 0.0f;
    g_nav.distance  = 0.0f;
    g_nav.path_index = 0;
    g_nav.progress_m = 0.0f;

    imu_yaw_reset();

    update_target();
}

/* ==================== 每拍递推 ==================== */

/* 只管位置。航向由 imu.c 在本函数之前一拍算好（yaw / yaw_mid），这里直接读。
 * ★ 本函数必须在 imu_update() **之后**调用，否则 yaw_mid 还是上一拍的。 */
void NavigationUpdate(float ds_m, float dt)
{
    float v_raw;

    if (dt <= 0.0f) return;             /* 防除零 */

    /* ---- 1. 位置递推（中点航向）----
     * ★ 本拍中点航向是 imu.c 在**更新 yaw 之前**存下来的（在那边叫 yaw_mid）。
     *   千万不要在这里现算成 «yaw - 0.5*d_yaw» —— yaw 已经回绕过（可能刚跨过 ±pi），
     *   减出来的 d_yaw 是错的。 */
    g_nav.x += ds_m * cosf(yaw_mid);
    g_nav.y += ds_m * sinf(yaw_mid);

    g_nav.distance += fabsf(ds_m);

    /* ---- 2. 测速 ----
     * ★ ds_m 是 float 累加出来的，千万不要先把每拍位移取整 ——
     *   那会把低速区整个量化掉（0.05m/s 时每拍只有约 3.7 个 count）。 */
    v_raw = ds_m / dt;
    g_nav.speed += SPEED_LPF_A * (v_raw - g_nav.speed);

    /* ---- 3. 重算前视点 ----
     * ★ 前视点每拍按当前位置重新找，不需要推进，也没有索引越界这回事。 */
    update_target();
}
