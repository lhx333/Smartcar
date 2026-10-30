#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能车绕桶路径规划器
====================

输入
----
一串按行进顺序排列的桶（障碍物）坐标。默认坐标系与固件 navigation.c 一致：

    x 向前，y 向左，单位 m；
    起点为 (0, 0)，发车朝向为 +x；
    角度逆时针为正，0 表示 +x。

规划结果
--------
1. 从起点出发，按左右交替的方式依次绕过每个桶；
2. 绕过最后一个桶后，向“最后一个桶通过侧的相反侧”做 180° 掉头；
3. 沿回程车道回到起点的前进位置（局部 x=0），终点朝向 -x。

路径由 Dubins 曲线段拼成：
    - 每一段都是“圆弧—直线—圆弧”的最短曲率约束路径；
    - 所有圆弧半径 >= 设定最小转弯半径；
    - 相邻段在连接点处位置、切线连续（G1），没有折角；
    - 用动态规划为每个桶选择通过点横向距离和切线角，使总长度尽量短，
      同时检查桶的安全距离。

只依赖 Python 标准库，不需要 numpy / scipy / matplotlib。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

PI = math.pi
TAU = 2.0 * PI
EPS = 1e-9
INF = float("inf")


# =============================================================================
# 基础几何
# =============================================================================


def wrap_pi(angle: float) -> float:
    """把角度折到 [-pi, pi)。"""
    return (angle + PI) % TAU - PI


def wrap_tau(angle: float) -> float:
    """把角度折到 [0, 2*pi)。"""
    return angle % TAU


def wrap_deg(angle: float) -> float:
    """把角度折到 [-180, 180)。"""
    return (angle + 180.0) % 360.0 - 180.0


def hypot(dx: float, dy: float) -> float:
    return math.hypot(dx, dy)


def left_normal(theta: float) -> Tuple[float, float]:
    """车头方向 theta 的左侧单位法向量。"""
    return (-math.sin(theta), math.cos(theta))


def right_normal(theta: float) -> Tuple[float, float]:
    return (math.sin(theta), -math.cos(theta))


def point_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    vx = bx - ax
    vy = by - ay
    l2 = vx * vx + vy * vy
    if l2 < EPS:
        return hypot(px - ax, py - ay)
    t = ((px - ax) * vx + (py - ay) * vy) / l2
    t = min(1.0, max(0.0, t))
    return hypot(px - (ax + t * vx), py - (ay + t * vy))


@dataclass
class Pose:
    x: float
    y: float
    theta: float

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.theta)

    def distance_to(self, other: "Pose") -> float:
        return hypot(self.x - other.x, self.y - other.y)


@dataclass
class PathPoint:
    x: float
    y: float
    heading: float       # rad, [-pi, pi)
    s: float             # 累计弧长, m
    curvature: float     # 1/m，左转为正
    segment: str         # outbound / uturn / return


# =============================================================================
# Dubins：圆弧-直线-圆弧（CSC）最短路径
# =============================================================================


@dataclass
class DubinsPath:
    q0: Pose
    q1: Pose
    radius: float
    word: str
    phi: float
    straight: float
    psi: float
    length: float
    turn_total: float
    points: List[Pose]
    curvatures: List[float]

    @property
    def chord(self) -> float:
        return self.q0.distance_to(self.q1)


def _circle_center(p: Pose, turn_dir: int, radius: float) -> Tuple[float, float]:
    """turn_dir=+1 左转，-1 右转。圆心在行进方向对应的一侧。"""
    nx, ny = left_normal(p.theta)
    return (p.x + turn_dir * radius * nx, p.y + turn_dir * radius * ny)


def _arc_points(
    center: Tuple[float, float],
    radius: float,
    turn_dir: int,
    heading_start: float,
    delta: float,
    step: float,
) -> List[Pose]:
    """采样一段圆弧。delta >= 0，实际转向为 turn_dir。"""
    n = max(1, int(math.ceil(max(delta, 1.0e-6) * radius / max(step, 1.0e-4))))
    pts: List[Pose] = []
    for k in range(n + 1):
        h = heading_start + turn_dir * delta * k / n
        nx, ny = left_normal(h)
        pts.append(
            Pose(
                center[0] - turn_dir * radius * nx,
                center[1] - turn_dir * radius * ny,
                wrap_pi(h),
            )
        )
    return pts


def _dedupe_poses(poses: Sequence[Pose], tol: float = 1.0e-7) -> List[Pose]:
    out: List[Pose] = []
    for p in poses:
        if not out or p.distance_to(out[-1]) > tol:
            out.append(p)
    return out


def _build_csc(
    q0: Pose,
    q1: Pose,
    radius: float,
    d0: int,
    d1: int,
    step: float,
    sample: bool = True,
) -> Optional[DubinsPath]:
    """
    构造指定转向组合的最短 CSC 路径。

    d0/d1: +1 左转，-1 右转。
    sample=False 时只算参数和长度，不采样点，用于快速比较四种组合。
    返回 None 表示这种组合无解（例如内公切线不存在）。LL/RR 总有解。
    """
    if radius <= 0.0:
        raise ValueError("radius 必须大于 0")

    c0 = _circle_center(q0, d0, radius)
    c1 = _circle_center(q1, d1, radius)
    vx = c1[0] - c0[0]
    vy = c1[1] - c0[1]
    dist = hypot(vx, vy)
    word = ("L" if d0 > 0 else "R") + ("L" if d1 > 0 else "R")

    # 两圆心重合：退化成一段圆弧（例如标准 180° 掉头）。
    if d0 == d1 and dist < 1.0e-8:
        if d0 > 0:
            phi = wrap_tau(q1.theta - q0.theta)
        else:
            phi = wrap_tau(q0.theta - q1.theta)
        total = radius * phi
        if not sample:
            return DubinsPath(
                q0=q0, q1=q1, radius=radius, word=word, phi=phi,
                straight=0.0, psi=0.0, length=total, turn_total=phi,
                points=[], curvatures=[],
            )
        pts = _arc_points(c0, radius, d0, q0.theta, phi, step)
        return DubinsPath(
            q0=q0,
            q1=q1,
            radius=radius,
            word=word,
            phi=phi,
            straight=0.0,
            psi=0.0,
            length=total,
            turn_total=phi,
            points=pts,
            curvatures=[d0 / radius] * len(pts),
        )

    if d0 == d1:
        t = math.atan2(vy, vx)
        straight = dist
    else:
        # 内公切线：需要两圆心距离 >= 2R。
        if dist < 2.0 * radius - 1.0e-7:
            return None
        straight = math.sqrt(max(0.0, dist * dist - 4.0 * radius * radius))
        base = math.atan2(vy, vx)
        if d0 > 0 and d1 < 0:
            t = base + math.atan2(2.0 * radius, straight)
        else:
            t = base - math.atan2(2.0 * radius, straight)

    if d0 > 0:
        phi = wrap_tau(t - q0.theta)
    else:
        phi = wrap_tau(q0.theta - t)

    if d1 > 0:
        psi = wrap_tau(q1.theta - t)
    else:
        psi = wrap_tau(t - q1.theta)

    total = radius * phi + straight + radius * psi
    if not sample:
        return DubinsPath(
            q0=q0, q1=q1, radius=radius, word=word, phi=phi,
            straight=straight, psi=psi, length=total,
            turn_total=phi + psi, points=[], curvatures=[],
        )

    # 采样第一段圆弧。
    arc0 = _arc_points(c0, radius, d0, q0.theta, phi, step)
    pts: List[Pose] = list(arc0)
    curs: List[float] = [d0 / radius] * len(arc0)

    # 直线段：方向是切线方向 t，长度 straight。
    if straight > 1.0e-9:
        p_start = arc0[-1]
        ux = math.cos(t)
        uy = math.sin(t)
        n = max(1, int(math.ceil(straight / max(step, 1.0e-4))))
        for k in range(1, n + 1):
            frac = k / n
            pts.append(
                Pose(
                    p_start.x + straight * frac * ux,
                    p_start.y + straight * frac * uy,
                    wrap_pi(t),
                )
            )
            curs.append(0.0)

    # 第二段圆弧。从 t 转到 q1.theta。
    if psi > 1.0e-9:
        arc1 = _arc_points(c1, radius, d1, t, psi, step)
        for k, p in enumerate(arc1):
            if k == 0 and straight > 1.0e-9:
                continue
            pts.append(p)
            curs.append(d1 / radius)
    elif straight <= 1.0e-9:
        # 零长度路径，仍保留一个点。
        if not pts:
            pts.append(q0.copy())
            curs.append(0.0)

    pts = _dedupe_poses(pts)
    curs = curs[: len(pts)]

    # 数值校验：终点必须回到 q1。
    end = pts[-1]
    if end.distance_to(q1) > 2.0e-4 or abs(wrap_pi(end.theta - q1.theta)) > 2.0e-4:
        return None

    return DubinsPath(
        q0=q0,
        q1=q1,
        radius=radius,
        word=word,
        phi=phi,
        straight=straight,
        psi=psi,
        length=total,
        turn_total=phi + psi,
        points=pts,
        curvatures=curs,
    )


def solve_dubins(q0: Pose, q1: Pose, radius: float, step: float = 0.04) -> Optional[DubinsPath]:
    """求两点位姿之间、曲率半径不小于 radius 的最短 CSC 路径。"""
    # 先只算长度并排序，再按长度从短到长尝试采样；通常第一条即成功。
    options = []
    for d0, d1 in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        p = _build_csc(q0, q1, radius, d0, d1, step, sample=False)
        if p is not None:
            options.append((p.length, d0, d1))
    options.sort(key=lambda item: item[0])
    for _, d0, d1 in options:
        p = _build_csc(q0, q1, radius, d0, d1, step, sample=True)
        if p is not None:
            return p
    return None


# =============================================================================
# 碰撞 / 安全距离
# =============================================================================


@dataclass
class ClearanceResult:
    min_distance: float
    nearest_index: int
    nearest_point: Optional[Pose]
    nearest_obstacle: Optional[Tuple[float, float]]


def path_clearance(points: Sequence[Pose], obstacles: Sequence[Tuple[float, float]]) -> ClearanceResult:
    best = ClearanceResult(INF, -1, None, None)
    if not points or not obstacles:
        return best
    for p in points:
        for i, (ox, oy) in enumerate(obstacles):
            d = hypot(p.x - ox, p.y - oy)
            if d < best.min_distance:
                best = ClearanceResult(d, i, p, (ox, oy))
    return best


def clearance_ok(
    points: Sequence[Pose],
    obstacles: Sequence[Tuple[float, float]],
    minimum: float,
) -> bool:
    if not obstacles:
        return True
    for p in points:
        for ox, oy in obstacles:
            if hypot(p.x - ox, p.y - oy) < minimum - 1.0e-9:
                return False
    return True


def corridor_ok(
    points: Sequence[Pose],
    polyline: Sequence[Tuple[float, float]],
    limit: float,
) -> bool:
    """限制绕桶段不要离“起点-桶列”折线太远。"""
    if limit <= 0.0 or len(polyline) < 2:
        return True
    # 粗采样即可，避免规划时间翻倍。
    stride = max(1, len(points) // 24)
    for p in points[::stride]:
        d = min(
            point_segment_distance(p.x, p.y, polyline[i][0], polyline[i][1],
                                   polyline[i + 1][0], polyline[i + 1][1])
            for i in range(len(polyline) - 1)
        )
        if d > limit:
            return False
    return True


# =============================================================================
# 规划参数和候选点
# =============================================================================


@dataclass
class PlannerConfig:
    clearance: float = 0.45
    max_offset: float = 1.20
    radius: float = 1.00
    min_radius: float = 0.55
    uturn_radius: float = 1.20
    first_side: int = 1              # +1 左，-1 右
    sides: Optional[List[int]] = None
    heading_step_deg: float = 10.0
    heading_span_deg: float = 50.0
    offset_levels: int = 3
    offset_step: float = 0.15
    offset_margin: float = 0.08      # 通过点比 clearance 再多留一点余量
    offset_penalty: float = 0.60     # 每多偏离 1m 的代价，用来让路径贴近桶
    corridor_limit: float = 1.60     # 绕桶段离“起点-桶列”折线的最大距离，0 表示关闭
    step: float = 0.04               # 路径采样间距
    exit_lengths: Tuple[float, ...] = (0.80, 1.00, 1.20, 1.60, 2.00)
    loop_turn_threshold: float = 1.75 * PI
    max_array_points: int = 240


@dataclass
class Candidate:
    index: int
    pose: Pose
    offset: float
    heading: float
    cost_offset: float
    side: int
    obstacle_index: int


@dataclass
class Gate:
    index: int
    obstacle: Tuple[float, float]
    side: int
    tangent: float
    candidates: List[Candidate]


@dataclass
class Segment:
    name: str
    points: List[Pose]
    curvatures: List[float]


@dataclass
class Solution:
    config: PlannerConfig
    start: Pose
    obstacles: List[Tuple[float, float]]
    sides: List[int]
    gates: List[Gate]
    chosen: List[Candidate]
    segments: List[Segment]
    points: List[PathPoint]
    summary: Dict[str, object]
    warnings: List[str] = field(default_factory=list)

    @property
    def end(self) -> PathPoint:
        return self.points[-1]


# =============================================================================
# 几何小工具
# =============================================================================


def circular_smooth_angles(angles: Sequence[float], weights: Sequence[float]) -> List[float]:
    """对角度序列做环形平滑，避免 ±pi 跳变。"""
    if not angles:
        return []
    n = len(angles)
    out: List[float] = []
    wsum = sum(weights)
    for i in range(n):
        sx = 0.0
        sy = 0.0
        for k, w in enumerate(weights):
            j = i + k - (len(weights) // 2)
            a = angles[j % n]
            sx += w * math.cos(a)
            sy += w * math.sin(a)
        out.append(math.atan2(sy, sx))
    return out


def obstacle_tangents(points: Sequence[Tuple[float, float]]) -> List[float]:
    """每个桶处的局部行进方向，用相邻桶做中心差分并做轻度平滑。"""
    n = len(points)
    if n == 0:
        return []
    raw: List[float] = []
    for i in range(n):
        if n == 1:
            raw.append(0.0)
        elif i == 0:
            dx = points[1][0] - points[0][0]
            dy = points[1][1] - points[0][1]
            raw.append(math.atan2(dy, dx))
        elif i == n - 1:
            dx = points[-1][0] - points[-2][0]
            dy = points[-1][1] - points[-2][1]
            raw.append(math.atan2(dy, dx))
        else:
            dx = points[i + 1][0] - points[i - 1][0]
            dy = points[i + 1][1] - points[i - 1][1]
            raw.append(math.atan2(dy, dx))
    if n >= 3:
        raw = circular_smooth_angles(raw, (0.25, 0.50, 0.25))
    return raw


def transform_to_start_frame(
    points: Sequence[Tuple[float, float]], start: Pose
) -> List[Tuple[float, float]]:
    """把全局点变换到发车坐标系：起点为原点，车头为 +x。"""
    c = math.cos(-start.theta)
    s = math.sin(-start.theta)
    out: List[Tuple[float, float]] = []
    for x, y in points:
        dx = x - start.x
        dy = y - start.y
        out.append((c * dx - s * dy, s * dx + c * dy))
    return out


def build_candidates(
    obstacles: Sequence[Tuple[float, float]],
    tangents: Sequence[float],
    sides: Sequence[int],
    config: PlannerConfig,
) -> List[Gate]:
    base_offset = config.clearance + config.offset_margin
    if base_offset > config.max_offset + 1.0e-9:
        raise ValueError(
            f"clearance+margin={base_offset:.3f}m 超过 max_offset={config.max_offset:.3f}m"
        )

    offsets = [base_offset + i * config.offset_step for i in range(config.offset_levels)]
    offsets = [min(o, config.max_offset) for o in offsets]
    # 去重并保留顺序
    offsets = sorted(set(round(o, 6) for o in offsets))

    gates: List[Gate] = []
    for i, (ox, oy) in enumerate(obstacles):
        tangent = tangents[i]
        side = sides[i]
        nx, ny = left_normal(tangent)
        cands: List[Candidate] = []
        for offset in offsets:
            gx = ox + side * offset * nx
            gy = oy + side * offset * ny
            # 通过点本身不能压到别的桶。
            bad = False
            for j, (qx, qy) in enumerate(obstacles):
                if j == i:
                    continue
                if hypot(gx - qx, gy - qy) < config.clearance + 0.02:
                    bad = True
                    break
            if bad:
                continue

            span = math.radians(config.heading_span_deg)
            step = math.radians(config.heading_step_deg)
            count = int(round(span / step))
            for k in range(-count, count + 1):
                theta = wrap_pi(tangent + k * step)
                cands.append(
                    Candidate(
                        index=len(cands),
                        pose=Pose(gx, gy, theta),
                        offset=offset,
                        heading=theta,
                        cost_offset=config.offset_penalty * (offset - base_offset),
                        side=side,
                        obstacle_index=i,
                    )
                )
        if not cands:
            raise ValueError(f"第 {i + 1} 个桶没有可用的通过点，请检查桶间距或 clearance")
        gates.append(
            Gate(
                index=i,
                obstacle=(ox, oy),
                side=side,
                tangent=tangent,
                candidates=cands,
            )
        )
    return gates


# =============================================================================
# 单一半径下的动态规划
# =============================================================================


def _edge_loop_penalty(
    edge: DubinsPath,
    config: PlannerConfig,
) -> float:
    """惩罚绕整圈的路径；正常 S 弯/180° 掉头不会被罚。"""
    penalty = 0.0
    if edge.turn_total > config.loop_turn_threshold:
        penalty += 8.0 * (edge.turn_total - config.loop_turn_threshold)
    excess = edge.length - (edge.chord + TAU * edge.radius + 2.5)
    if excess > 0.0:
        penalty += 4.0 * excess
    return penalty


def _make_uturn_arc(
    pre: Pose,
    radius: float,
    turn_dir: int,
    step: float,
) -> List[Pose]:
    """从朝向 +x 的 pre 点开始，向 turn_dir 转 180°。"""
    cx = pre.x
    cy = pre.y + turn_dir * radius
    theta0 = -turn_dir * PI / 2.0
    n = max(12, int(math.ceil(PI * radius / max(step, 1.0e-4))))
    pts: List[Pose] = []
    for k in range(n + 1):
        t = PI * k / n
        ang = theta0 + turn_dir * t
        pts.append(
            Pose(
                cx + radius * math.cos(ang),
                cy + radius * math.sin(ang),
                wrap_pi(turn_dir * t),
            )
        )
    return pts


def _sample_straight(q0: Pose, q1: Pose, step: float) -> List[Pose]:
    length = q0.distance_to(q1)
    if length < 1.0e-9:
        return [q0.copy()]
    n = max(1, int(math.ceil(length / max(step, 1.0e-4))))
    theta = math.atan2(q1.y - q0.y, q1.x - q0.x)
    pts = [q0.copy()]
    for k in range(1, n + 1):
        f = k / n
        pts.append(
            Pose(
                q0.x + (q1.x - q0.x) * f,
                q0.y + (q1.y - q0.y) * f,
                wrap_pi(theta),
            )
        )
    return pts


def _stitch_segments(segments: Sequence[Segment], step: float) -> List[PathPoint]:
    raw: List[Tuple[float, float, float, str, float]] = []
    for seg in segments:
        pts = seg.points
        curs = seg.curvatures
        for k, p in enumerate(pts):
            c = curs[k] if k < len(curs) else 0.0
            if raw and hypot(p.x - raw[-1][0], p.y - raw[-1][1]) < 1.0e-7:
                # 保留后一段的曲率/标签，位置不重复。
                raw[-1] = (p.x, p.y, p.theta, seg.name, c)
                continue
            raw.append((p.x, p.y, p.theta, seg.name, c))

    out: List[PathPoint] = []
    s = 0.0
    prev: Optional[Tuple[float, float]] = None
    for item in raw:
        x, y, theta, name, cur = item  # type: ignore[misc]
        if prev is not None:
            s += hypot(x - prev[0], y - prev[1])
        out.append(PathPoint(x, y, wrap_pi(theta), s, cur, name))
        prev = (x, y)
    return out


def _summarize(
    config: PlannerConfig,
    radius_used: float,
    points: Sequence[PathPoint],
    obstacles: Sequence[Tuple[float, float]],
    gates: Sequence[Gate],
    chosen: Sequence[Candidate],
    start: Pose,
    walls: Sequence[str],
) -> Dict[str, object]:
    total_length = points[-1].s if points else 0.0
    min_clear = INF
    nearest = None
    for p in points:
        for i, (ox, oy) in enumerate(obstacles):
            d = hypot(p.x - ox, p.y - oy)
            if d < min_clear:
                min_clear = d
                nearest = i
    end = points[-1] if points else PathPoint(0, 0, 0, 0, 0, "")
    max_offset = max((c.offset for c in chosen), default=0.0)

    seg_lengths: Dict[str, float] = {"outbound": 0.0, "uturn": 0.0, "return": 0.0}
    for i in range(1, len(points)):
        # 交接处的微小距离计入后一段，保证三段之和等于总长。
        seg_lengths[points[i].segment] += points[i].s - points[i - 1].s

    polyline = [(0.0, 0.0)] + list(obstacles)
    max_dev = 0.0
    for p in points:
        if p.segment != "outbound":
            continue
        d = min(
            point_segment_distance(p.x, p.y, polyline[i][0], polyline[i][1],
                                   polyline[i + 1][0], polyline[i + 1][1])
            for i in range(len(polyline) - 1)
        )
        max_dev = max(max_dev, d)

    return {
        "length_m": round(total_length, 4),
        "outbound_length_m": round(seg_lengths["outbound"], 4),
        "uturn_length_m": round(seg_lengths["uturn"], 4),
        "return_length_m": round(seg_lengths["return"], 4),
        "min_clearance_m": round(min_clear, 4) if min_clear < INF else None,
        "nearest_obstacle_index": nearest,
        "max_gate_offset_m": round(max_offset, 4),
        "requested_radius_m": round(config.radius, 4),
        "radius_used_m": round(radius_used, 4),
        "uturn_radius_m": round(max(config.uturn_radius, radius_used), 4),
        "clearance_target_m": round(config.clearance, 4),
        "max_corridor_deviation_m": round(max_dev, 4),
        "corridor_limit_m": round(config.corridor_limit, 4),
        "last_pass_side": "L" if chosen and chosen[-1].side > 0 else "R",
        "uturn_side": "R" if chosen and chosen[-1].side > 0 else "L",
        "end_x_m": round(end.x, 4),
        "end_y_m": round(end.y, 4),
        "end_heading_deg": round(math.degrees(end.heading), 3),
        "path_points": len(points),
        "warnings": list(walls),
    }


def plan_for_radius(
    obstacles: Sequence[Tuple[float, float]],
    tangents: Sequence[float],
    sides: Sequence[int],
    config: PlannerConfig,
    radius: float,
) -> Optional[Solution]:
    """在固定最小转弯半径下，用 DP 选每个桶的通过点和切线角。"""
    gates = build_candidates(obstacles, tangents, sides, config)
    n = len(gates)
    if n == 0:
        return None

    start = Pose(0.0, 0.0, 0.0)
    course_polyline = [(0.0, 0.0)] + list(obstacles)
    prev_poses: List[Pose] = [start]
    prev_costs: List[float] = [0.0]
    prev_candidates: List[Optional[Candidate]] = [None]
    parents: List[List[int]] = []

    for gi, gate in enumerate(gates):
        cands = gate.candidates
        curr_cost = [INF] * len(cands)
        curr_parent = [-1] * len(cands)
        for j, cand in enumerate(cands):
            best_cost = INF
            best_k = -1
            for k, pcost in enumerate(prev_costs):
                if not math.isfinite(pcost):
                    continue
                edge = solve_dubins(prev_poses[k], cand.pose, radius, config.step)
                if edge is None:
                    continue
                if not clearance_ok(edge.points, obstacles, config.clearance):
                    continue
                if not corridor_ok(edge.points, course_polyline, config.corridor_limit):
                    continue
                cost = (
                    pcost
                    + edge.length
                    + cand.cost_offset
                    + _edge_loop_penalty(edge, config)
                )
                if cost < best_cost:
                    best_cost = cost
                    best_k = k
            if best_k >= 0:
                curr_cost[j] = best_cost
                curr_parent[j] = best_k
        if not any(math.isfinite(c) for c in curr_cost):
            return None
        parents.append(curr_parent)
        prev_poses = [c.pose for c in cands]
        prev_costs = curr_cost
        prev_candidates = list(cands)

    # 最后一段：最后一个桶 -> 掉头起始点 -> 180° 弧 -> 回程直线。
    best_final: Optional[Dict[str, object]] = None
    best_final_any: Optional[Dict[str, object]] = None
    last_gate = gates[-1]
    uturn_r = max(config.uturn_radius, radius)

    for j, cand in enumerate(last_gate.candidates):
        base_cost = prev_costs[j]
        if not math.isfinite(base_cost):
            continue

        # 规则：最后一个桶从左侧绕，就从右侧掉头返回；从右侧绕，就从左侧掉头返回。
        # 即掉头方向必须与最后通过侧相反，并且先离开最后一个桶一段距离再掉头。
        turn_dirs = [-cand.side]
        for turn_dir in turn_dirs:
            for exit_len in config.exit_lengths:
                # 掉头起点必须确实越过最后一个桶：取通过点和桶 x 的较大者，
                # 再向前 exit_len，避免斜向桶列时通过点 x 反而落后于桶。
                pre_x = max(cand.pose.x, last_gate.obstacle[0]) + exit_len
                pre = Pose(pre_x, cand.pose.y, 0.0)
                if pre.x <= 0.8:
                    continue
                connect = solve_dubins(cand.pose, pre, radius, config.step)
                if connect is None:
                    continue
                if not clearance_ok(connect.points, obstacles, config.clearance):
                    continue

                arc = _make_uturn_arc(pre, uturn_r, turn_dir, config.step)
                if not clearance_ok(arc, obstacles, config.clearance):
                    continue

                lane_y = cand.pose.y + turn_dir * 2.0 * uturn_r
                ret = _sample_straight(
                    Pose(pre.x, lane_y, PI), Pose(0.0, lane_y, PI), config.step
                )
                if not clearance_ok(ret, obstacles, config.clearance):
                    continue
                if abs(lane_y) < 0.6:
                    continue
                # 回程车道优先放在整列桶的外侧，避免从桶列中间穿回去。
                outside = (
                    lane_y
                    >= max(y for _, y in obstacles) + config.clearance + 0.05
                    if turn_dir > 0
                    else lane_y
                    <= min(y for _, y in obstacles) - config.clearance - 0.05
                )

                total = base_cost + connect.length + PI * uturn_r + pre.x
                record = {
                    "total": total,
                    "last_index": j,
                    "turn_dir": turn_dir,
                    "exit_len": exit_len,
                    "pre": pre,
                    "connect": connect,
                    "arc": arc,
                    "return": ret,
                    "lane_y": lane_y,
                    "uturn_r": uturn_r,
                }
                if best_final_any is None or total < float(best_final_any["total"]):
                    best_final_any = record
                if outside and (best_final is None or total < float(best_final["total"])):
                    best_final = record

    # 极少数布局没有“整列外侧”的回程道，此时退回只保证安全距离的方案。
    outside_lane = best_final is not None
    if best_final is None:
        best_final = best_final_any
    if best_final is None:
        return None

    # 回溯每个桶选中的候选。
    last_idx = int(best_final["last_index"])
    chosen_rev: List[int] = []
    j = last_idx
    for gi in range(n - 1, -1, -1):
        chosen_rev.append(j)
        j = parents[gi][j]
        if j < 0 and gi > 0:
            # 理论上只有 gi=0 时 parent 才会是 -1（起点）。
            pass
    chosen_indices = list(reversed(chosen_rev))
    chosen: List[Candidate] = [gates[i].candidates[chosen_indices[i]] for i in range(n)]

    # 重建路径段。
    segments: List[Segment] = []
    q = start.copy()
    for gi, cand in enumerate(chosen):
        edge = solve_dubins(q, cand.pose, radius, config.step)
        if edge is None:
            return None
        segments.append(Segment("outbound", edge.points, edge.curvatures))
        q = cand.pose
    connect = best_final["connect"]
    assert isinstance(connect, DubinsPath)
    segments.append(Segment("outbound", connect.points, connect.curvatures))

    arc = best_final["arc"]
    assert isinstance(arc, list)
    segments.append(Segment("uturn", arc, [1.0 / uturn_r * int(best_final["turn_dir"])] * len(arc)))

    ret = best_final["return"]
    assert isinstance(ret, list)
    segments.append(Segment("return", ret, [0.0] * len(ret)))

    points = _stitch_segments(segments, config.step)

    # 环路检测：只检查绕桶段，不检查最后的掉头。
    has_loop = False
    max_turn = 0.0
    q = start.copy()
    for gi, cand in enumerate(chosen):
        edge = solve_dubins(q, cand.pose, radius, config.step)
        if edge is not None:
            max_turn = max(max_turn, edge.turn_total)
            if edge.turn_total > config.loop_turn_threshold:
                has_loop = True
            if edge.length > edge.chord + TAU * radius + 2.5:
                has_loop = True
        q = cand.pose
    max_turn = max(max_turn, connect.turn_total)
    if connect.turn_total > config.loop_turn_threshold:
        has_loop = True

    warnings: List[str] = []
    if radius < config.radius - 1.0e-6:
        warnings.append(
            f"目标最小转弯半径 {config.radius:.2f}m 在该桶距下无法顺畅通过，"
            f"已降到 {radius:.2f}m；建议加大桶距或降低车速。"
        )
    if has_loop:
        warnings.append(
            "当前桶距/安全距离下存在绕整圈的 Dubins 段，路径偏长；"
            "建议加大桶距、减小 clearance 或降低最小转弯半径。"
        )
    if not outside_lane:
        warnings.append(
            "找不到位于整列桶外侧的回程车道，回程可能从桶列附近穿回；"
            "请检查桶坐标或适当增大掉头半径。"
        )

    summary = _summarize(
        config, radius, points, obstacles, gates, chosen, start, warnings
    )
    summary["has_loop"] = has_loop
    summary["max_edge_turn_deg"] = round(math.degrees(max_turn), 2)

    return Solution(
        config=config,
        start=start,
        obstacles=list(obstacles),
        sides=list(sides),
        gates=gates,
        chosen=chosen,
        segments=segments,
        points=points,
        summary=summary,
        warnings=warnings,
    )


def plan_path(
    obstacles: Sequence[Tuple[float, float]],
    config: Optional[PlannerConfig] = None,
    start: Optional[Pose] = None,
) -> Solution:
    """规划完整路径。返回最优可行解，必要时自动降低半径。"""
    config = config or PlannerConfig()
    start = start or Pose(0.0, 0.0, 0.0)

    if not obstacles:
        raise ValueError("至少需要一个桶坐标")

    # 统一变换到发车坐标系。
    local_obs = transform_to_start_frame(obstacles, start)
    tangents = obstacle_tangents(local_obs)

    n = len(local_obs)
    if config.sides is not None:
        sides = list(config.sides)
        if len(sides) != n:
            raise ValueError(f"sides 数量 {len(sides)} 与桶数量 {n} 不一致")
    else:
        sides = [config.first_side if i % 2 == 0 else -config.first_side for i in range(n)]

    # 点序检查：投影到首尾方向应基本单调。
    if n >= 2:
        ex = local_obs[-1][0] - local_obs[0][0]
        ey = local_obs[-1][1] - local_obs[0][1]
        elen = hypot(ex, ey)
        if elen > 1.0e-6:
            ux, uy = ex / elen, ey / elen
            proj = [x * ux + y * uy for x, y in local_obs]
            if any(proj[i + 1] < proj[i] - 0.25 for i in range(n - 1)):
                # 仍继续规划，但把警告写进最终结果。
                pass

    radii: List[float] = []
    r = config.radius
    while r >= config.min_radius - 1.0e-9:
        radii.append(round(r, 4))
        r -= 0.10
    if not radii:
        radii = [config.radius]

    best_any: Optional[Solution] = None
    for radius in radii:
        sol = plan_for_radius(local_obs, tangents, sides, config, radius)
        if sol is None:
            continue
        if best_any is None:
            best_any = sol
        if not sol.summary.get("has_loop", False):
            # 保留第一个（半径最大的）无环路解。输出坐标仍是发车坐标系。
            sol.obstacles = list(local_obs)
            return sol

    if best_any is None:
        raise RuntimeError(
            f"找不到可行路径（已尝试半径 {config.radius:.2f}~{config.min_radius:.2f}m）。"
            "请增大 max_offset、减小 clearance/半径，或检查桶坐标是否过近。"
        )
    best_any.obstacles = list(local_obs)
    return best_any


# =============================================================================
# 输出
# =============================================================================


def _resample_by_spacing(points: Sequence[PathPoint], spacing: float) -> List[PathPoint]:
    """按弧长等距重采样，用于生成 C 数组。"""
    if len(points) <= 2:
        return list(points)
    wanted = [points[0]]
    next_s = spacing
    total = points[-1].s
    j = 0
    while next_s < total - 1.0e-9:
        while j + 1 < len(points) and points[j + 1].s < next_s:
            j += 1
        if j + 1 >= len(points):
            break
        p0 = points[j]
        p1 = points[j + 1]
        ds = p1.s - p0.s
        if ds < 1.0e-9:
            j += 1
            continue
        f = (next_s - p0.s) / ds
        wanted.append(
            PathPoint(
                p0.x + (p1.x - p0.x) * f,
                p0.y + (p1.y - p0.y) * f,
                wrap_pi(p0.heading + wrap_pi(p1.heading - p0.heading) * f),
                next_s,
                p0.curvature,
                p0.segment,
            )
        )
        next_s += spacing
    if wanted[-1].s < total - 1.0e-9:
        wanted.append(points[-1])
    else:
        wanted[-1] = points[-1]
    return wanted


def write_outputs(sol: Solution, out_dir: str) -> Dict[str, str]:
    os.makedirs(out_dir, exist_ok=True)
    files: Dict[str, str] = {}

    # CSV
    csv_path = os.path.join(out_dir, "path.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["index", "x_m", "y_m", "heading_deg", "s_m", "curvature_1pm", "segment"])
        for i, p in enumerate(sol.points):
            w.writerow(
                [
                    i,
                    f"{p.x:.6f}",
                    f"{p.y:.6f}",
                    f"{math.degrees(p.heading):.4f}",
                    f"{p.s:.6f}",
                    f"{p.curvature:.6f}",
                    p.segment,
                ]
            )
    files["csv"] = csv_path

    # JSON
    json_path = os.path.join(out_dir, "path.json")
    payload = {
        "coordinate_frame": {
            "x": "forward at start, m",
            "y": "left at start, m",
            "heading": "rad, CCW from +x",
            "origin": "start pose",
        },
        "settings": {
            "clearance_m": sol.config.clearance,
            "max_offset_m": sol.config.max_offset,
            "corridor_limit_m": sol.config.corridor_limit,
            "requested_radius_m": sol.config.radius,
            "min_radius_m": sol.config.min_radius,
            "uturn_radius_m": sol.config.uturn_radius,
            "first_side": "left" if sol.config.first_side > 0 else "right",
        },
        "summary": sol.summary,
        "obstacles": [{"index": i, "x": x, "y": y} for i, (x, y) in enumerate(sol.obstacles)],
        "sides": ["L" if s > 0 else "R" for s in sol.sides],
        "gates": [
            {
                "index": i,
                "x": round(c.pose.x, 6),
                "y": round(c.pose.y, 6),
                "heading_deg": round(math.degrees(c.pose.theta), 4),
                "offset_m": round(c.offset, 6),
            }
            for i, c in enumerate(sol.chosen)
        ],
        "path": [
            {
                "x": round(p.x, 6),
                "y": round(p.y, 6),
                "heading_deg": round(math.degrees(p.heading), 4),
                "s_m": round(p.s, 6),
                "curvature_1pm": round(p.curvature, 6),
                "segment": p.segment,
            }
            for p in sol.points
        ],
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    files["json"] = json_path

    # C array
    c_path = os.path.join(out_dir, "path_array.txt")
    total = sol.points[-1].s if sol.points else 0.0
    spacing = 0.10
    arr = _resample_by_spacing(sol.points, spacing)
    if len(arr) > sol.config.max_array_points:
        spacing = max(0.10, total / (sol.config.max_array_points - 1))
        arr = _resample_by_spacing(sol.points, spacing)
    lines = []
    lines.append("/* 由 path_planner.py 生成：x 前、y 左，单位 m。")
    lines.append(
        f" * 路径长度 {sol.summary['length_m']:.3f} m，"
        f"最小半径 {sol.summary['radius_used_m']:.3f} m，"
        f"最小桶距 {sol.summary['min_clearance_m']} m。"
    )
    lines.append(f" * 点数 {len(arr)}，点距约 {spacing:.3f} m。")
    lines.append(" * 替换 navigation.c 里的 PATH_PTS 数组即可。 */")
    lines.append("static const float PATH_PTS[][2] =")
    lines.append("{")
    for i, p in enumerate(arr):
        comma = "," if i < len(arr) - 1 else ""
        lines.append(f"    {{ {p.x: .4f}f, {p.y: .4f}f }}{comma}  /* {i:3d} */")
    lines.append("};")
    with open(c_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    files["c_array"] = c_path

    # SVG
    svg_path = os.path.join(out_dir, "path.svg")
    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(render_svg(sol))
    files["svg"] = svg_path

    return files


def render_svg(sol: Solution) -> str:
    """生成俯视图 SVG，纯标准库。"""
    pts = sol.points
    obs = sol.obstacles
    xs = [p.x for p in pts] + [x for x, _ in obs] + [sol.start.x, 0.0]
    ys = [p.y for p in pts] + [y for _, y in obs] + [sol.start.y, 0.0]
    pad_world = max(0.8, sol.config.clearance + 0.4, sol.config.uturn_radius * 0.5)
    xmin, xmax = min(xs) - pad_world, max(xs) + pad_world
    ymin, ymax = min(ys) - pad_world, max(ys) + pad_world
    spanx = max(1.0e-6, xmax - xmin)
    spany = max(1.0e-6, ymax - ymin)

    W, H = 1280, 860
    margin_l, margin_r, margin_t, margin_b = 55, 300, 55, 55
    scale = min((W - margin_l - margin_r) / spanx, (H - margin_t - margin_b) / spany)
    # 居中
    ox = margin_l + ((W - margin_l - margin_r) - spanx * scale) / 2.0
    oy = margin_t + ((H - margin_t - margin_b) - spany * scale) / 2.0

    def sx(x: float) -> float:
        return ox + (x - xmin) * scale

    def sy(y: float) -> float:
        return oy + (ymax - y) * scale

    def path_d(points: Sequence[PathPoint]) -> str:
        if not points:
            return ""
        return "M " + " L ".join(f"{sx(p.x):.2f},{sy(p.y):.2f}" for p in points)

    # 网格
    grid = 1.0
    if spanx > 30 or spany > 30:
        grid = 5.0
    elif spanx > 12 or spany > 12:
        grid = 2.0
    gx0 = math.floor(xmin / grid) * grid
    gy0 = math.floor(ymin / grid) * grid
    grid_lines = []
    x = gx0
    while x <= xmax + 1e-9:
        grid_lines.append(
            f'<line x1="{sx(x):.2f}" y1="{sy(ymin):.2f}" x2="{sx(x):.2f}" y2="{sy(ymax):.2f}" class="grid"/>'
        )
        grid_lines.append(
            f'<text x="{sx(x):.2f}" y="{sy(ymin)+18:.2f}" class="tick">{x:g}</text>'
        )
        x += grid
    y = gy0
    while y <= ymax + 1e-9:
        grid_lines.append(
            f'<line x1="{sx(xmin):.2f}" y1="{sy(y):.2f}" x2="{sx(xmax):.2f}" y2="{sy(y):.2f}" class="grid"/>'
        )
        grid_lines.append(
            f'<text x="{sx(xmin)-8:.2f}" y="{sy(y)+4:.2f}" class="tick" text-anchor="end">{y:g}</text>'
        )
        y += grid

    # 桶和安全圆
    obs_svg = []
    for i, (x, y) in enumerate(obs):
        cx, cy = sx(x), sy(y)
        r_clear = max(2.0, sol.config.clearance * scale)
        obs_svg.append(
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r_clear:.2f}" class="clearance"/>'
        )
        obs_svg.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="7" class="barrel"/>')
        side = "L" if sol.sides[i] > 0 else "R"
        obs_svg.append(
            f'<text x="{cx+11:.2f}" y="{cy-11:.2f}" class="barrel-label">{i+1}{side}</text>'
        )

    # 通过点
    gate_svg = []
    for i, c in enumerate(sol.chosen):
        cx, cy = sx(c.pose.x), sy(c.pose.y)
        gate_svg.append(
            f'<path d="M {cx:.2f} {cy-5:.2f} L {cx+5:.2f} {cy:.2f} '
            f'L {cx:.2f} {cy+5:.2f} L {cx-5:.2f} {cy:.2f} Z" class="gate"/>'
        )

    # 分段路径
    segs = []
    colors = {"outbound": "#1565c0", "uturn": "#ef6c00", "return": "#2e7d32"}
    names = {"outbound": "绕桶段", "uturn": "180° 掉头", "return": "回程"}
    for name in ("outbound", "uturn", "return"):
        sp = [p for p in pts if p.segment == name]
        if not sp:
            continue
        segs.append(
            f'<path d="{path_d(sp)}" fill="none" stroke="{colors[name]}" '
            f'stroke-width="4.2" stroke-linecap="round" stroke-linejoin="round"/>'
        )

    # 方向箭头：每约 1.2m 一个
    arrows = []
    last_s = -1e9
    for i, p in enumerate(pts):
        if p.s - last_s < 1.2:
            continue
        last_s = p.s
        L = 0.28
        x1, y1 = p.x, p.y
        x2 = x1 + L * math.cos(p.heading)
        y2 = y1 + L * math.sin(p.heading)
        arrows.append(
            f'<line x1="{sx(x1):.2f}" y1="{sy(y1):.2f}" x2="{sx(x2):.2f}" y2="{sy(y2):.2f}" class="arrow"/>'
        )
        # 箭头头部
        hx = x2 - 0.08 * math.cos(p.heading - 0.5)
        hy = y2 - 0.08 * math.sin(p.heading - 0.5)
        arrows.append(
            f'<line x1="{sx(x2):.2f}" y1="{sy(y2):.2f}" x2="{sx(hx):.2f}" y2="{sy(hy):.2f}" class="arrow"/>'
        )

    # 起点终点
    s0 = sol.points[0]
    e0 = sol.points[-1]
    start_svg = (
        f'<circle cx="{sx(s0.x):.2f}" cy="{sy(s0.y):.2f}" r="9" class="start"/>'
        f'<text x="{sx(s0.x)+13:.2f}" y="{sy(s0.y)+5:.2f}" class="marker-label">起点 S</text>'
    )
    end_svg = (
        f'<rect x="{sx(e0.x)-8:.2f}" y="{sy(e0.y)-8:.2f}" width="16" height="16" class="end"/>'
        f'<text x="{sx(e0.x)+13:.2f}" y="{sy(e0.y)+5:.2f}" class="marker-label">终点 E</text>'
    )

    summary = sol.summary
    warn_lines = []
    for w in sol.warnings:
        # 简单换行
        while len(w) > 34:
            warn_lines.append(w[:34])
            w = w[34:]
        warn_lines.append(w)
    warn_svg = "".join(
        f'<text x="{W-275:.2f}" y="{530 + i*18:.2f}" class="warn">{escape_xml(s)}</text>'
        for i, s in enumerate(warn_lines[:10])
    )

    legend = f"""
    <g transform="translate({W-275}, 70)">
      <rect x="0" y="0" width="255" height="170" rx="8" class="panel"/>
      <line x1="14" y1="24" x2="54" y2="24" stroke="{colors['outbound']}" stroke-width="5"/>
      <text x="64" y="29" class="legend">绕桶段</text>
      <line x1="14" y1="50" x2="54" y2="50" stroke="{colors['uturn']}" stroke-width="5"/>
      <text x="64" y="55" class="legend">180° 掉头</text>
      <line x1="14" y1="76" x2="54" y2="76" stroke="{colors['return']}" stroke-width="5"/>
      <text x="64" y="81" class="legend">回程</text>
      <circle cx="34" cy="105" r="7" class="barrel"/>
      <text x="64" y="110" class="legend">桶 / 安全圆</text>
      <path d="M 29 128 L 39 138 L 29 148 L 19 138 Z" class="gate"/>
      <text x="64" y="142" class="legend">规划通过点</text>
    </g>
    """

    info = f"""
    <g transform="translate({W-275}, 270)">
      <rect x="0" y="0" width="255" height="235" rx="8" class="panel"/>
      <text x="14" y="26" class="info-title">规划结果</text>
      <text x="14" y="52" class="info">路径长度：{summary['length_m']:.2f} m</text>
      <text x="14" y="76" class="info">最小转弯半径：{summary['radius_used_m']:.2f} m</text>
      <text x="14" y="100" class="info">掉头半径：{summary['uturn_radius_m']:.2f} m</text>
      <text x="14" y="124" class="info">最小桶距：{summary['min_clearance_m']} m</text>
      <text x="14" y="148" class="info">最大通过偏移：{summary['max_gate_offset_m']:.2f} m</text>
      <text x="14" y="172" class="info">绕桶段最大偏离：{summary['max_corridor_deviation_m']:.2f} m</text>
      <text x="14" y="196" class="info">终点：({summary['end_x_m']:.2f}, {summary['end_y_m']:.2f})</text>
      <text x="14" y="220" class="info">终点朝向：{summary['end_heading_deg']:.0f}°</text>
    </g>
    """

    title = "智能车绕桶路径规划（x 前 / y 左 / m）"
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">
  <style>
    text {{ font-family: "Microsoft YaHei", "Noto Sans SC", Arial, sans-serif; }}
    .bg {{ fill: #fbfcfe; }}
    .grid {{ stroke: #e6ebf2; stroke-width: 1; }}
    .tick {{ fill: #8a96a6; font-size: 12px; }}
    .clearance {{ fill: none; stroke: #d66; stroke-width: 1.2; stroke-dasharray: 5 5; opacity: 0.75; }}
    .barrel {{ fill: #c62828; stroke: #8e0000; stroke-width: 1.5; }}
    .barrel-label {{ fill: #8e0000; font-size: 13px; font-weight: 700; }}
    .gate {{ fill: #ffffff; stroke: #6a1b9a; stroke-width: 2; }}
    .arrow {{ stroke: #37474f; stroke-width: 1.7; opacity: 0.8; }}
    .start {{ fill: #2e7d32; stroke: #1b5e20; stroke-width: 2; }}
    .end {{ fill: #c62828; stroke: #8e0000; stroke-width: 2; }}
    .marker-label {{ fill: #263238; font-size: 14px; font-weight: 700; }}
    .panel {{ fill: #ffffff; stroke: #d7dee8; stroke-width: 1; opacity: 0.96; }}
    .legend {{ fill: #37474f; font-size: 13px; }}
    .info-title {{ fill: #263238; font-size: 15px; font-weight: 700; }}
    .info {{ fill: #37474f; font-size: 13px; }}
    .warn {{ fill: #c62828; font-size: 12px; }}
    .title {{ fill: #1a237e; font-size: 21px; font-weight: 700; }}
  </style>
  <rect x="0" y="0" width="{W}" height="{H}" class="bg"/>
  <text x="{margin_l}" y="32" class="title">{title}</text>
  {''.join(grid_lines)}
  {''.join(obs_svg)}
  {''.join(segs)}
  {''.join(arrows)}
  {''.join(gate_svg)}
  {start_svg}
  {end_svg}
  {legend}
  {info}
  {warn_svg}
</svg>
"""


def escape_xml(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


# =============================================================================
# 输入解析 / 命令行
# =============================================================================


def parse_point_list(text: str) -> List[Tuple[float, float]]:
    """解析 "1,2 3,4" 或 "1 2; 3 4" 形式的点。"""
    cleaned = text.replace(";", " ").replace("|", " ").strip()
    if not cleaned:
        return []
    tokens = cleaned.split()
    pts: List[Tuple[float, float]] = []
    if all("," in t for t in tokens):
        for t in tokens:
            a, b = t.split(",", 1)
            pts.append((float(a), float(b)))
        return pts
    # 成对数字
    nums = [float(t) for t in tokens]
    if len(nums) % 2 != 0:
        raise ValueError("坐标个数必须是偶数")
    return [(nums[i], nums[i + 1]) for i in range(0, len(nums), 2)]


def load_input(args: argparse.Namespace) -> Tuple[List[Tuple[float, float]], Pose, Optional[List[int]]]:
    obstacles: List[Tuple[float, float]] = []
    start = Pose(0.0, 0.0, 0.0)
    sides: Optional[List[int]] = None

    if args.input:
        with open(args.input, "r", encoding="utf-8") as f:
            if args.input.lower().endswith(".json"):
                data = json.load(f)
                for item in data.get("obstacles", data.get("barrels", [])):
                    if isinstance(item, dict):
                        obstacles.append((float(item["x"]), float(item["y"])))
                    else:
                        obstacles.append((float(item[0]), float(item[1])))
                if "start" in data:
                    st = data["start"]
                    start = Pose(float(st[0]), float(st[1]), float(st[2]) if len(st) > 2 else 0.0)
                if "sides" in data:
                    sides = [1 if s in ("L", "l", "left", 1, "+") else -1 for s in data["sides"]]
            else:
                with open(args.input, "r", encoding="utf-8-sig", newline="") as csvf:
                    reader = csv.reader(csvf)
                    for row in reader:
                        if not row:
                            continue
                        try:
                            x = float(row[0].strip())
                            y = float(row[1].strip())
                        except (ValueError, IndexError):
                            continue  # 跳过表头
                        obstacles.append((x, y))
    elif args.points:
        obstacles = parse_point_list(args.points)
    else:
        raise ValueError("请用 --points 或 --input 提供桶坐标")

    if args.start:
        nums = [float(x) for x in args.start.replace(",", " ").split()]
        if len(nums) == 2:
            start = Pose(nums[0], nums[1], 0.0)
        elif len(nums) == 3:
            start = Pose(nums[0], nums[1], math.radians(nums[2]))
        else:
            raise ValueError("--start 格式为 x,y 或 x,y,yaw_deg")
    elif args.input:
        pass

    if args.sides:
        raw = args.sides.replace(",", " ").split()
        if len(raw) == 1 and len(raw[0]) > 1 and all(ch.upper() in ("L", "R") for ch in raw[0]):
            raw = list(raw[0])
        sides = [1 if s.upper().startswith("L") else -1 for s in raw]

    if not obstacles:
        raise ValueError("没有读到有效的桶坐标")
    return obstacles, start, sides


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="智能车左右交替绕桶 + 180° 掉头 + 回程路径规划器",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--points", help='直接给点，例如 "x1,y1 x2,y2 x3,y3"')
    g.add_argument("--input", help="JSON 或 CSV 文件；CSV 两列 x,y")
    p.add_argument("--start", help="起点 x,y 或 x,y,yaw_deg（默认 0,0,0）")
    p.add_argument("--sides", help='指定通过侧，如 "L,R,L"；默认左右交替')
    p.add_argument("--first-side", choices=("left", "right"), default="left", help="第一个桶从哪侧绕")
    p.add_argument("--clearance", type=float, default=0.45, help="路径中心到桶中心的最小安全距离 m")
    p.add_argument("--max-offset", type=float, default=1.20, help="通过点距桶中心的最大横向距离 m")
    p.add_argument("--corridor", type=float, default=1.60,
                   help="绕桶段离“起点-桶列”折线的最大距离 m，0 表示关闭")
    p.add_argument("--radius", type=float, default=1.00, help="目标最小转弯半径 m")
    p.add_argument("--min-radius", type=float, default=0.55, help="自动降半径的下限 m")
    p.add_argument("--uturn-radius", type=float, default=1.20, help="180° 掉头半径 m")
    p.add_argument("--exit", type=float, default=0.80, help="最后一个桶到掉头起点的前向距离 m")
    p.add_argument("--out", default="out_my", help="输出目录")
    p.add_argument("--heading-step", type=float, default=10.0, help="通过点切线角搜索步长 deg")
    p.add_argument("--heading-span", type=float, default=50.0, help="通过点切线角搜索半宽 deg")
    p.add_argument("--max-array-points", type=int, default=240, help="C 数组最多点数")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    # Windows 旧控制台默认可能是 GBK；统一按 UTF-8 输出中文提示。
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
        except Exception:
            pass

    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        obstacles, start, sides = load_input(args)
    except Exception as exc:
        print(f"[输入错误] {exc}", file=sys.stderr)
        return 2

    config = PlannerConfig(
        clearance=args.clearance,
        max_offset=args.max_offset,
        corridor_limit=args.corridor,
        radius=args.radius,
        min_radius=args.min_radius,
        uturn_radius=args.uturn_radius,
        first_side=1 if args.first_side == "left" else -1,
        sides=sides,
        heading_step_deg=args.heading_step,
        heading_span_deg=args.heading_span,
        exit_lengths=(args.exit, max(args.exit * 1.35, 0.9), 1.2, 1.6, 2.0),
        max_array_points=args.max_array_points,
    )

    try:
        sol = plan_path(obstacles, config, start)
    except Exception as exc:
        print(f"[规划失败] {exc}", file=sys.stderr)
        return 3

    files = write_outputs(sol, args.out)
    s = sol.summary
    print("=" * 64)
    print("规划完成")
    print(f"  桶数量          : {len(obstacles)}")
    print(f"  通过侧          : {' '.join('L' if x > 0 else 'R' for x in sol.sides)}")
    last_side = "左" if sol.sides[-1] > 0 else "右"
    turn_side = "右" if sol.sides[-1] > 0 else "左"
    print(f"  最后桶/掉头侧   : {last_side} / {turn_side}")
    print(f"  路径总长        : {s['length_m']:.3f} m")
    print(f"  绕桶段/掉头/回程: {s['outbound_length_m']:.3f} / "
          f"{s['uturn_length_m']:.3f} / "
          f"{s['return_length_m']:.3f} m")
    print(f"  最小转弯半径    : {s['radius_used_m']:.3f} m (目标 {s['requested_radius_m']:.3f} m)")
    print(f"  掉头半径        : {s['uturn_radius_m']:.3f} m")
    print(f"  最小桶距        : {s['min_clearance_m']} m (目标 {s['clearance_target_m']:.3f} m)")
    print(f"  绕桶段最大偏离  : {s['max_corridor_deviation_m']:.3f} m")
    print(f"  终点            : ({s['end_x_m']:.3f}, {s['end_y_m']:.3f})，"
          f"朝向 {s['end_heading_deg']:.1f}°")
    for w in sol.warnings:
        print(f"  [警告] {w}")
    print("-" * 64)
    for k, v in files.items():
        print(f"  {k:10s}: {v}")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    from robust_planner import main as robust_main
    raise SystemExit(robust_main())
