#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v23ログ起点の修正版: 斜め前回避・旋回反転・後退脱出を調整。

直線の微小補正と、凹みを追わない壁基準を使用。通常アクセル指令60。
GPIO/PWM初期設定・測距・実機入出力は元と同じ。
実機ではCtrl+Cで停止した後、走行中の測距・指令ログをUTF-8のTXTに保存する。
全制御周期の判断・出力PWMは、同じ走行IDのJSONLにも記録する。
車体モデルは仮値。実機の走行・復帰は未検証。
"""

import hashlib
import json
import math
import datetime
import os
import time

BRAKE_SECONDS = 0.20
REVERSE_NEUTRAL_SECONDS = 0.02
REVERSE_PULSE_SECONDS = 0.12
LOG_INTERVAL_SECONDS = 0.20
PROGRAM_VERSION = 'v28_confirmed_turn_diagonal_recovery'


def _clip(value, low, high):
    return max(low, min(high, value))


def select_start_direction(ranges):
    """前方3方向を比較。近い差なら正面、横は明確な差がある時だけ。"""
    front, left, right = (float(v) for v in ranges[:3])
    if not all(math.isfinite(v) and 0 < v <= 500 for v in (front, left, right)):
        return dict(angle_deg=0.0, direction="測距を再確認", valid=False)
    farthest = max(front, left, right)
    if farthest - front <= max(30.0, farthest * .1):
        return dict(angle_deg=0.0, direction="正面", valid=True)
    angle, direction = (45.0, "左前方") if left >= right else (-45.0, "右前方")
    return dict(angle_deg=angle, direction=direction, valid=True)


class _WallReferenceController:
    """右壁基準の前進判断と壁端への切り込み抑制。"""

    EDGE_STRAIGHT_SECONDS = 0.8
    EDGE_MEMORY_SECONDS = 6.0

    PARAMS = dict(
        target=48.0,
        angle_gain=2.7,
        distance_gain=1.1,
        turn_start=100.0,
        turn_release=150.0,
        turn_confirm_scans=2,
        range_filter_tau=0.06,
        straight_angle_gain=2.2,
        straight_distance_gain=0.45,
        straight_handle_limit=30.0,
        accel=60.0,
        fit_limit=55.0,
        max_handle=100.0,
        rate=260.0,
    )

    def __init__(self):
        self.params = dict(self.PARAMS)
        self._filtered = None
        self._handle = 0.0
        self._turn = False
        self._turn_reason = None
        self._turn_elapsed = 0.0
        self._turn_release_scans = 0
        self._turn_entry_scans = 0
        self._turn_candidate = None
        self._turn_exit_guard = 0.0
        self._turn_unknown_scans = 0
        self._opening_direction = None
        self._opening_scans = 0
        self._approach_scans = 0
        self._last_angle = 0.0
        self._edge_guard = False
        self._edge_confirm = 0
        self._edge_elapsed = 0.0
        self._edge_recent = 0.0
        self._turn_right = False
        self._follow_left = False
        self._right_target = None
        self._left_target = None
        self._startup_elapsed = 0.0
        self._start_preview = None
        self.debug = dict(
            p=0.0,
            d=0.0,
            corner=0.0,
            gap="right_wall",
            pp=0.0,
            lookahead_cm=0.0,
            narrow=False,
            side_close_count=0,
            safety_reason="",
        )

    def prepare_start(self, ranges):
        self._start_preview = dict(ranges_cm=list(ranges), **select_start_direction(ranges))
        return self._start_preview

    def update(self, Fr, FrLh, FrRh, RrLh, RrRh, dt):
        try:
            raw = [float(v) for v in (Fr, FrLh, FrRh, RrLh, RrRh)]
            dt = float(dt)
            valid = all(math.isfinite(v) and 0 < v <= 500 for v in raw)
            valid = valid and math.isfinite(dt) and 0 < dt <= 0.5
        except (ValueError, TypeError, OverflowError):
            valid = False

        if not valid:
            self._edge_guard = False
            self._edge_confirm = 0
            self._edge_elapsed = 0.0
            self._edge_recent = 0.0
            self._turn_right = False
            self._follow_left = False
            self._right_target = None
            self._left_target = None
            self._filtered = None
            self._handle = 0.0
            self._turn = False
            self._turn_reason = None
            self._turn_elapsed = 0.0
            self._turn_release_scans = 0
            self.debug["safety_reason"] = "測距・周期異常：停止"
            return 0.0, 0.0, self.debug["safety_reason"]

        if raw[0] < 16 or min(raw[1:3]) < 12 or min(raw[3:]) < 8:
            self._edge_guard = False
            self._edge_confirm = 0
            self._edge_elapsed = 0.0
            self._edge_recent = 0.0
            self._turn_right = False
            self.debug["safety_reason"] = "接近限界：停止"
            return 0.0, 0.0, self.debug["safety_reason"]

        self.debug["safety_reason"] = ""
        step = min(dt, 0.1)
        p = self.params
        self._startup_elapsed += step
        start_choice = select_start_direction(raw)
        startup_active = self._startup_elapsed <= 0.8

        if self._filtered is None:
            self._filtered = raw[:]

        alpha = 1 - math.exp(-step / p["range_filter_tau"])
        self._filtered = [
            a + alpha * (b - a)
            for a, b in zip(self._filtered, raw)
        ]
        f, fl, fr, sl, sr = self._filtered
        front = min(f, raw[0])

        # 右斜め・右側方から右壁の向きと間隔を推定。
        ray = fr / math.sqrt(2)
        slope = (1.5 + sr - ray) / (6 + ray)
        angle = math.degrees(math.atan(slope))
        reliable = (
            fr < 300
            and sr < 250
            and abs(angle) < p["fit_limit"]
        )

        if reliable:
            self._last_angle = angle
            clearance = (
                (9 + sr + 14 * slope)
                / math.sqrt(1 + slope * slope)
            )
            demand = (
                p["angle_gain"] * angle
                + p["distance_gain"] * (p["target"] - clearance)
            )
        else:
            # 壁が見えないことを旋回の指示にしない。
            demand = -60.0 if self._edge_recent > 0 and sr > 200 else 0.0

        # 右の凹みを追いかけず、左壁基準を使う。
        left_ray = fl / math.sqrt(2)
        left_slope = (left_ray - 1.5 - sl) / (6 + left_ray)
        left_angle = math.degrees(math.atan(left_slope))
        left_reliable = (
            fl < 250
            and sl < 170
            and abs(left_angle) < p["fit_limit"]
        )

        if self._follow_left:
            if not left_reliable or front < 120 or sr > 160:
                self._follow_left = False
        elif (
            left_reliable
            and abs(left_angle) < 15
            and front > 160
            and sr < 160
            and ((sl > 80 and ray > sr + 15) or sr > sl + 30)
        ):
            self._follow_left = True
            # 凹みの幅を追わず、切替直前の左壁との間隔を保つ。
            self._left_target = _clip(
                (9 + sl - 14 * left_slope) / math.sqrt(1 + left_slope * left_slope),
                40.0, 80.0,
            )

        if self._follow_left:
            left_clearance = (
                (9 + sl - 14 * left_slope)
                / math.sqrt(1 + left_slope * left_slope)
            )
            demand = (
                p["angle_gain"] * left_angle
                + p["distance_gain"] * (left_clearance - self._left_target)
            )

        straight = front > 160 and not self._turn
        reference_angle = left_angle if self._follow_left else angle
        reference_valid = left_reliable if self._follow_left else reliable
        if straight and reference_valid:
            if not self._follow_left and self._right_target is None and abs(angle) < 12:
                self._right_target = p["target"]
            distance_error = (
                left_clearance - self._left_target if self._follow_left
                else (self._right_target if self._right_target is not None else p["target"]) - clearance
            )
            # 小さな測距差で左右へ切り返さない。向きの補正を主に使う。
            a = math.copysign(max(0.0, abs(reference_angle) - 2.0), reference_angle)
            e = math.copysign(max(0.0, abs(distance_error) - 3.0), distance_error)
            demand = _clip(p["straight_angle_gain"] * a + p["straight_distance_gain"] * e,
                           -p["straight_handle_limit"], p["straight_handle_limit"])
        self.debug.update(straight=straight, wall_fit_valid=reference_valid,
                          wall_angle_deg=reference_angle,
                          right_target_cm=self._right_target, left_target_cm=self._left_target)
        # 発進直後も直前の測距で方向を選び直す。待機前の値だけで発進しない。
        if startup_active and front > 120 and min(raw[1:3]) > 35 and min(raw[3:]) > 25:
            if start_choice["angle_deg"]:
                demand = math.copysign(25.0, start_choice["angle_deg"])
            elif reference_valid and abs(reference_angle) < 12:
                demand = 0.0
        self.debug.update(startup_active=startup_active, startup_choice=start_choice)

        # 右壁の先端への早すぎる切り込みを抑える。
        self._edge_recent = max(0.0, self._edge_recent - step)
        edge_candidate = (
            not self._turn
            and not self._follow_left
            and front > 100
            and 15 < sr < 80.0
            and fr > max(150.0, 3 * sr)
        )
        self._edge_confirm = (
            self._edge_confirm + 1 if edge_candidate else 0
        )

        if self._edge_confirm == 2 and not self._edge_guard:
            self._edge_guard = True
            self._edge_elapsed = 0.0
            self._edge_recent = self.EDGE_MEMORY_SECONDS

        if self._edge_guard:
            self._edge_elapsed += step
            if (
                self._edge_elapsed > self.EDGE_STRAIGHT_SECONDS
                or front < 80
                or self._follow_left
            ):
                self._edge_guard = False
            else:
                demand = max(demand, 0.0)

        self.debug.update(
            edge_guard=self._edge_guard,
            edge_recent_seconds=self._edge_recent,
        )
        self.debug["wall_reference"] = (
            "left" if self._follow_left else "right"
        )

        # 左側の開きを確認できるカーブだけ早めに旋回。
        early_left = (
            front < 120.0
            and sl > 120
            and fl > fr + 40
            and fr < 120
        )

        # 340cmは最大レンジ/未検出でも出る。単独で「左が開いた」としない。
        left_open = ((45 < raw[1] < 300 and raw[1] > raw[2] + 25)
                     or (45 < raw[3] < 300 and raw[3] > raw[4] + 25))
        right_open = ((45 < raw[2] < 300 and raw[2] > raw[1] + 25)
                      or (45 < raw[4] < 300 and raw[4] > raw[3] + 25))
        if raw[1] >= 300 and 100 < raw[2] < 300 and raw[2] > raw[0] + 35:
            left_open, right_open = False, True
        elif raw[2] >= 300 and 100 < raw[1] < 300 and raw[1] > raw[0] + 35:
            left_open, right_open = True, False
        if self._edge_recent > 0 and 100 < raw[2] < 300 and raw[2] > raw[0] + 35:
            left_open, right_open = False, True
        if self._edge_recent > 0 and raw[2] > max(100, raw[1] + 30) and 45 < raw[4] < 300:
            left_open, right_open = False, True
        candidate = "left" if left_open and not right_open else "right" if right_open and not left_open else None
        wants_turn = raw[0] < p["turn_start"] or early_left
        self._opening_scans = self._opening_scans + 1 if candidate and candidate == self._opening_direction else (1 if candidate else 0)
        self._opening_direction = candidate
        self._approach_scans = self._approach_scans + 1 if raw[0] < 160 else 0
        if not self._turn:
            self._turn_unknown_scans = self._turn_unknown_scans + 1 if wants_turn and candidate is None else 0
            self._turn_entry_scans = self._turn_entry_scans + 1 if wants_turn and candidate == self._turn_candidate and candidate else (1 if wants_turn and candidate else 0)
            self._turn_candidate = candidate if wants_turn else None
            if candidate and self._opening_scans >= 2 and self._approach_scans >= 2:
                self._turn_entry_scans = max(self._turn_entry_scans, p["turn_confirm_scans"])
        self.debug.update(turn_candidate=self._turn_candidate or "unknown",
                          turn_entry_scans=self._turn_entry_scans,
                          left_open_confirmed=left_open, right_open_confirmed=right_open)
        if wants_turn and not self._turn and self._turn_entry_scans < p["turn_confirm_scans"]:
            if candidate:
                self.debug.update(safety_reason="", p=self._handle, corner=0.0)
                return p["accel"] * 0.5, self._handle, "旋回方向を連続確認：現在の舵を維持"
            self._handle = 0.0
            reason = "正面接近：旋回方向の根拠不足" if self._turn_unknown_scans >= 3 else "正面接近：旋回方向を再確認"
            self.debug.update(safety_reason=reason, p=0.0, corner=0.0)
            return 0.0, 0.0, reason

        if wants_turn and not self._turn:
            self._turn = True
            self._turn_reason = (
                "early_left_opening" if early_left else "front_close"
            )
            self._turn_elapsed = 0.0
            self._turn_release_scans = 0
            self._turn_right = candidate == "right"

        straight_exit = (raw[0] >= 300 and max(raw[1:3]) < 300
                         and abs(raw[1] - raw[2]) < 20
                         and max(raw[3:]) < 300 and abs(raw[3] - raw[4]) < 20)
        if self._turn:
            self._turn_elapsed += step
            self._turn_release_scans = (
                self._turn_release_scans + 1
                if (raw[0] if straight_exit else front) > p["turn_release"]
                else 0
            )
            if self._turn_elapsed >= 0.12 and self._turn_release_scans >= 2:
                self._turn = False
                self._right_target = None
                self._turn_reason = None
                self._turn_elapsed = 0.0
                self._turn_release_scans = 0
                self._turn_entry_scans = 0
                if straight_exit:
                    self._turn_exit_guard = 0.2
                    self._handle = 0.0

        if not self._turn and self._turn_exit_guard > 0 and raw[0] > p["turn_release"]:
            self._turn_exit_guard = max(0.0, self._turn_exit_guard - step)
            demand = 0.0

        if self._turn:
            demand = (
                -p["max_handle"]
                if self._turn_right
                else p["max_handle"]
            )

        if raw[2] < 35 and front > 50:
            demand = max(demand, 50.0)
        if raw[1] < 25 and front > 50:
            demand = min(demand, -50.0)

        demand = _clip(
            demand, -p["max_handle"], p["max_handle"]
        )
        self._handle += _clip(
            demand - self._handle,
            -p["rate"] * step,
            p["rate"] * step,
        )

        # 狭路では操舵による減速を掛けず、通常指令を維持。
        narrow = (
            raw[0] >= 90.0
            and min(raw[1:3]) >= 25.0
            and max(raw[1:3]) <= 100.0
            and max(raw[3:]) <= 65.0
        )
        accel = (
            p["accel"]
            if narrow
            else p["accel"]
            * max(0.5, 1 - 0.3 * abs(self._handle) / 100)
        )

        self.debug["narrow"] = narrow
        self.debug.update(
            p=demand,
            d=angle,
            corner=float(self._turn),
            turn_reason=self._turn_reason or "none",
            turn_elapsed=round(self._turn_elapsed, 3),
            turn_release_scans=self._turn_release_scans,
            pp=self._handle,
            lookahead_cm=front,
            gap=(
                ("right_corner" if self._turn_right else "left_corner")
                if self._turn
                else "right_wall"
            ),
        )

        if self._turn:
            comment = (
                "壁端の開きへ右旋回"
                if self._turn_right
                else "右壁基準・左旋回"
            )
        elif self._edge_guard:
            comment = "壁端回避：右への切り込みを一時抑制"
        elif self._follow_left:
            comment = "凹み回避：左壁との間隔を維持"
        else:
            comment = "右壁の向きと間隔を維持"

        return accel, self._handle, comment


class GapFinder:
    """最遠センサーと隣接センサーから目標方向を選ぶ。"""

    def __init__(self, angles=(-45.0, 0.0, 45.0), hysteresis_cm=10.0):
        if len(angles) < 2 or any(
            a >= b for a, b in zip(angles, angles[1:])
        ):
            raise ValueError("Angles must be increasing")

        self.angles = tuple(angles)
        self.hysteresis = float(hysteresis_cm)
        self.peak = None

    def reset(self):
        self.peak = None

    def select(self, distances, allowed=None):
        if len(distances) != len(self.angles):
            raise ValueError("Range count mismatch")

        if not all(math.isfinite(d) and d > 0 for d in distances):
            self.reset()
            return None

        allowed = (
            list(allowed)
            if allowed is not None
            else [True] * len(distances)
        )
        if len(allowed) != len(distances):
            raise ValueError("Allowed count mismatch")

        valid = [i for i, yes in enumerate(allowed) if yes]
        if not valid:
            self.reset()
            return None

        best = max(
            valid,
            key=lambda i: (distances[i], -abs(self.angles[i])),
        )

        if (
            self.peak in valid
            and distances[best]
            <= distances[self.peak] + self.hysteresis
        ):
            best = self.peak

        self.peak = best
        neighbors = [i for i in (best - 1, best + 1) if i in valid]

        if not neighbors:
            return dict(
                peak=best,
                angle=self.angles[best],
                contributors=[best],
                method="single_ray",
            )

        if len(neighbors) == 2:
            weights = [
                distances[i]
                * distances[best]
                * math.sin(
                    math.radians(
                        abs(self.angles[i] - self.angles[best])
                    )
                )
                for i in neighbors
            ]
            total = sum(weights)

            if total > 0:
                angle = sum(
                    self.angles[i] * w
                    for i, w in zip(neighbors, weights)
                ) / total
                return dict(
                    peak=best,
                    angle=angle,
                    contributors=[best] + neighbors,
                    method="adjacent_area",
                )

        indices = [best] + neighbors
        total = sum(distances[i] for i in indices)
        angle = sum(
            self.angles[i] * distances[i] for i in indices
        ) / total

        return dict(
            peak=best,
            angle=angle,
            contributors=indices,
            method="edge_distance",
        )


class LocalGapPlanner:
    """疎な観測点に対して車体余裕を確認する局所プランナー。"""

    def __init__(
        self,
        wheelbase_cm=17.0,
        width_cm=20.0,
        margin_cm=8.0,
        minimum_cm=45.0,
        maximum_cm=90.0,
        front_offset_cm=30.0,
    ):
        self.finder = GapFinder(hysteresis_cm=10.0)
        self.wheelbase = wheelbase_cm
        self.width = width_cm
        self.margin = margin_cm
        self.minimum = minimum_cm
        self.maximum = maximum_cm
        self.offset = front_offset_cm
        self.last_reason = "not_planned"

    def reset(self):
        self.finder.reset()
        self.last_reason = "reset"

    def pursuit(self, angle_deg, distance_cm):
        if not (
            math.isfinite(distance_cm)
            and distance_cm > 0
            and math.isfinite(angle_deg)
        ):
            return None

        a = math.radians(angle_deg)
        curvature = 2 * math.sin(a) / distance_cm
        delta = math.degrees(
            math.atan(self.wheelbase * curvature)
        )

        if abs(delta) > 25.0 + 1e-9:
            return None

        return dict(
            handle=_clip(delta / 25.0 * 100, -100, 100),
            curvature=curvature,
            goal_x=distance_cm * math.cos(a),
            goal_y=distance_cm * math.sin(a),
        )

    def path_clear(self, angle_deg, distance_cm, observed):
        a = math.radians(angle_deg)
        k = 2 * math.sin(a) / distance_cm
        arc = (
            distance_cm
            if abs(a) < 1e-9
            else distance_cm * a / math.sin(a)
        )
        return self.path_clear_arc(k, arc, observed)

    def path_clear_arc(self, k, arc, observed):
        count = max(1, int(math.ceil(abs(arc) / 4)))

        for j in range(count + 1):
            length = arc * j / count
            yaw = k * length

            if abs(k) < 1e-9:
                x, y = length, 0.0
            else:
                x = math.sin(yaw) / k
                y = (1 - math.cos(yaw)) / k

            c, s = math.cos(yaw), math.sin(yaw)

            for px, py in observed:
                dx, dy = px - x, py - y
                bx = c * dx + s * dy
                by = -s * dx + c * dy

                gap_x = max(-4.0 - bx, 0.0, bx - 24.0)
                gap_y = max(abs(by) - self.width / 2, 0.0)

                if math.hypot(gap_x, gap_y) < self.margin:
                    return False

        return True

    def plan(self, ranges):
        f, fl, fr, sl, sr = ranges
        distances = (fr, f, fl)

        observed = [
            (23 + f, 0.0),
            (20 + fl / math.sqrt(2), 7.5 + fl / math.sqrt(2)),
            (20 + fr / math.sqrt(2), -7.5 - fr / math.sqrt(2)),
            (14.0, 9 + sl),
            (14.0, -9 - sr),
        ]
        allowed = [
            d < 339.0 and d - 20 >= self.minimum
            for d in distances
        ]
        blocked = []
        desired = _clip(
            f - self.offset, self.minimum, self.maximum
        )

        for _ in range(3):
            choice = self.finder.select(distances, allowed)

            if choice is None:
                self.last_reason = "no_clear_candidate"
                return None

            available = min(
                distances[i] for i in choice["contributors"]
            ) - 20.0
            feasible = (
                2
                * self.wheelbase
                * abs(math.sin(math.radians(choice["angle"])))
                / math.tan(math.radians(25.0))
            )
            lookahead = max(desired, feasible)

            if (
                lookahead <= min(available, self.maximum)
                and self.path_clear(
                    choice["angle"], lookahead, observed
                )
            ):
                pp = self.pursuit(choice["angle"], lookahead)
                self.last_reason = "observed_points_clear"

                if pp is None:
                    allowed[choice["peak"]] = False
                    continue

                return dict(
                    choice,
                    **pp,
                    lookahead=lookahead,
                    blocked=blocked,
                    observed=observed,
                    selected_range=distances[choice["peak"]],
                )

            allowed[choice["peak"]] = False
            blocked.append(choice["peak"])

        self.last_reason = "body_margin_rejected"
        return None


class SmoothDriveController(_WallReferenceController):
    """壁基準の前進判断にGap/Pure Pursuit補助を加える。"""

    GAP_BLEND = 0.05

    def __init__(self):
        super().__init__()
        self.gap_planner = LocalGapPlanner()

    def update(self, Fr, FrLh, FrRh, RrLh, RrRh, dt):
        previous_handle = self._handle
        accel, handle, comment = super().update(
            Fr, FrLh, FrRh, RrLh, RrRh, dt
        )

        self.debug.update(
            pp_applied=False,
            planner_reason="wall_reference",
            gap="none",
            gap_angle_deg=None,
            gap_method="none",
            gap_selected_range_cm=None,
            goal_x=None,
            goal_y=None,
            curvature=0.0,
            gap_blocked=[],
            pp=0.0,
            lookahead_cm=0.0,
        )

        if accel <= 0:
            self.gap_planner.reset()
            self._handle = 0.0
            return accel, handle, comment

        raw = tuple(
            float(v) for v in (Fr, FrLh, FrRh, RrLh, RrRh)
        )
        plan = self.gap_planner.plan(raw)
        self.debug["planner_reason"] = self.gap_planner.last_reason

        if plan is None:
            if max(raw[:3]) < 65:
                self._handle = 0.0
                self.debug["safety_reason"] = (
                    "車幅余裕のある進路候補なし：停止"
                )
                return 0.0, 0.0, self.debug["safety_reason"]

            return accel, handle, comment

        self.debug.update(
            gap=("far_right", "far_front", "far_left")[plan["peak"]],
            gap_angle_deg=plan["angle"],
            gap_method=plan["method"],
            gap_selected_range_cm=plan["selected_range"],
            lookahead_cm=plan["lookahead"],
            goal_x=plan["goal_x"],
            goal_y=plan["goal_y"],
            pp=plan["handle"],
            curvature=plan["curvature"],
            gap_blocked=plan["blocked"],
        )

        protected = (
            min(raw[1:3]) < 35
            or min(raw[3:]) < 20
            or max(raw[3:]) >= 330
        )
        same_direction = plan["handle"] * handle > 0

        if (
            self._turn
            and not self._follow_left
            and not protected
            and same_direction
        ):
            target = (
                (1 - self.GAP_BLEND) * handle
                + self.GAP_BLEND * plan["handle"]
            )
            step = min(float(dt), 0.1)

            handle = previous_handle + _clip(
                target - previous_handle,
                -self.params["rate"] * step,
                self.params["rate"] * step,
            )
            handle = _clip(handle, -100.0, 100.0)

            curvature = (
                math.tan(math.radians(25.0 * handle / 100))
                / self.gap_planner.wheelbase
            )

            if not self.gap_planner.path_clear_arc(
                curvature,
                min(plan["lookahead"], 60.0),
                plan["observed"],
            ):
                self.debug["planner_reason"] = "blend_margin_rejected"
                return accel, self._handle, comment

            self._handle = handle
            accel = (
                self.params["accel"]
                if self.debug["narrow"]
                else self.params["accel"]
                * max(0.5, 1 - 0.3 * abs(self._handle) / 100)
            )
            self.debug["pp_applied"] = True
            comment = "Gap＋PPでカーブ補助（壁基準を維持）"

        return accel, self._handle, comment


# 上の前進用クラスを保存し、後退復帰付きクラスで拡張する。
_ForwardDriveController = SmoothDriveController


class SmoothDriveController(_ForwardDriveController):
    """後退方向を保ち、接触側の実測余裕を確認して低速で離脱する。"""

    PARAMS = dict(_ForwardDriveController.PARAMS,
                  reverse_trigger_cm=25.0,
                  recovery_front_release_cm=35.0,
                  recovery_side_release_cm=25.0,
                  recovery_clear_scans=3,
                  recovery_confirm_timeout_s=0.8,
                  recovery_min_pulses=3,
                  recovery_escape_seconds=0.5,
                  recovery_probe_seconds=2.0,
                  recovery_escape_accel=25.0)
    MAX_RECOVERY_PULSES = 60

    def __init__(self):
        super().__init__()
        self._recovery_pulses = 0
        self._recovery_wait = False
        self._clear_scans = 0
        self._clear_history = []
        self._confirmation_elapsed = 0.0
        self._forward_seconds = 0.0
        self._reverse_handle = 0.0
        self._recovery_side_index = None
        self._side_release = 25.0
        self._recovery_phase = "drive"
        self._escape_seconds = 0.0
        self._side_contact = False
        self._closest_side = 500.0
        self._closest_diagonal = 500.0
        self._clear_mode = "clear"
        self._escape_clear_scans = 0
        self._recovery_origin = "front"
        self._recovery_observation_index = None
        self._observation_release = 25.0
        self._front_fault_history = []

    def _near(self, raw):
        trigger = self.params["reverse_trigger_cm"]
        return (raw[0] < trigger
                or min(raw[1:3]) < trigger
                or min(raw[3:]) < 8)

    def update(self, Fr, FrLh, FrRh, RrLh, RrRh, dt):
        try:
            raw = tuple(float(v) for v in (Fr, FrLh, FrRh, RrLh, RrRh))
            elapsed = float(dt)
            valid = (all(math.isfinite(v) and 0 < v <= 500 for v in raw)
                     and math.isfinite(elapsed) and 0 < elapsed <= 0.5)
        except (ValueError, TypeError, OverflowError):
            valid = False
        if not valid:
            self._clear_history = []
            self._clear_scans = 0
            self._forward_seconds = 0.0
            _ForwardDriveController.__init__(self)
            self._startup_elapsed = 1.0
            return self._recovery_stop("測距・周期異常：停止")

        if self._recovery_phase == "blocked":
            return self._recovery_stop("後退上限：手動確認")
        if self._recovery_phase == "reverse":
            side = raw[self._recovery_side_index]
            diagonal = raw[2 if self._recovery_side_index == 4 else 1]
            self._closest_side = min(self._closest_side, side)
            self._closest_diagonal = min(self._closest_diagonal, diagonal)
            self._side_contact = self._side_contact or side < 12
            if self._recovery_wait:
                self._recovery_wait = False
                return self._recovery_stop("後退後：舵を保って停止・再測距", hold=True)
            # 340cm付近への跳躍を「接触側が離れた」証拠にしない。
            released = (not self._near(raw)
                        and raw[0] >= self.params["recovery_front_release_cm"]
                        and min(raw[1:3]) >= 18 and min(raw[3:]) >= 12
                        and self._observation_release <= raw[self._recovery_observation_index] < 300)
            opposite = raw[3 if self._recovery_side_index == 4 else 4]
            # 壁に並行して近い時、後退旋回だけを続けると車体後部が壁へ寄る。
            # 実測の改善を確認できれば、低速の離脱操舵で横方向の余裕を作る。
            improved = (side >= self._closest_side + 2
                        or (diagonal < 300 and diagonal >= self._closest_diagonal + 2))
            probe = (self._side_contact and improved and raw[0] >= self.params["recovery_front_release_cm"]
                     and 6 <= side < 300 and diagonal >= 12 and opposite >= 25)
            if (released or probe) and self._recovery_pulses >= self.params["recovery_min_pulses"]:
                mode = "clear" if released else "side_probe"
                if mode != self._clear_mode:
                    self._clear_history = []
                self._clear_mode = mode
                self._confirmation_elapsed += elapsed
                count = self.params["recovery_clear_scans"]
                observation = raw[self._recovery_observation_index] if released else side
                self._clear_history = (self._clear_history + [observation])[-count:]
                self._clear_scans = len(self._clear_history)
                stable = max(self._clear_history) - min(self._clear_history) <= 10
                if self._clear_scans < count or not stable:
                    if self._confirmation_elapsed >= self.params["recovery_confirm_timeout_s"]:
                        return self._reverse_step(raw)
                    return self._recovery_stop("接触側の実測余裕を連続確認", hold=True)
                self._recovery_phase = "escape"
                self._escape_seconds = 0.0
                self._escape_clear_scans = 0
            else:
                self._clear_history = []
                self._clear_scans = 0
                return self._reverse_step(raw)

        if self._near(raw) and self._recovery_phase != "escape":
            return self._begin_recovery(raw)
        if self._recovery_phase == "escape":
            side = raw[self._recovery_side_index]
            diagonal = raw[2 if self._recovery_side_index == 4 else 1]
            opposite = raw[3 if self._recovery_side_index == 4 else 4]
            probe = self._clear_mode == "side_probe"
            observation = raw[self._recovery_observation_index]
            observation_min = self.params["reverse_trigger_cm"] if self._recovery_origin == "diagonal" else self._observation_release - 3
            unsafe = (raw[0] < self.params["recovery_front_release_cm"]
                      or (side >= 300 and self._recovery_origin == "side") or opposite < 12
                      or (probe and (side < 6 or diagonal < 12))
                      or (not probe and (self._near(raw) or observation < observation_min)))
            if unsafe:
                self._recovery_phase = "reverse"
                return self._reverse_step(raw)
            # 復帰直後は通常の壁追従へ戻さず、近かった側から低速で離れる。
            handle = -self._reverse_handle / 2
            escape_ray = raw[2] if handle < 0 else raw[1]
            escape_side = raw[4] if handle < 0 else raw[3]
            if escape_ray <= self.params["reverse_trigger_cm"] or escape_side < 25:
                handle = 0.0
            self._handle = handle
            self._escape_seconds += elapsed
            self._forward_seconds += elapsed
            self._recovery_debug(raw)
            self.debug.update(p=handle, pp_applied=False, safety_reason="",
                              recovery_escape_elapsed=self._escape_seconds)
            real_clear = self._observation_release <= observation < 300
            # 斜めの壁が射線から外れた場合は、直前の斜め実測確認に加え
            # 現在の側面実測でも余裕を確認する。340cm単独では解除しない。
            passed_diagonal = (self._recovery_origin == "diagonal" and observation >= 300
                               and self._side_release <= side < 300)
            self._escape_clear_scans = self._escape_clear_scans + 1 if (real_clear or passed_diagonal) and not self._near(raw) else 0
            if self._escape_seconds >= self.params["recovery_escape_seconds"] and self._escape_clear_scans >= 3:
                self._recovery_phase = "drive"
            elif self._escape_seconds >= self.params["recovery_probe_seconds"]:
                self._recovery_phase = "reverse"
                return self._reverse_step(raw)
            return self.params["recovery_escape_accel"], handle, "後退復帰：接触側から低速離脱"

        accel, handle, comment = super().update(*raw, elapsed)
        if accel == 0 and self.debug.get("safety_reason") in ("車幅余裕のある進路候補なし：停止", "正面接近：旋回方向の根拠不足"):
            return self._begin_recovery(raw)
        if accel > 0:
            self._forward_seconds += elapsed
            if self._forward_seconds >= 1.0:
                self._recovery_pulses = 0
                self._reverse_handle = 0.0
                self._recovery_side_index = None
                self._clear_scans = 0
                self._clear_history = []
        else:
            self._forward_seconds = 0.0
        self._recovery_debug(raw)
        return accel, handle, comment

    def _begin_recovery(self, raw):
        if not self._reverse_handle:
            right_min, left_min = min(raw[2], raw[4]), min(raw[1], raw[3])
            if min(raw[3:]) < 8:
                right = raw[4] <= raw[3]
                origin = "side"
            elif min(raw[1:3]) < self.params["reverse_trigger_cm"]:
                right = raw[2] <= raw[1]
                origin = "diagonal"
            else:
                right = right_min <= left_min
                origin = "front"
            self._reverse_handle = -40.0 if right else 40.0
            self._recovery_side_index = 4 if self._reverse_handle < 0 else 3
            self._side_release = self.params["recovery_side_release_cm"]
            self._recovery_origin = origin
            # 斜め前の壁は同じ斜めセンサーの改善で確認。遠い側面を待ち続けない。
            self._recovery_observation_index = (2 if right else 1) if origin == "diagonal" else (0 if origin == "front" and raw[self._recovery_side_index] >= 300 else self._recovery_side_index)
            self._observation_release = 35.0 if self._recovery_observation_index < 3 else self._side_release
            self._side_contact = raw[self._recovery_side_index] < 25
            self._closest_side = raw[self._recovery_side_index]
            self._closest_diagonal = raw[2 if self._recovery_side_index == 4 else 1]
        # 前進の壁追従履歴は後退開始時だけ破棄し、後退の舵は途中で変えない。
        _ForwardDriveController.__init__(self)
        self._startup_elapsed = 1.0
        self._recovery_phase = "reverse"
        self._forward_seconds = 0.0
        self._clear_history = []
        self._clear_scans = 0
        return self._reverse_step(raw)

    def _recovery_debug(self, raw=None):
        self.debug.update(recovery_phase=self._recovery_phase,
                          recovery_side=("右" if self._reverse_handle < 0 else "左") if self._reverse_handle else "none",
                          recovery_handle=self._reverse_handle,
                          recovery_pulses=self._recovery_pulses,
                          recovery_side_release_cm=self._side_release,
                          recovery_clear_scans=self._clear_scans,
                          recovery_confirm_elapsed=self._confirmation_elapsed,
                          recovery_direction_locked=bool(self._reverse_handle))
        self.debug.update(recovery_clear_mode=self._clear_mode,
                          recovery_origin=self._recovery_origin,
                          recovery_observation_index=self._recovery_observation_index,
                          recovery_observation_release_cm=self._observation_release,
                          recovery_closest_side_cm=self._closest_side,
                          recovery_closest_diagonal_cm=self._closest_diagonal)
        if raw is not None:
            self.debug.update(recovery_right_min_cm=min(raw[2],raw[4]),
                              recovery_left_min_cm=min(raw[1],raw[3]),
                              recovery_contact_side_cm=raw[self._recovery_side_index] if self._recovery_side_index is not None else None)

    def _recovery_stop(self, reason, hold=False):
        handle = self._reverse_handle if hold else 0.0
        self._handle = handle
        self.gap_planner.reset()
        self._recovery_debug()
        self.debug.update(safety_reason=reason, pp_applied=False)
        return 0.0, handle, reason

    def _reverse_step(self, raw):
        self._forward_seconds = 0.0
        self._confirmation_elapsed = 0.0
        self._clear_history = []
        self._clear_scans = 0
        if raw[0] < 3:
            self._front_fault_history = (self._front_fault_history + [raw[0]])[-8:]
            if self._recovery_pulses >= 8 and len(self._front_fault_history) == 8 and max(self._front_fault_history) - min(self._front_fault_history) < 1:
                self._recovery_phase = "blocked"
                return self._recovery_stop("正面の極小値が固定：測距・移動状態を確認")
        else:
            self._front_fault_history = []
        if self._recovery_pulses >= self.MAX_RECOVERY_PULSES:
            self._recovery_phase = "blocked"
            return self._recovery_stop("後退{}回で復帰余裕なし：手動確認".format(self.MAX_RECOVERY_PULSES))
        self._recovery_pulses += 1
        self._recovery_wait = True
        self._handle = self._reverse_handle
        self._recovery_debug(raw)
        reason = "後退の{}旋回：舵を保ち短く後退 {}/{}".format(
            "右" if self._reverse_handle < 0 else "左", self._recovery_pulses, self.MAX_RECOVERY_PULSES)
        self.debug.update(safety_reason=reason, pp_applied=False)
        return -50.0, self._reverse_handle, reason


# 障害物センサ測定関数：元プログラムと同じ
def Mesure(GPIO,time,trig,echo):
    sigoff = 0
    sigon = 0
    GPIO.output(trig,GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig,GPIO.LOW)
    kijyun=time.time()
    while(GPIO.input(echo)==GPIO.LOW):
        sigoff=time.time()
        if sigoff - kijyun > 0.02:
            break
    while(GPIO.input(echo)==GPIO.HIGH):
        sigon=time.time()
        if sigon - sigoff > 0.02:
            break
    dis = (sigon - sigoff)*34000/2
    return dis


def main():
    # 実機用ライブラリは走行開始時だけ読み込む。
    import RPi.GPIO as GPIO
    import Adafruit_PCA9685

    # GPIO初期設定
    GPIO.setmode(GPIO.BOARD)
    GPIO.cleanup()

    # 超音波センサ初期設定
    t_list=[15,13,35,32,36]
    GPIO.setup(t_list,GPIO.OUT,initial=GPIO.LOW)
    e_list=[33,40,37,31,38]
    GPIO.setup(e_list,GPIO.IN)

    # PWM初期化
    pwm = Adafruit_PCA9685.PCA9685(address=0x40)
    pwm.set_pwm_freq(60)

    STEERING_CHANNEL = 1
    STEERING_RIGHT_PWM = 330
    STEERING_CENTER_PWM = 435
    STEERING_LEFT_PWM = 540

    THROTTLE_CHANNEL = 7
    THROTTLE_FORWARD_PWM = 500
    THROTTLE_STOPPED_PWM = 375
    THROTTLE_REVERSE_PWM = 220

    pwm.set_pwm(THROTTLE_CHANNEL, 0, THROTTLE_STOPPED_PWM)
    pwm.set_pwm(STEERING_CHANNEL, 0, STEERING_CENTER_PWM)

    controller = SmoothDriveController()
    reverse_ready = False
    last_log = float('-inf')
    last_comment = None
    log_file = None
    log_path = None
    trace_file = None
    trace_path = None
    log_records = 0
    stop_reason = '走行開始前に終了'
    run_started = None

    try:
        print('実行プログラム: {} ({})'.format(PROGRAM_VERSION, os.path.basename(__file__)))
        # GPIO/PWM設定を変えず、停止したまま3回の測距の中央値を表示する。
        startup_scans = []
        for _ in range(3):
            startup_scans.append([
                Mesure(GPIO,time,15,33), Mesure(GPIO,time,13,40),
                Mesure(GPIO,time,32,31), Mesure(GPIO,time,35,37),
                Mesure(GPIO,time,36,38),
            ])
        startup_ranges = [sorted(values)[1] for values in zip(*startup_scans)]
        preview = controller.prepare_start(startup_ranges)
        print('開始前測距(cm): Fr={:.0f} 左前={:.0f} 右前={:.0f} 左側={:.0f} 右側={:.0f}'.format(*startup_ranges))
        print('開始方向候補: {}（Enter後に再測距して判断）'.format(preview['direction']))
        input('Press Enter to start...')
        if not hasattr(GPIO, 'runner'):
            started_at = datetime.datetime.now()
            run_stamp = started_at.strftime('%Y%m%d_%H%M%S_%f')
            log_path = os.path.abspath(os.path.join(
                os.getcwd(), 'drive_log_{}.txt'.format(run_stamp)))
            trace_path = os.path.abspath(os.path.join(
                os.getcwd(), 'drive_trace_{}.jsonl'.format(run_stamp)))
            log_file = open(log_path, 'w', encoding='utf-8')
            trace_file = open(trace_path, 'w', encoding='utf-8')
            log_file.write('走行ログ開始: {}\n'.format(started_at.isoformat(timespec='seconds')))
            log_file.write('5センサー・制御判断・出力PWM（全制御周期）\n')
            log_file.flush()
            with open(__file__, 'rb') as program_file:
                program_sha256 = hashlib.sha256(program_file.read()).hexdigest()
            log_file.write('プログラム: {} / {} / SHA256={}\n'.format(
                PROGRAM_VERSION, os.path.basename(__file__), program_sha256))
            start_record = {
                'type': 'start',
                'schema_version': 1,
                'run_id': run_stamp,
                'started_at': started_at.isoformat(timespec='milliseconds'),
                'program_path': os.path.abspath(__file__),
                'program_sha256': program_sha256,
                'program_version': PROGRAM_VERSION,
                'sensor_order': ['Fr', 'FrLh', 'FrRh', 'RrLh', 'RrRh'],
                'sensor_pins': {
                    'Fr': {'trig': 15, 'echo': 33},
                    'FrLh': {'trig': 13, 'echo': 40},
                    'FrRh': {'trig': 32, 'echo': 31},
                    'RrLh': {'trig': 35, 'echo': 37},
                    'RrRh': {'trig': 36, 'echo': 38},
                },
                'sensor_unit': 'cm',
                'clock': 'elapsed_s and dt_s use time.monotonic()',
                'controller': type(controller).__name__,
                'controller_parameters': dict(getattr(controller, 'params', {})),
                'start_observation': preview,
                'pwm': {
                    'frequency_hz': 60,
                    'steering': {
                        'channel': STEERING_CHANNEL,
                        'right': STEERING_RIGHT_PWM,
                        'center': STEERING_CENTER_PWM,
                        'left': STEERING_LEFT_PWM,
                    },
                    'throttle': {
                        'channel': THROTTLE_CHANNEL,
                        'forward': THROTTLE_FORWARD_PWM,
                        'stopped': THROTTLE_STOPPED_PWM,
                        'reverse': THROTTLE_REVERSE_PWM,
                    },
                },
            }
            trace_file.write(json.dumps(start_record, ensure_ascii=False) + '\n')
            trace_file.flush()
        t = time.monotonic()
        run_started = t
        stop_reason = '停止理由不明'
        iteration = 0

        while True:
            Fr   = Mesure(GPIO,time,15,33)
            FrLh = Mesure(GPIO,time,13,40)
            FrRh = Mesure(GPIO,time,32,31)
            RrLh = Mesure(GPIO,time,35,37)
            RrRh = Mesure(GPIO,time,36,38)

            now = time.monotonic()
            dt = now - t
            t = now

            Accel, Handle, comment = controller.update(
                Fr, FrLh, FrRh, RrLh, RrRh, dt
            )
            decision_elapsed = time.monotonic() - run_started

            # ステアリングPWM換算：元プログラムと同じ
            if Handle <= 0:
                steer_pwm = int(STEERING_CENTER_PWM + (STEERING_CENTER_PWM - STEERING_RIGHT_PWM) * Handle / 100)
                pwm.set_pwm(STEERING_CHANNEL, 0, steer_pwm)
            else:
                steer_pwm = int(STEERING_CENTER_PWM + (STEERING_LEFT_PWM - STEERING_CENTER_PWM) * Handle / 100)
                pwm.set_pwm(STEERING_CHANNEL, 0, steer_pwm)

            # スロットルPWM換算：元プログラムと同じ
            throttle_events = []
            if Accel > 0:
                throttle_pwm = int(THROTTLE_STOPPED_PWM + (THROTTLE_FORWARD_PWM - THROTTLE_STOPPED_PWM) * Accel / 100)
                pwm.set_pwm(THROTTLE_CHANNEL, 0, throttle_pwm)
                throttle_phase = 'forward'
                throttle_events.append({
                    'phase': throttle_phase,
                    'pwm_off': throttle_pwm,
                    'duration_s': None,
                    'ends_at_next_update': True,
                })
                reverse_ready = False

            elif Accel == 0:
                pwm.set_pwm(THROTTLE_CHANNEL, 0, THROTTLE_STOPPED_PWM)
                throttle_pwm = THROTTLE_STOPPED_PWM
                throttle_phase = 'neutral'
                throttle_events.append({
                    'phase': throttle_phase,
                    'pwm_off': throttle_pwm,
                    'duration_s': 0.01,
                })
                time.sleep(0.01)

            else:
                throttle_phase = 'reverse'
                throttle_pwm = int(THROTTLE_STOPPED_PWM + (THROTTLE_STOPPED_PWM - THROTTLE_REVERSE_PWM) * Accel / 100)

                if not reverse_ready:
                    # 初回はブレーキ→ニュートラル。
                    pwm.set_pwm(THROTTLE_CHANNEL, 0, throttle_pwm)
                    throttle_events.append({
                        'phase': 'brake',
                        'pwm_off': throttle_pwm,
                        'duration_s': BRAKE_SECONDS,
                    })
                    time.sleep(BRAKE_SECONDS)
                    pwm.set_pwm(THROTTLE_CHANNEL, 0, THROTTLE_STOPPED_PWM)
                    throttle_events.append({
                        'phase': 'neutral_before_reverse',
                        'pwm_off': THROTTLE_STOPPED_PWM,
                        'duration_s': REVERSE_NEUTRAL_SECONDS,
                    })
                    time.sleep(REVERSE_NEUTRAL_SECONDS)
                    reverse_ready = True

                # 短く後退して、再測距前にニュートラルへ。
                pwm.set_pwm(THROTTLE_CHANNEL, 0, throttle_pwm)
                throttle_events.append({
                    'phase': 'reverse_pulse',
                    'pwm_off': throttle_pwm,
                    'duration_s': REVERSE_PULSE_SECONDS,
                })
                time.sleep(REVERSE_PULSE_SECONDS)
                pwm.set_pwm(THROTTLE_CHANNEL, 0, THROTTLE_STOPPED_PWM)
                throttle_events.append({
                    'phase': 'neutral_after_reverse',
                    'pwm_off': THROTTLE_STOPPED_PWM,
                    'duration_s': REVERSE_NEUTRAL_SECONDS,
                })
                time.sleep(REVERSE_NEUTRAL_SECONDS)

            record = [
                'Fr:{:.0f}  FrLh:{:.0f}  FrRh:{:.0f}  RrLh:{:.0f}  RrRh:{:.0f}'
                .format(Fr, FrLh, FrRh, RrLh, RrRh),
                '{}  ハンドル={:+.1f}  アクセル={:.1f}  周期={:.3f}s  P={:+.1f} D={:+.1f} C={:.2f}'
                .format(comment, Handle, Accel, dt,
                        controller.debug['p'], controller.debug['d'], controller.debug['corner']),
                'Gap={}  PP={:+.1f}  目標距離={:.0f}cm'
                .format(controller.debug['gap'], controller.debug['pp'],
                        controller.debug['lookahead_cm']),
                '狭路={} 側方連続={} 安全判定={}'.format(
                    controller.debug['narrow'], controller.debug['side_close_count'],
                    controller.debug['safety_reason']),
            ]

            if log_file is not None:
                for line in record:
                    log_file.write(line + '\n')

                trace_record = {
                    'type': 'cycle',
                    'index': iteration + 1,
                    'timestamp': datetime.datetime.now().isoformat(timespec='milliseconds'),
                    'elapsed_s': round(decision_elapsed, 6),
                    'dt_s': round(dt, 6),
                    'sensor_cm': {
                        'Fr': Fr,
                        'FrLh': FrLh,
                        'FrRh': FrRh,
                        'RrLh': RrLh,
                        'RrRh': RrRh,
                    },
                    'decision': {
                        'comment': comment,
                        'handle': Handle,
                        'accel': Accel,
                        'controller_debug': dict(controller.debug),
                    },
                    'actuators': {
                        'steering': {
                            'channel': STEERING_CHANNEL,
                            'pwm_off': steer_pwm,
                        },
                        'throttle': {
                            'channel': THROTTLE_CHANNEL,
                            'pwm_off_after_cycle': (
                                THROTTLE_STOPPED_PWM if Accel < 0 else throttle_pwm
                            ),
                            'requested_pwm': throttle_pwm,
                            'phase': throttle_phase,
                            'events': throttle_events,
                        },
                        'reverse_ready_after_cycle': reverse_ready,
                    },
                }
                trace_file.write(json.dumps(
                    trace_record, ensure_ascii=False, separators=(',', ':')
                ) + '\n')
                if (iteration + 1) % 10 == 0:
                    log_file.flush()
                    trace_file.flush()

            iteration += 1
            log_records = iteration

            if now - last_log >= LOG_INTERVAL_SECONDS or comment != last_comment:
                for line in record:
                    print(line)
                last_log = now
                last_comment = comment

    except KeyboardInterrupt:
        stop_reason = 'Ctrl+Cによる停止'
        print('停止します')
    except BaseException as exc:
        stop_reason = '異常終了: {}: {}'.format(type(exc).__name__, exc)
        raise

    finally:
        try:
            pwm.set_pwm(THROTTLE_CHANNEL, 0, THROTTLE_STOPPED_PWM)
        finally:
            try:
                pwm.set_pwm(STEERING_CHANNEL, 0, STEERING_CENTER_PWM)
            finally:
                try:
                    GPIO.cleanup()
                finally:
                    if log_file is not None:
                        elapsed = (time.monotonic() - run_started) if run_started is not None else 0.0
                        try:
                            log_file.write('\n走行停止: {}\n'.format(stop_reason))
                            log_file.write('走行時間: {:.2f}s\n'.format(elapsed))
                            log_file.write('全制御周期数: {}\n'.format(log_records))
                            log_file.flush()
                        finally:
                            log_file.close()
                        if trace_file is not None:
                            end_record = {
                                'type': 'stop',
                                'timestamp': datetime.datetime.now().isoformat(timespec='milliseconds'),
                                'elapsed_s': round(elapsed, 6),
                                'reason': stop_reason,
                                'cycles': log_records,
                                'final_outputs': {
                                    'throttle_pwm': THROTTLE_STOPPED_PWM,
                                    'steering_pwm': STEERING_CENTER_PWM,
                                },
                            }
                            try:
                                trace_file.write(json.dumps(end_record, ensure_ascii=False) + '\n')
                                trace_file.flush()
                            finally:
                                trace_file.close()
                        print('走行ログを保存しました: {}'.format(log_path))
                        if trace_path is not None:
                            print('詳細トレースを保存しました: {}'.format(trace_path))
                        print('全制御周期数: {}'.format(log_records))


if __name__ == '__main__':
    main()
