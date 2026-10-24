#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v23: 距離優先の前進判断と、近接時の短い後退復帰。

通常アクセル指令30。
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


def _clip(value, low, high):
    return max(low, min(high, value))


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
        self._last_angle = 0.0
        self._edge_guard = False
        self._edge_confirm = 0
        self._edge_elapsed = 0.0
        self._edge_recent = 0.0
        self._turn_right = False
        self._follow_left = False
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
            self._filtered = None
            self._handle = 0.0
            self._turn = False
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

        if self._filtered is None:
            self._filtered = raw[:]

        alpha = 1 - math.exp(-step / 0.06)
        self._filtered = [
            a + alpha * (b - a)
            for a, b in zip(self._filtered, raw)
        ]
        f, fl, fr, sl, sr = self._filtered
        front = min(f, raw[0])
        p = self.params

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
        elif sr < 200:
            demand = p["distance_gain"] * (p["target"] - (sr + 9))
        else:
            demand = -60.0

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
            and abs(left_angle) < 20
            and front > 160
            and sl > 80.0
            and sr < 95
            and ray > sr + 15
        ):
            self._follow_left = True

        if self._follow_left:
            left_clearance = (
                (9 + sl - 14 * left_slope)
                / math.sqrt(1 + left_slope * left_slope)
            )
            demand = (
                p["angle_gain"] * left_angle
                + p["distance_gain"] * (left_clearance - 69.0)
            )

        # 右壁の先端への早すぎる切り込みを抑える。
        self._edge_recent = max(0.0, self._edge_recent - step)
        edge_candidate = (
            not self._turn
            and not self._follow_left
            and front > 100
            and 15 < sr < 50.0
            and fr > max(150.0, 4 * sr)
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

        if (front < p["turn_start"] or early_left) and not self._turn:
            self._turn = True
            self._turn_right = (
                fr > 100
                and (
                    (self._edge_recent > 0 and fr > fl + 30)
                    or (sr > 150 and self._handle < 0)
                )
            )

        if self._turn and front > p["turn_release"]:
            self._turn = False

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

        # 狭路では操舵による減速を掛けず、指令30を維持。
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
    """近接時に短く後退し、余裕を確認して前進へ復帰する。"""

    MAX_RECOVERY_PULSES = 999

    def __init__(self):
        super().__init__()
        self._recovery_pulses = 0
        self._recovery_wait = False
        self._clear_scans = 0
        self._forward_seconds = 0.0

    def update(self, Fr, FrLh, FrRh, RrLh, RrRh, dt):
        try:
            raw = tuple(
                float(v) for v in (Fr, FrLh, FrRh, RrLh, RrRh)
            )
            elapsed = float(dt)
            valid = (
                all(math.isfinite(v) and 0 < v <= 500 for v in raw)
                and math.isfinite(elapsed)
                and 0 < elapsed <= 0.5
            )
        except (ValueError, TypeError, OverflowError):
            valid = False

        # 測距・周期異常では後退しない。
        if not valid:
            self._forward_seconds = 0.0
            self._clear_scans = 0
            return super().update(
                Fr, FrLh, FrRh, RrLh, RrRh, dt
            )

        near = (
            raw[0] < 16
            or min(raw[1:3]) < 12
            or min(raw[3:]) < 8
        )
        released = (
            raw[0] >= 24
            and min(raw[1:3]) >= 18
            and min(raw[3:]) >= 12
        )

        # mainでの後退・ニュートラル終了後、一度停止して確認。
        if self._recovery_wait:
            self._recovery_wait = False
            self._clear_scans = 0
            return self._recovery_stop(
                "後退後：停止して再測距"
            )

        if self._recovery_pulses:
            self._clear_scans = (
                self._clear_scans + 1 if released else 0
            )

            if self._clear_scans == 1:
                return self._recovery_stop(
                    "復帰余裕を再確認"
                )

            if self._clear_scans < 2:
                return self._reverse_step(raw)

        if near:
            return self._reverse_step(raw)

        accel, handle, comment = super().update(*raw, elapsed)

        # 前進候補がなく停止した場合にも後退復帰を試す。
        if (
            accel == 0
            and self.debug.get("safety_reason")
            == "車幅余裕のある進路候補なし：停止"
        ):
            return self._reverse_step(raw)

        if accel > 0:
            self._forward_seconds += elapsed

            # 前進が1秒続いたら後退回数をリセット。
            if self._forward_seconds >= 1.0:
                self._recovery_pulses = 0
                self._clear_scans = 0
        else:
            self._forward_seconds = 0.0

        return accel, handle, comment

    def _recovery_stop(self, reason):
        self._handle = 0.0
        self.gap_planner.reset()
        self.debug.update(
            safety_reason=reason,
            pp_applied=False,
        )
        return 0.0, 0.0, reason

    def _reverse_step(self, raw):
        self._forward_seconds = 0.0
        self._clear_scans = 0

        if self._recovery_pulses >= self.MAX_RECOVERY_PULSES:
            return self._recovery_stop(
                "後退5回で改善なし：手動確認"
            )

        # 側面が近い場合は直進後退。
        handle = 0.0

        # 側方に余裕がある場合だけ小さく舵を付け、
        # 後退で車首を近い前方障害物の反対側へ向ける。
        if min(raw[3:]) >= 18:
            if raw[1] + 5 < raw[2]:
                handle = 20.0
            elif raw[2] + 5 < raw[1]:
                handle = -20.0

        # 前進用のフィルタ・旋回・壁端履歴を破棄。
        _ForwardDriveController.__init__(self)

        self._recovery_pulses += 1
        self._recovery_wait = True

        reason = "近接復帰：短く後退 {}/{}".format(
            self._recovery_pulses,
            self.MAX_RECOVERY_PULSES,
        )
        self.debug.update(
            safety_reason=reason,
            pp_applied=False,
        )

        # mainの既存ESC後退手順を使用。
        return -50.0, handle, reason


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
            start_record = {
                'type': 'start',
                'schema_version': 1,
                'run_id': run_stamp,
                'started_at': started_at.isoformat(timespec='milliseconds'),
                'program_path': os.path.abspath(__file__),
                'program_sha256': program_sha256,
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
