#!/usr/bin/env python3
"""tak 로버 웹 UI + 자율 주행 제어 서버. Pi 호스트에서 실행한다 (표준 라이브러리 + pyserial).

  python3 rover_ui.py [--dry] [--no-pin] [--port 8000]

- 인식 결과: Pi 의 rover 코드 대시보드(http://127.0.0.1:8080/state)에서 선·표지판을,
  보조 인식기(~/rover_animals/out/extra.json)에서 동물·손바닥을 읽는다.
- 모터: 아두이노 우노(L298 펌웨어, /dev/ttyACM0)로 "M 왼 오" / "W a b c d" 를 보낸다.
  펌웨어는 명령이 0.3초 끊기면 스스로 멈춘다. --dry 면 계산만 하고 보내지 않는다.
- 모드: line(자율 주행: 선 추종 + 표지판 규칙), hand(손바닥 추적), manual(수동), replay(기록 재생).
- 표지판 규칙: 진입 금지·정지(stop, nenter1, nenter2)는 가까워지면 멈추고, 그 밖의 표지판은
  가까워지면 그쪽으로 가지 않고 제자리에서 돌아 다른 선을 찾아 이어간다.
- 접속: 같은 네트워크의 다른 컴퓨터는 접속 번호(pin.txt)를 한 번 넣어야 한다. Pi 자신(127.0.0.1)은 묻지 않는다.
"""
import collections
import json
import math
import os
import re
import secrets
import shutil
import signal
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOME = os.path.expanduser("~")
UI_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_URL = "http://127.0.0.1:8080/state"
FRAME_FILE = os.path.join(HOME, "robot_ws/.runtime/latest.jpg")
EXTRA_DIR = os.path.join(HOME, "rover_animals/out")
EXTRA_FILE = os.path.join(EXTRA_DIR, "extra.json")
MODE_FILE = os.path.join(EXTRA_DIR, "mode.txt")
RUN_DIR = os.path.join(HOME, "rover_runs")
AUTO_DIR = os.path.join(RUN_DIR, "auto")      # 자율 주행 중 남기는 화면·상태 기록
ROAD_CONFIG = os.path.join(HOME, "robot_ws/.runtime/venue_yolo.yaml")
PIN_FILE = os.path.join(UI_DIR, "pin.txt")
PARAM_FILE = os.path.join(UI_DIR, "params.json")
PAGE_FILE = os.path.join(UI_DIR, "index.html")
SERIAL = "/dev/ttyACM0"
WIDTH, HEIGHT = 640, 480
PERIOD = 0.066
WHEEL_ORDER = ["RL", "FL", "FR", "RR"]   # 모터 0..3 (2026-11-25 실측)
# 표지판 모델의 다섯 종류 (2026-11-25 팀이 찍어 둔 학습 사진으로 확인):
#   stop = 빨간 팔각 STOP, detour = 노란 삼각 주의, nenter1 = 빨간 팔각 손바닥, nenter2 = 빨간 원 흰 가로줄,
#   hdetour = 파란 원 유턴 화살표.  "유턴 금지" 종류는 모델에 없다.
STOP_SIGNS = {"stop"}
UTURN_SIGN = "hdetour"
SIGN_KO = {"stop": "STOP", "nenter1": "진입 금지", "nenter2": "진입 금지", "detour": "주의", "hdetour": "유턴"}
COMMAND = re.compile(r"^(S|M -?\d{1,3} -?\d{1,3}|W -?\d{1,3} -?\d{1,3} -?\d{1,3} -?\d{1,3})$")

DEFAULTS = {
    "base": 150, "kp": 1.6, "ki": 0.6, "tmax": 0.4, "slow": 0.45, "coast": 0.5,
    "pivot": 220, "search": 3.5, "min_conf": 0.5, "turn": "right",
    "corner_mode": "wall", "turn90": 1.2, "bump_push": 1.0, "bump_max": 2.5, "wall_fill": 0.85,
    "steer": "strafe", "strafe_gain": 150, "turn_gain": 150, "shift_at": 0.35, "shift_pwm": 255,
    "sign_rules": True, "stop_action": "stop", "stop_conf": 0.5, "stop_size": 0.05,
    "sign_conf": 0.75, "sign_near": 0.25,
    "detour_min": 1.2, "detour_max": 16.0,
    "animal_action": "detour", "animal_size": 0.06, "animal_trust": "once", "uturn_line": "ignore", "uturn_time": 2.4,
    "hand_base": 130, "hand_pivot": 200, "hand_stop_area": 0.075, "hand_center": 0.35,
    "manual_speed": 150, "manual_turn": 220, "manual_strafe": 200,
    "motors": True,
}
LIMITS = {
    "base": (0, 255), "kp": (0, 5), "ki": (0, 3), "tmax": (0.1, 1.5), "slow": (0, 0.9), "coast": (0, 2),
    "pivot": (0, 255), "search": (0.5, 8), "min_conf": (0.2, 0.95),
    "turn90": (0.3, 3.0), "bump_push": (0, 2.5), "bump_max": (1, 6), "wall_fill": (0.6, 0.98),
    "stop_conf": (0.3, 0.99), "stop_size": (0.0, 0.9),
    "strafe_gain": (0, 600), "turn_gain": (0, 600), "shift_at": (0.1, 1.5), "shift_pwm": (120, 255),
    "sign_conf": (0.3, 0.99), "sign_near": (0.05, 0.9), "detour_min": (0.3, 4), "detour_max": (2, 40),
    "animal_size": (0.05, 0.6), "uturn_time": (0.6, 6.0),
    "hand_base": (0, 255), "hand_pivot": (0, 255), "hand_stop_area": (0.01, 0.5), "hand_center": (0.1, 0.9),
    "manual_speed": (0, 255), "manual_turn": (0, 255), "manual_strafe": (0, 255),
}
CHOICES = {"turn": ("left", "right"), "stop_action": ("stop", "ignore"), "corner_mode": ("wall", "search"),
           "steer": ("strafe", "turn"),
           "animal_action": ("detour", "wait", "ignore"), "animal_trust": ("once", "twice"), "uturn_line": ("ignore", "turn")}
FLAGS = ("sign_rules", "motors")
MOVES = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 0, 1), "d": (0, 0, -1), "q": (0, 1, 0), "e": (0, -1, 0)}


def clamp(value, low, high):
    return max(low, min(high, value))


def mecanum(vx, vy, w):
    """vx 전진, vy 왼쪽, w 반시계 (각 -1..1) -> 모터 0..3 순서의 비율."""
    mix = {"FL": vx - vy - w, "FR": vx + vy + w, "RL": vx + vy - w, "RR": vx - vy + w}
    top = max(1.0, max(abs(v) for v in mix.values()))
    return [mix[n] / top for n in WHEEL_ORDER]


def pivot_command(direction, pwm):
    """direction +1 = 시계 방향(오른쪽으로 회전)."""
    return f"M {direction * pwm} {-direction * pwm}"


# ---------- 두꺼운 중앙선 추적 ----------
# 인식 스택(road_paths.py)은 선의 굵기를 보지 않고 화면 가운데에 가장 가까운 노란 띠를 중앙선으로 고른다.
# 그래서 얇은 테두리 선을 따라가거나, 두 선이 비슷한 거리에 있으면 선을 놓친 것으로 처리한다.
# 여기서는 굵기로 고른다. 숫자는 반 크기 화면(320x240) 기준이고 2026-11-25 대회장 사진으로 맞췄다:
# 중앙선의 가로 폭은 줄 y 에서 약 0.143 * (y - 7) 픽셀, 테두리 선은 그 1/5 쯤이다.
LINE_HORIZON = 7.0        # 바닥의 선 폭이 0 이 되는 줄 (지평선)
LINE_WIDTH_K = 0.143      # 중앙선의 가로 폭 = LINE_WIDTH_K * (y - LINE_HORIZON)
LINE_FOCAL = 314.0        # 초점 거리 (수평 화각 54도)
LINE_PITCH = math.atan((120.0 - LINE_HORIZON) / LINE_FOCAL)     # 카메라가 아래로 숙인 각도 (약 20도)


def read_hsv():
    """인식 스택 설정에서 노란색 HSV 범위를 읽는다 (같은 색을 보게). 못 읽으면 대회장 값."""
    low, high = (15, 0, 170), (45, 255, 255)
    try:
        with open(ROAD_CONFIG) as handle:
            text = handle.read()
        found = {name: tuple(int(v) for v in re.search(name + r"\s*:\s*\[([^\]]+)\]", text).group(1).split(","))
                 for name in ("hsv_low", "hsv_high")}
        if len(found["hsv_low"]) == 3 and len(found["hsv_high"]) == 3:
            low, high = found["hsv_low"], found["hsv_high"]
    except (OSError, AttributeError, ValueError):
        pass
    return low, high


def yellow_mask(a, np, low, high, value=None):
    """RGB(int16) 화면에서 OpenCV HSV 범위(H 0..180)에 드는 픽셀. value 는 미리 구한 밝기(채널 최댓값).
    밝기 조건에 드는 픽셀만 골라 색상을 계산한다 (매트 위에서는 화면의 1~2할뿐이라 Pi 에서 몇 배 빠르다)."""
    if value is None:
        value = np.maximum(np.maximum(a[..., 0], a[..., 1]), a[..., 2])
    bright = (value >= low[2]) & (value <= high[2])
    mask = np.zeros(value.shape, dtype=bool)
    if not bright.any():
        return mask
    px = a[bright]
    r, g, b = px[:, 0], px[:, 1], px[:, 2]
    mx = value[bright]
    mn = np.minimum(np.minimum(r, g), b)
    d = (mx - mn).astype(np.float32)
    safe = np.where(d > 0, d, 1.0)
    hue = np.where(mx == r, (g - b) * 30.0 / safe,
                   np.where(mx == g, 60.0 + (b - r) * 30.0 / safe, 120.0 + (r - g) * 30.0 / safe))
    hue = np.where(hue < 0, hue + 180.0, hue)
    hue = np.where(d > 0, hue, 0.0)
    sat = d * 255.0 / np.maximum(mx, 1)
    mask[bright] = ((hue >= low[0] - 0.5) & (hue <= high[0] + 0.5) & (sat >= low[1] - 0.5) & (sat <= high[1] + 0.5))
    return mask


def _stands_out(cum, a, b, wide, both):
    """노란 띠 [a, b) 가 양옆(both) 또는 한쪽보다 뚜렷이 밝은가. 밝은 바닥의 얼룩은 옆도 밝아서 걸러진다.
    cum 은 그 줄 밝기의 누적합(앞에 0 을 붙인 목록): 구간 평균을 numpy 호출 없이 바로 구한다 (Pi 에서 느려서)."""
    k = max(3, int(round(0.2 * wide)))
    inside = (cum[b] - cum[a]) / float(b - a)
    sides = []
    if a - 1 - k >= 0:
        sides.append((cum[a - 1] - cum[a - 1 - k]) / float(k))
    if b + 1 + k <= len(cum) - 1:
        sides.append((cum[b + 1 + k] - cum[b + 1]) / float(k))
    if not sides:
        return False
    dark = [inside - side >= 30 for side in sides]
    return all(dark) if both else any(dark)


def _fit_line(points):
    """(y, x) 점들에 x = x0 + m * (y - y0) 를 맞춘다. y0 는 첫 점(가장 아래)의 y."""
    y0 = points[0][0]
    n = len(points)
    sy = sum(p[0] - y0 for p in points)
    sx = sum(p[1] for p in points)
    syy = sum((p[0] - y0) ** 2 for p in points)
    sxy = sum((p[0] - y0) * p[1] for p in points)
    den = n * syy - sy * sy
    if n < 2 or den == 0:
        return points[0][1], 0.0
    m = (n * sxy - sy * sx) / den
    return (sx - m * sy) / n, m


def _trace_up(rows, start, x, width, w, h, stride, thick):
    """rows[start] 의 띠 (x, width) 에서 위쪽 줄로 이어지는 띠를 따라간다."""
    points = [(rows[start][0], x, width)]
    after_bar = False
    for y, wide, runs in rows[start + 1:]:
        last_y, last_x, _ = points[-1]
        ref = points[max(0, len(points) - 5)]
        slope = (last_x - ref[1]) / (last_y - ref[0]) if last_y != ref[0] else 0.0
        predicted = last_x + clamp(slope, -2.0, 2.0) * (y - last_y)
        if last_y - y > (0.14 * h if after_bar else 4 * stride):
            break
        if thick:
            if any(a <= predicted <= b and b - a > 2.2 * wide for a, b in runs):
                after_bar = True          # 갈림길의 가로 선: 건너뛰고 그 너머에서 이어지는 선을 찾는다
                continue
            pool = [((a + b - 1) / 2.0, b - a) for a, b in runs if 0.55 * wide <= b - a <= 2.2 * wide]
            radius = max(0.9 * wide, 0.045 * w) * (1.6 if after_bar else 1.0)
        else:
            pool = [((a + b - 1) / 2.0, b - a) for a, b in runs if 2 <= b - a <= 0.45 * wide]
            radius = max(0.5 * wide, 0.035 * w)
        near = [c for c in pool if abs(c[0] - predicted) <= radius]
        if not near:
            continue
        cx, cw = min(near, key=lambda c: abs(c[0] - predicted))
        points.append((y, cx, cw))
        after_bar = False
    return points


def _runs_into(track, boxes, horizon, h):
    """선(track: 아래에서 위로 (y, x, 폭) 점들)이 동물 박스에 닿으면 그 박스, 아니면 None.
    선의 점이 박스 아래쪽 절반(발이 바닥에 닿는 곳)을 지나거나, 선이 박스 바로 아래에서 끊겼고 끝부분의 방향대로
    이으면 발 높이에서 박스 안을 지날 때 닿는 것으로 본다. boxes: 이 화면 기준 (x1, y1, x2, y2)."""
    ty, tx = track[-1][0], track[-1][1]
    slope = _fit_line([(p[0], p[1]) for p in track[-min(len(track), 6):]])[1]
    for box in boxes:
        x1, y1, x2, y2 = box
        wide = LINE_WIDTH_K * max(1.0, y2 - horizon)
        pad = 0.25 * (x2 - x1) + wide
        lo, hi = x1 - pad, x2 + pad
        feet = y1 + 0.5 * (y2 - y1)
        if any(feet <= py <= y2 + wide and lo <= px <= hi for py, px, _ in track):
            return box
        if y2 < ty <= y2 + 0.25 * h and lo <= tx + slope * (y2 - ty) <= hi:
            return box
    return None


def find_line(a, np, hsv, hint=None, blocked=()):
    """반 크기 RGB 화면에서 두꺼운 중앙선과 양옆 얇은 테두리 선을 찾는다.
    blocked: 동물 박스들 (이 화면 기준 x1, y1, x2, y2). 그 박스에 닿는 선은 고르지 않는다. 다른 선이 없을 때만
    그 선을 돌려주고 blocked = True, animal = 그 동물의 크기와 위치를 붙인다 (따라갈지는 부르는 쪽이 정한다).

    line (없으면 None): x 가까운 점의 좌우 위치 -1..1, row 그 점의 높이 0..1, offset 로버(카메라) 밑에서 선이 옆으로
    떨어진 거리(카메라 높이 단위, + = 선이 오른쪽), angle 선 방향 - 로버 방향(rad, + = 선이 오른쪽으로 향함),
    span 따라간 길이(화면 높이 비율), straight 그중 아래쪽에서 곧게 이어지는 길이, bend 그 위에서 꺾이는 쪽(-1 왼쪽,
    0 없음, +1 오른쪽), conf 0..1 (곧은 구간의 길이로 매긴다), points 그리기용 점.
    위치와 방향은 곧은 구간만으로 잰다: 코너에서 휘어 나가는 부분까지 넣으면 코너에 닿기 전에 미리 돌게 된다.
    left, right: 테두리 선까지의 옆 거리(같은 단위, 왼쪽은 음수) 또는 None.
    """
    h, w = a.shape[:2]
    scale = h / 240.0
    horizon, focal = LINE_HORIZON * scale, LINE_FOCAL * scale
    stride = max(2, int(round(3 * scale)))
    top, bottom = int(h * 0.30), int(h * 0.955)
    crop = a[top - 2:bottom + 3]
    value = np.maximum(np.maximum(crop[..., 0], crop[..., 1]), crop[..., 2])
    mask = yellow_mask(crop, np, hsv[0], hsv[1], value)
    empty = {"line": None, "left": None, "right": None, "thin": []}
    if float(mask.mean()) > 0.30:
        return empty                          # 화면의 3할 넘게 노랗다: 매트 위가 아니다 (밝은 바닥)
    # 볼 줄들을 한꺼번에 처리한다 (줄마다 numpy 를 부르면 Pi 에서 화면 한 장에 80ms 가 걸렸다)
    ys = list(range(bottom, top, -stride))
    idx = np.asarray(ys) - (top - 2)
    bands = (mask[idx - 1].astype(np.int8) + mask[idx] + mask[idx + 1]) >= 2
    padded = np.zeros((len(ys), w + 2), dtype=np.int8)
    padded[:, 1:-1] = bands
    edges = np.diff(padded, axis=1)
    start_row, start_col = np.nonzero(edges == 1)
    end_col = np.nonzero(edges == -1)[1]
    per_row = [[] for _ in ys]
    for r0, s0, e0 in zip(start_row.tolist(), start_col.tolist(), end_col.tolist()):
        per_row[r0].append([s0, e0])
    cums = np.concatenate((np.zeros((len(ys), 1), dtype=np.int64),
                           np.cumsum(value[idx].astype(np.int64), axis=1)), axis=1).tolist()
    rows = []
    for n, y in enumerate(ys):
        wide = LINE_WIDTH_K * (y - horizon)
        gap = max(1.0, 0.08 * wide)
        runs = []
        for s0, e0 in per_row[n]:
            if runs and s0 - runs[-1][1] <= gap:
                runs[-1][1] = e0              # 테이프 이음매나 반사로 생긴 작은 틈은 잇는다
            else:
                runs.append([s0, e0])
        if len(runs) > 12:
            runs = []                         # 줄 전체가 얼룩덜룩하다: 매트가 아닌 바닥
        cum = cums[n]
        rows.append((y, wide, [(s0, e0) for s0, e0 in runs if e0 - s0 > 2.2 * wide
                               or _stands_out(cum, s0, e0, wide, e0 - s0 >= 0.55 * wide)]))

    # 1) 두꺼운 선: 아래쪽 줄부터 씨앗을 잡아 위로 따라가고, 가장 길게 이어지는 것을 고른다
    cos_p, tan_p = math.cos(LINE_PITCH), math.tan(LINE_PITCH)
    target = hint if hint is not None else w / 2.0
    best, best_score, used, seeded = None, None, set(), 0
    shut, shut_score, shut_box = None, None, None       # 동물이 서 있는 선 중 가장 좋은 것
    for i, (y, wide, runs) in enumerate(rows):
        if seeded >= 14:
            break
        thick = [((s0 + e0 - 1) / 2.0, e0 - s0) for s0, e0 in runs if 0.55 * wide <= e0 - s0 <= 2.2 * wide]
        seeded += 1 if thick else 0
        for x, width in thick:
            if (y, int(round(x))) in used:
                continue
            track = _trace_up(rows, i, x, width, w, h, stride, True)
            used.update((p[0], int(round(p[1]))) for p in track)
            if len(track) < 4:
                continue
            # 아래쪽부터 곧게 이어지는 구간을 찾는다 (새 점이 맞춘 직선에서 벗어나면 거기서부터는 꺾이는 부분)
            pts = [(p[0], p[1]) for p in track]
            k = min(len(pts), 5)
            x0, m = _fit_line(pts[:k])
            while k < len(pts):
                py, px = pts[k]
                if abs(px - (x0 + m * (py - y))) > max(1.5 * scale, 0.15 * LINE_WIDTH_K * (py - horizon)):
                    break
                k += 1
                if k % 3 == 0:
                    x0, m = _fit_line(pts[:k])
            # 위에서 꺾이면, 곧은 구간의 끝부분은 이미 휘기 시작한 곳일 수 있어 아래쪽 7할로만 맞춘다
            x0, m = _fit_line(pts[:max(4, int(k * 0.7))] if k < len(pts) else pts[:k])
            if abs(m) > 2.0:
                continue                      # 거의 가로로 누운 선: 갈림길 가지나 벽 쪽 선
            # 기울어진 선은 가로로 잰 폭이 넓어 보인다. 선에 수직인 굵기로 고쳐서 얇은 선을 걸러낸다
            ratios = sorted(p[2] / math.sqrt(1 + m * m) / (LINE_WIDTH_K * (p[0] - horizon)) for p in track)
            if ratios[len(ratios) // 2] < 0.5:
                continue
            score = len(track) - 0.1 * abs(x0 - target) / scale
            box = _runs_into(track, blocked, horizon, h) if blocked else None
            if box is not None:
                if shut_score is None or score > shut_score:
                    shut, shut_score, shut_box = (track, x0, m, k), score, box
            elif best_score is None or score > best_score:
                best, best_score = (track, x0, m, k), score

    def describe(found):
        track, x0, m, k = found
        y0 = track[0][0]
        span = track[0][0] - track[-1][0] + stride
        straight = track[0][0] - track[k - 1][0] + stride
        bend = 0
        if k < len(track):
            far = track[min(len(track) - 1, k + 3)]
            bend = 1 if far[1] > x0 + m * (far[0] - y0) else -1
        # 바닥의 직선은 화면에서도 직선이다: 지평선에서 만나는 점이 방향을, 기울기가 옆 거리를 준다
        tan_angle = (x0 - m * (y0 - horizon) - w / 2.0) * cos_p / focal
        return {
            "x": round(clamp(x0 / (w / 2.0) - 1.0, -1.0, 1.0), 3), "row": round(y0 / (h - 1.0), 3),
            "offset": round(m / cos_p + tan_p * tan_angle, 3), "angle": round(math.atan(tan_angle), 3),
            "span": round(span / float(h), 3), "n": len(track),
            "straight": round(straight / float(h), 3), "bend": bend,
            # 확신도: 곧은 구간이 화면 높이의 16% 면 0.5. 위에서 꺾이는 선은 22% 가 되어야 0.5 로 친다
            # (코너 바로 앞에서는 보이는 선 전체가 휘어 있어 위치와 방향을 잴 수 없다. 그때는 따라가지 않는다)
            "conf": round(min(1.0, straight / ((0.44 if bend else 0.32) * h)) * min(1.0, k * stride / float(straight)), 3),
            "points": [[round(p[1] / (w - 1.0), 3), round(p[0] / (h - 1.0), 3)]
                       for p in track[::max(1, len(track) // 9)]],
            # 화면에서의 직선 (가로·세로 모두 0..1): x = fit[0] + fit[2] * (y - fit[1])
            "fit": [round(x0 / (w - 1.0), 4), round(y0 / (h - 1.0), 4), round(m * (h - 1.0) / (w - 1.0), 4)],
            "blocked": False,
        }, tan_angle

    line, tan_angle, chosen = None, None, None
    if best is not None:
        (line, tan_angle), chosen = describe(best), best
    if shut is not None:
        if line is None or line["conf"] < 0.5:
            # 동물이 없는 다른 선이 없다(또는 따라가기엔 너무 짧다): 동물이 서 있는 선임을 알리고 돌려준다
            (line, tan_angle), chosen = describe(shut), shut
            bx1, by1, bx2, by2 = shut_box
            line["blocked"] = True
            line["animal"] = {"center": round((bx1 + bx2) / float(w) - 1.0, 3), "tall": round((by2 - by1) / float(h), 3),
                              "foot": round(by2 / float(h), 3)}
        else:
            line["avoided"] = True            # 동물이 서 있는 선 대신 고른 선
    if chosen is not None:
        track, x0, m, k = chosen
        y0 = track[0][0]

    # 2) 얇은 테두리 선: 화면 아래쪽에서, 두꺼운 선의 양옆으로 가장 가까운 것
    sides, thin = {"left": None, "right": None}, []
    for name, sign in (("left", -1), ("right", 1)):
        tried = 0
        for i, (y, wide, runs) in enumerate(rows):
            if y < h * 0.45 or tried >= 10:
                break
            middle = (x0 + m * (y - y0)) if chosen is not None else w / 2.0
            pool = [((s0 + e0 - 1) / 2.0, e0 - s0) for s0, e0 in runs
                    if 2 <= e0 - s0 <= 0.45 * wide and sign * ((s0 + e0 - 1) / 2.0 - middle) > 0.5 * wide]
            if not pool:
                continue
            tried += 1
            x, width = min(pool, key=lambda c: abs(c[0] - middle))
            track = _trace_up(rows, i, x, width, w, h, stride, False)
            if len(track) >= 5 and track[0][0] - track[-1][0] >= 0.06 * h:
                bx, bm = _fit_line([(p[0], p[1]) for p in track])
                tan_b = tan_angle if tan_angle is not None else (bx - bm * (track[0][0] - horizon) - w / 2.0) * cos_p / focal
                sides[name] = round(bm / cos_p + tan_p * tan_b, 3)
                thin.append([[round(p[1] / (w - 1.0), 3), round(p[0] / (h - 1.0), 3)]
                             for p in track[::max(1, len(track) // 6)]])
                break
    return {"line": line, "left": sides["left"], "right": sides["right"], "thin": thin}


def strafe_command(direction, pwm):
    """제자리 게걸음. direction +1 = 오른쪽."""
    wheel = {"FL": direction * pwm, "FR": -direction * pwm, "RL": -direction * pwm, "RR": direction * pwm}
    return "W " + " ".join(str(int(round(wheel[name]))) for name in WHEEL_ORDER)


class Uno:
    """아두이노 직렬 연결. 끊기면 뒤에서 다시 붙는다 (제어 루프를 막지 않는다)."""

    def __init__(self, port):
        self.port = port
        self.s = None
        self.ok = False
        self.error = "연결하는 중"
        self.busy = False
        self.last_try = 0.0
        self.lock = threading.Lock()

    def _connect(self):
        try:
            import serial
            s = serial.Serial(self.port, 115200, timeout=0.2, write_timeout=0.3, exclusive=True)
            deadline = time.time() + 4.0
            seen = b""
            while time.time() < deadline and b"L298 READY" not in seen:
                seen += s.read(64)
            if b"L298 READY" not in seen:
                s.close()
                raise IOError("아두이노가 L298 펌웨어로 응답하지 않습니다")
            with self.lock:
                self.s, self.ok, self.error = s, True, ""
            print("uno connected", flush=True)
        except Exception as problem:
            self.ok, self.error = False, str(problem)[:140]
        finally:
            self.busy = False

    def send(self, text):
        with self.lock:
            s = self.s
            if s is not None:
                try:
                    s.write((text + "\n").encode("ascii"))
                    if s.in_waiting > 512:
                        s.reset_input_buffer()
                    return True
                except Exception as problem:
                    try:
                        s.close()
                    except Exception:
                        pass
                    self.s, self.ok, self.error = None, False, str(problem)[:140]
                    print("uno lost:", self.error, flush=True)
        if not self.busy and time.time() - self.last_try > 3.0:
            self.busy, self.last_try = True, time.time()
            threading.Thread(target=self._connect, daemon=True).start()
        return False


class Front:
    """카메라 화면에서 앞·왼쪽·오른쪽 벽까지의 여유를 잰다 (반 크기로 풀어 numpy 로 계산).

    매트(어둡거나 푸른빛)가 아닌 것이 세로 띠를 가로로 60% 넘게 채우는 줄이 위로 길게 이어지면 벽으로 본다.
    rows: 띠별 벽 밑단의 화면 높이 비율 (1.0 = 화면 맨 아래 = 바로 앞, None = 벽 없음)
    fill: 가운데 띠의 아래쪽 55% 가 벽으로 가려진 비율 (직선 구간은 가운데 선 때문에 0.3 안팎, 벽 코앞이면 1.0)
    기준값은 2026-11-25 대회장 사진으로 맞췄다 (매트 0.00~0.07, 가까운 벽 0.91~1.00).
    """
    STRIPS = {"L": (0.04, 0.34), "C": (0.35, 0.65), "R": (0.66, 0.96)}

    def __init__(self):
        self.at = 0.0
        self.fill = 0.0
        self.rows = {"L": None, "C": None, "R": None}
        self.error = "아직 화면을 분석하지 않았습니다"
        self.fast = False             # 자율 주행 중에는 매 화면, 아니면 가끔만 계산한다
        self.pixels = None            # 마지막으로 푼 화면 (반 크기, 박스 안 색을 볼 때 쓴다)
        self.hsv = read_hsv()         # 노란 선의 색 범위 (인식 스택 설정과 같게)
        self.line = None              # 두꺼운 중앙선 (find_line 의 line)
        self.left = None              # 왼쪽·오른쪽 얇은 테두리 선까지의 옆 거리
        self.right = None
        self.thin = []
        self.line_ms = 0.0
        self.blocked = []             # 동물 박스들 (640×480 기준). 여기에 닿는 선은 고르지 않는다 (Rover 가 채운다)

    def colors(self, box):
        """640×480 기준 [x1, y1, x2, y2] 안에서 파란색, 빨간색 픽셀이 차지하는 비율."""
        a = self.pixels
        if a is None:
            return None
        h, w = a.shape[:2]
        x1, y1, x2, y2 = box
        crop = a[max(0, int(y1 * h / HEIGHT)):max(1, int(y2 * h / HEIGHT)),
                 max(0, int(x1 * w / WIDTH)):max(1, int(x2 * w / WIDTH))]
        if crop.size == 0:
            return None
        r, g, b = crop[..., 0], crop[..., 1], crop[..., 2]
        blue = float(((b - r >= 40) & (b >= 90)).mean())
        red = float(((r - b >= 50) & (r - g >= 40) & (r >= 100)).mean())
        return blue, red

    def analyse(self, a, np):
        h, w = a.shape[:2]
        r, g, b = a[..., 0], a[..., 1], a[..., 2]
        v = a.max(axis=2)
        non = (((r - b) >= 20) & (r >= 70)) | ((v >= 100) & ((r - b) > -15))
        # 노란 테이프(가운데 선·테두리 선)는 벽이 아니다: 굽은 가운데 선이 옆 띠를 가로질러도 벽으로 세지 않는다
        non &= ~(((r - b) >= 60) & (v >= 160) & (g >= 110))
        tall = max(6, h // 8)
        rows = {}
        for name, (x0, x1) in self.STRIPS.items():
            blocked = non[:, int(x0 * w):int(x1 * w)].mean(axis=1) >= 0.6
            total = np.concatenate(([0], np.cumsum(blocked)))
            found = None
            for y in range(h - 1, int(0.25 * h) + tall, -1):
                if blocked[y] and total[y + 1] - total[y + 1 - tall] >= 0.8 * tall:
                    found = round(y / (h - 1), 3)
                    break
            rows[name] = found
        x0, x1 = self.STRIPS["C"]
        fill = float(non[int(0.45 * h):, int(x0 * w):int(x1 * w)].mean())
        return rows, round(fill, 3)

    def run(self):
        try:
            import numpy as np
            from PIL import Image
        except Exception as problem:
            self.error = f"화면 분석을 쓸 수 없습니다: {problem}"[:120]
            return
        last = None
        while True:
            started = time.time()
            try:
                stamp = os.stat(FRAME_FILE).st_mtime_ns
                if stamp != last:
                    last = stamp
                    with Image.open(FRAME_FILE) as image:
                        image.draft("RGB", (320, 240))
                        pixels = np.asarray(image.convert("RGB"), dtype=np.int16)
                    rows, fill = self.analyse(pixels, np)
                    began = time.time()
                    try:
                        hint = (self.line["x"] + 1.0) / 2.0 * pixels.shape[1] if self.line else None
                        ph, pw = pixels.shape[:2]
                        boxes = [(b[0] * pw / WIDTH, b[1] * ph / HEIGHT, b[2] * pw / WIDTH, b[3] * ph / HEIGHT)
                                 for b in self.blocked]
                        found, problem = find_line(pixels, np, self.hsv, hint, boxes), ""
                    except Exception as issue:      # 선 추적이 실패해도 벽 판단과 나머지는 계속 돈다
                        found = {"line": None, "left": None, "right": None, "thin": []}
                        problem = f"선 추적 오류: {type(issue).__name__}"
                    self.line_ms = round((time.time() - began) * 1000.0, 1)
                    self.pixels = pixels
                    self.line, self.left, self.right, self.thin = found["line"], found["left"], found["right"], found["thin"]
                    self.rows, self.fill, self.at, self.error = rows, fill, time.time(), problem
            except Exception as problem:
                self.error = f"화면 분석 오류: {type(problem).__name__}"
            time.sleep(max(0.01, (0.03 if self.fast else 0.3) - (time.time() - started)))


class Rover:
    def __init__(self, dry=False):
        self.lock = threading.RLock()
        self.front = Front()
        self.params = dict(DEFAULTS)
        self.load_params()
        self.params["motors"] = not dry
        self.mode = "idle"            # idle | line | hand | manual | replay
        self.active = False
        self.view = "line"
        self.state = "대기"
        self.reason = ""
        self.command = "S"
        self.events = collections.deque(maxlen=40)
        self.percept = {}
        self.percept_at = 0.0
        self.percept_error = "아직 인식 결과가 없습니다"
        self.frame_seq = None
        self.frame_fresh_at = 0.0
        self.extra = {}
        self.extra_stamp = None
        self.manual_cmd = "S"
        self.manual_until = 0.0
        self.replay_events = []
        self.replay_index = 0
        self.replay_start = 0.0
        self.replay_cmd = "S"
        self.runs = []
        self.runs_at = 0.0
        self.uno = Uno(SERIAL)
        self.reset_line()
        self.reset_hand()
        self.write_mode()

    def note(self, text):
        self.events.append((time.strftime("%H:%M:%S"), text))
        print(text, flush=True)

    # ---------- 설정 ----------
    def apply(self, key, value):
        if key in FLAGS:
            self.params[key] = bool(value)
        elif key in CHOICES:
            if value not in CHOICES[key]:
                return "고를 수 없는 값입니다"
            self.params[key] = value
        elif key in LIMITS:
            try:
                low, high = LIMITS[key]
                number = clamp(float(value), low, high)
            except (TypeError, ValueError):
                return "숫자가 아닙니다"
            self.params[key] = int(round(number)) if isinstance(DEFAULTS[key], int) else round(number, 4)
        else:
            return "모르는 설정입니다"
        return ""

    def load_params(self):
        try:
            with open(PARAM_FILE) as handle:
                saved = json.load(handle)
            for key, value in saved.items():
                if key != "motors":
                    self.apply(key, value)
        except (OSError, ValueError, AttributeError):
            pass

    def save_params(self):
        try:
            keep = {k: v for k, v in self.params.items() if k != "motors"}
            with open(PARAM_FILE + ".tmp", "w") as handle:
                json.dump(keep, handle, indent=1)
            os.replace(PARAM_FILE + ".tmp", PARAM_FILE)
        except OSError:
            pass

    # ---------- 입력 ----------
    def poll_percept(self):
        while True:
            started = time.time()
            try:
                with urllib.request.urlopen(STATE_URL, timeout=0.5) as reply:
                    data = json.load(reply)
                seq = (data.get("vision_timing") or {}).get("frame_sequence")
                with self.lock:
                    if seq is not None and seq != self.frame_seq:
                        self.frame_seq = seq
                        self.frame_fresh_at = started
                    self.percept, self.percept_at, self.percept_error = data, started, ""
            except Exception as problem:
                with self.lock:
                    self.percept_error = f"선·표지판 인식기가 응답하지 않습니다 ({type(problem).__name__})"
            time.sleep(max(0.0, PERIOD - (time.time() - started)))

    def read_extra(self):
        try:
            stamp = os.stat(EXTRA_FILE).st_mtime_ns
            if stamp != self.extra_stamp:
                with open(EXTRA_FILE) as handle:
                    self.extra = json.load(handle)
                self.extra_stamp = stamp
        except (OSError, ValueError):
            pass

    def write_mode(self):
        """보조 인식기에 할 일을 알린다: 손 탭이면 손바닥, 아니면 동물 (자율 주행 중에는 쉬지 않고)."""
        if self.view == "hand" or self.mode == "hand":
            wanted = "hand"
        elif self.active and self.mode == "line" and self.params["animal_action"] != "ignore":
            wanted = "animals fast"
        else:
            wanted = "animals"
        if wanted == getattr(self, "mode_written", None):
            return
        try:
            os.makedirs(EXTRA_DIR, exist_ok=True)
            with open(MODE_FILE + ".tmp", "w") as handle:
                handle.write(wanted + "\n")
            os.replace(MODE_FILE + ".tmp", MODE_FILE)
            self.mode_written = wanted
        except OSError:
            pass

    def list_runs(self, now):
        if now - self.runs_at < 5.0:
            return self.runs
        self.runs_at = now
        found = []
        try:
            for name in os.listdir(RUN_DIR):
                if not name.endswith(".jsonl"):
                    continue
                path = os.path.join(RUN_DIR, name)
                with open(path) as handle:
                    lines = [line for line in handle if line.strip()]
                if lines:
                    found.append((os.stat(path).st_mtime, name, round(json.loads(lines[-1])["t"], 1)))
        except (OSError, ValueError, KeyError, TypeError):
            pass
        self.runs = [[name, seconds] for _, name, seconds in sorted(found, reverse=True)[:12]]
        return self.runs

    # ---------- 표지판 ----------
    def sign_action(self, label):
        """표지판마다 할 일: stop(보이면 주행 종료), detour(가까우면 그 길로 가지 않고 다른 선), turn(180도), ignore."""
        if label in STOP_SIGNS:
            return self.params["stop_action"]
        if label == UTURN_SIGN:
            return self.params["uturn_line"]
        return "detour"

    def signs_now(self, now):
        """인식기가 준 표지판([x, y, 폭, 높이])을 [x1, y1, x2, y2] 와 판정 결과로 정리한다."""
        P, p = self.params, self.percept
        timing = p.get("vision_timing") or {}
        if now - self.percept_at > 1.0 or (timing.get("sign_age_ms") or 0) > 2000:
            return []
        out = []
        for sign in p.get("signs") or []:
            try:
                x, y, w, h = (float(v) for v in sign["box"])
                confidence = float(sign["confidence"])
                label = str(sign.get("label", ""))
            except (KeyError, TypeError, ValueError):
                continue
            # 화면 끝에 걸린 박스는 쓰지 않는다 (일부만 보이는 물체: 빨간 의자를 정지 표지판으로 잡은 적이 있다).
            # 다만 다가가면 표지판은 화면 위쪽으로 빠져나가므로, 위쪽에만 걸리고 절반 넘게 보이면 인정한다
            # (2026-11-25 실제 STOP: 박스 [241, 0, 369, 127]). 그때 크기는 잘리지 않은 폭으로 잰다.
            side = x <= 4 or x + w >= WIDTH - 4 or y + h >= HEIGHT - 4
            top = y <= 4
            top_only = top and not side and h >= 0.5 * w
            cut = (top or side) and not top_only
            tall = (max(h, w) if top_only else h) / HEIGHT
            kind = "stop" if label in STOP_SIGNS else "uturn" if label == UTURN_SIGN else "avoid"
            # STOP 은 보이면 바로 주행을 끝내야 해서 기준이 따로 있다. 팀이 찍어 둔 STOP 사진 127장에서 박스 높이는
            # 화면의 7~20%, 확신도는 대부분 0.9 이상이었다: 예전 기준(25% 이상, 0.75 이상)으로는 15장만 잡혔다.
            # 그래서 확신도 0.5, 크기는 아주 작은 오인식만 거르는 5% 를 기본으로 한다.
            if kind == "stop":
                sure = confidence >= P["stop_conf"] and not cut
                near = sure and tall >= P["stop_size"]
            else:
                sure = confidence >= P["sign_conf"] and not cut
                near = sure and tall >= P["sign_near"]
            box = [round(x), round(y), round(x + w), round(y + h)]
            if label == UTURN_SIGN:
                # 유턴 화살표 모양인데 파랗지 않고 빨간 표지판이면 유턴 금지로 본다 (모델에는 그 종류가 없다)
                colors = self.front.colors(box)
                if colors is not None and colors[1] >= 0.10 and colors[1] > colors[0]:
                    kind = "nouturn"
            out.append({
                "label": label, "confidence": round(confidence, 3), "box": box,
                "tall": round(tall, 3), "center": round((x + w / 2) / WIDTH * 2 - 1, 3),
                "cut": cut, "sure": sure, "near": near,     # near: 주행이 반응하는 표지판 (STOP 은 보이는 즉시)
                # 유턴 금지: 자율 주행에서는 다른 표지판처럼 그 길을 피하고, 손 추적에서는 hand_sign 이 멈춘다
                "kind": kind, "action": "detour" if kind == "nouturn" else self.sign_action(label),
            })
        return out

    def stop_ahead(self, now):
        """STOP 표지판이 보이면 그 표지판. 최근 인식 결과 4개 중 2개 이상에서 보이고 지금도 보일 때만 알린다
        (한 번 번쩍한 오인식으로 주행이 끝나지 않게, 그리고 확신도가 기준 근처에서 오르내려도 놓치지 않게)."""
        P = self.params
        found = None
        if P["sign_rules"] and P["stop_action"] == "stop":
            for sign in self.signs_now(now):
                if sign["kind"] == "stop" and sign["near"] and (found is None or sign["tall"] > found["tall"]):
                    found = sign
        seq = (self.percept.get("vision_timing") or {}).get("sign_sequence")
        if seq is None:
            seq = int(now * 4)
        if seq != self.stop_seq:
            self.stop_seq = seq
            self.stop_recent.append(found is not None)
        return found if found is not None and sum(self.stop_recent) >= 2 else None

    def find_near(self, now):
        """STOP 을 뺀 가까운 표지판 중 가장 큰 것. 인식 결과가 2회 연속으로 가까울 때만 confirmed."""
        near = None
        if self.params["sign_rules"] and now >= self.sign_ignore_until:
            for sign in self.signs_now(now):
                if sign["kind"] != "stop" and sign["near"] and sign["action"] != "ignore" \
                        and (near is None or sign["tall"] > near["tall"]):
                    near = sign
        seq = (self.percept.get("vision_timing") or {}).get("sign_sequence")
        if seq is None:
            seq = int(now * 4)
        if near is None:
            self.near_hits = 0
        elif seq != self.near_seq:
            self.near_hits += 1
        self.near_seq = seq
        return near, near is not None and self.near_hits >= 2

    def animals_shown(self, now, limit=2.5):
        """최근에 보인 동물들 (전체 화면과 확대 화면 결과를 합친 것). 인식 결과가 오래됐으면 빈 목록."""
        if now - self.extra.get("animals_time", 0) > limit:
            return []
        shown = self.extra.get("animals_shown")
        return shown if shown is not None else (self.extra.get("animals") or [])

    def animal_place(self, box, line):
        """동물 박스가 내가 가는 길에서 어디에 있는가: 0 = 길 위(이대로 가면 몸통이 닿는다), -1 = 왼쪽 옆, +1 = 오른쪽 옆.
        길 = 따라가는 선(곧은 구간을 동물의 발 높이까지 이은 것)을 가운데로 한 로버 몸통 폭. 선이 안 보이면 화면 정면.
        몸통 반폭은 두꺼운 선 굵기의 4배로 잡는다 (선 굵기는 반 크기 화면에서 LINE_WIDTH_K * (y - 지평선) 픽셀)."""
        x1, y1, x2, y2 = box
        middle = 0.5
        if line is not None:
            fx, fy, fm = line["fit"]
            middle = fx + fm * (y2 / HEIGHT - fy)
        apart = ((x1 + x2) / 2) / WIDTH - middle
        body = 4.0 * LINE_WIDTH_K * max(0.0, y2 / HEIGHT * 240.0 - LINE_HORIZON) / 320.0
        if abs(apart) <= (x2 - x1) / 2 / WIDTH + body:
            return 0
        return 1 if apart > 0 else -1

    def animal_ahead(self, now, line):
        """(내가 가는 길 위에 서 있는 동물 또는 None, 그 인식 결과의 시각). 방금 한 번의 인식 결과만 본다."""
        P = self.params
        stamp = self.extra.get("animals_time", 0)
        if now - stamp > 2.5:
            return None, stamp
        best = None
        for animal in self.extra.get("animals") or []:
            x1, y1, x2, y2 = animal["box"]
            tall = (y2 - y1) / HEIGHT
            # 길 위에 있고, 발이 화면 아래쪽 7할에 닿아 있으며(바닥 위), 너무 작지 않을 때
            if tall >= P["animal_size"] and y2 / HEIGHT >= 0.30 and self.animal_place(animal["box"], line) == 0:
                if best is None or tall > best["tall"]:
                    best = {"center": ((x1 + x2) / 2) / WIDTH * 2 - 1, "tall": tall, "foot": y2 / HEIGHT}
        return best, stamp

    def animal_beside(self, now, line):
        """옆 길에 서 있는 동물 중 가장 크게 보이는 것이 내 선의 어느 쪽인가 (-1 왼쪽, +1 오른쪽, 0 없음)."""
        best, side = 0.0, 0
        for animal in self.animals_shown(now):
            x1, y1, x2, y2 = animal["box"]
            tall = (y2 - y1) / HEIGHT
            if tall < 0.04 or y2 / HEIGHT < 0.30 or tall <= best:
                continue
            place = self.animal_place(animal["box"], line)
            if place:
                best, side = tall, place
        return side

    def start_detour(self, now, center, what):
        """그쪽으로 가지 않는다: 보이는 쪽의 반대로 제자리 회전하며 다른 선을 찾는다 (정면이면 기본 방향)."""
        P = self.params
        self.detour_side = 1 if center < -0.1 else -1 if center > 0.1 else (-1 if P["turn"] == "left" else 1)
        self.line_state = "detour"
        self.detour_at = now
        self.detour_back_until = 0.0
        self.detour_looking = False
        self.burst_until = 0.0
        self.regain = 0
        self.wall_state = "none"
        self.note(f"{what}: 그 길로 가지 않고 {'오른쪽' if self.detour_side > 0 else '왼쪽'}으로 돌아 다른 선을 찾습니다")

    # ---------- 자율 주행 (선 + 표지판) ----------
    def reset_follower(self):
        self.filtered = None
        self.integral = 0.0
        self.f_off = None             # 걸러 낸 옆 거리와 방향 차 (center_step)
        self.f_ang = None
        self.burst_until = 0.0        # 크게 벗어났을 때의 짧은 움직임 (끝나면 멈춰서 새 화면을 본다)
        self.burst_cmd = "S"
        self.burst_state = ""
        self.valid_at = None
        self.lost_at = None
        self.searching = False
        self.side = 1
        self.turning = 1
        self.regain = 0

    def reset_line(self):
        self.reset_follower()
        self.last_good = None
        self.seen = False
        self.hint = 0
        self.hint_at = None
        self.bend = 0                 # 마지막으로 본, 선이 앞에서 꺾이는 쪽
        self.bend_at = 0.0
        self.shift_time = 0.4         # 멈춰서 하는 게걸음 한 번의 길이 (결과를 보고 늘리거나 줄인다)
        self.shift_from = None
        self.chase_until = 0.0        # 이 시각까지는 옆에 보이는 선으로 가는 중이다
        self.line_state = "follow"
        self.sign_ignore_until = 0.0
        self.near_hits = 0
        self.near_seq = object()
        self.stop_recent = collections.deque(maxlen=4)
        self.stop_seq = object()
        self.detour_side = 1
        self.detour_at = 0.0
        self.stale_since = None
        self.animal_stamp = None
        self.animal_recent = collections.deque(maxlen=3)   # 최근 인식 결과들에서 내 선 위에 있던 동물 (없으면 None)
        self.animal_at = 0.0
        self.animal_ignore_until = 0.0
        self.animal_after = 0.0           # 이 시각 전에 찍힌 인식 결과는 쓰지 않는다 (우회로 돌기 전의 화면)
        self.detour_back_until = 0.0      # 우회를 시작할 때 동물이 코앞이면 이 시각까지 먼저 물러난다
        self.detour_looking = False       # 우회: 돌아선 뒤 조금씩 돌며 선을 찾는 단계인가
        self.animal_side = 0              # 옆 길에 동물이 서 있던 쪽 (-1 왼쪽, +1 오른쪽)
        self.animal_side_at = 0.0
        self.wall_state = "none"      # none | approach | push | turn | seek
        self.wall_at = 0.0
        self.wall_until = 0.0
        self.wall_dir = 1
        self.wall_turn_at = 0.0
        self.wall_turn_len = 0.0
        self.wall_regain = 0
        self.wall_turns = 0
        self.wall_after_turn = 0.0
        self.seek_at = 0.0
        self.wall_reversed = False
        self.wall_hits = 0
        self.open_score = 0.0
        self.open_at = 0.0

    # ---------- 코너: 선이 안 보이면 벽에 닿을 때까지 직진 → 90도 회전 → 직진 ----------
    def begin_turn(self, now, direction, length, why):
        self.wall_state = "turn"
        self.wall_dir = direction
        self.bend_at = 0.0            # 꺾인 쪽 기억은 이번 회전에 썼다: 다음 코너는 새로 본 것으로 정한다
        self.animal_side_at = 0.0
        self.wall_turn_at = now
        self.wall_turn_len = length
        self.wall_regain = 0
        angle = "180도" if length > self.params["turn90"] * 1.5 else "90도"
        self.state = f"{'오른쪽' if direction > 0 else '왼쪽'}으로 회전"
        self.note(f"{why}: {'오른쪽' if direction > 0 else '왼쪽'}으로 {angle} 회전")

    def pick_side(self, now):
        """도는 방향과 그 근거. +1 = 오른쪽."""
        if now - self.animal_side_at < 12.0:
            return -self.animal_side, "동물이 없는 쪽"       # 동물이 서 있는 길로는 돌지 않는다
        if now - self.bend_at < 4.0:
            return self.bend, "선이 꺾인 쪽"
        if now - self.open_at < 3.0 and abs(self.open_score) >= 0.1:
            return (1 if self.open_score > 0 else -1), "화면에서 열린 쪽"
        if self.last_good is not None and abs(self.last_good) >= 0.04:
            return (1 if self.last_good > 0 else -1), "선이 사라진 쪽"
        if self.hint_at is not None and now - self.hint_at < 4.0:
            return self.hint, "갈림길 표시"
        return (-1 if self.params["turn"] == "left" else 1), "기본 방향"

    def wall_step(self, now, valid, error):
        """모터 명령을 돌려준다. None 이면 선을 다시 잡았으니 일반 추종으로 넘어간다."""
        P, F = self.params, self.front
        fresh = now - F.at < 0.4
        speed = int(round(P["base"] * 0.8))
        straight = f"M {speed} {speed}"
        pivot = int(P["pivot"])
        self.wall_regain = self.wall_regain + 1 if valid else 0
        seen = F.line if (fresh and F.line is not None and F.line["n"] >= 4) else None

        if self.wall_state == "seek":
            # 코너에서 돈 뒤: 직진하지 않고 옆으로 움직이며 두꺼운 선을 찾는다. 돈 직후에는 새 선이 화면 한쪽 끝에
            # 조금만 보이거나 아직 안 보인다. 그대로 직진하면 선과 나란히 가면서 트랙을 벗어난다 (2026-11-25 주행 기록).
            if self.wall_regain >= 2:
                self.wall_state = "none"
                self.reset_follower()
                self.chase_until = now + 3.0
                self.note("선을 찾아 이어갑니다")
                return None
            if seen is None and not self.wall_reversed and now - self.seek_at < 1.2:
                # 선은 안 보이고 코앞이 전부 막혀 있다: 닫힌 쪽으로 돈 것이니 반대로 180도
                self.wall_hits = self.wall_hits + 1 if (fresh and F.fill >= 0.95) else 0
                if self.wall_hits >= 3:
                    self.wall_reversed = True
                    self.begin_turn(now, -self.wall_dir, 2 * P["turn90"], "돌자마자 앞이 막혀 있어 반대로")
                    return pivot_command(self.wall_dir, pivot)
            if now - self.seek_at < 0.4:
                self.state = "회전 끝, 선을 찾는 중"       # 돈 직후에는 멈춰서 흔들리지 않은 화면을 먼저 본다
                return "S"
            hold = self.burst_hold(now)
            if hold is not None:
                return hold
            waited = now - self.wall_after_turn
            if seen is not None:
                if abs(seen["x"]) <= 0.2:
                    self.state = "선이 앞에 조금 보여 다가가는 중"
                    return straight
                side = 1 if seen["x"] > 0 else -1          # 조금이라도 보이면 그쪽으로
                self.wall_after_turn = max(self.wall_after_turn, now - 2.0)      # 보이는 동안은 포기하지 않는다
            if waited > 9.0 or now - self.seek_at > 16.0:
                self.wall_state = "approach"               # 옆으로 찾아도 없다: 예전처럼 직진하며 찾는다
                self.wall_at = now
                self.wall_after_turn = 0.0
                self.note("옆으로 찾아도 선이 없어 직진합니다")
                return straight
            if seen is None:
                # 선은 방금까지 가던 방향 쪽에 있을 가능성이 높다 (왼쪽으로 돌았으면 오른쪽). 먼저 그쪽, 다음에 반대쪽
                side = -self.wall_dir if waited < 3.0 else self.wall_dir
            return self.start_burst(now, 0.6, strafe_command(side, int(P["shift_pwm"])),
                                    f"옆으로 움직이며 선을 찾는 중 ({'오른쪽' if side > 0 else '왼쪽'})")

        if self.wall_state in ("approach", "push"):
            if self.wall_regain >= 3:              # 선이 다시 또렷이 보인다
                self.wall_state = "none"
                self.reset_follower()
                self.note("선이 다시 보여 이어갑니다")
                return None
            center = F.rows["C"]
            if fresh and center is not None and center >= 0.40 and F.fill < 0.9:
                # 앞 벽이 보이는 동안 좌우 벽 높이를 비교해 열린 쪽을 기억한다 (벽이 먼 쪽 = 열린 쪽)
                gap = (F.rows["L"] or 0.0) - (F.rows["R"] or 0.0)
                self.open_score += 0.3 * (gap - self.open_score)
                self.open_at = now

        if self.wall_state == "approach":
            if self.wall_after_turn and now - self.wall_after_turn < 0.8 and not self.wall_reversed:
                # 방금 돈 직후 0.8초: 코앞이 벽이면 닫힌 쪽으로 돈 것이니 반대로 180도 돈다 (같은 쪽으로 또 돌면 되돌아가게 된다)
                self.wall_hits = self.wall_hits + 1 if (fresh and F.fill >= 0.95) else 0
                if self.wall_hits >= 3:
                    self.wall_reversed = True
                    self.begin_turn(now, -self.wall_dir, 2 * P["turn90"], "돌자마자 벽이 막고 있어 반대로")
                    return pivot_command(self.wall_dir, pivot)
                self.state = "회전 끝, 직진"
                return straight
            if fresh and F.fill >= P["wall_fill"]:
                self.wall_state = "push"
                self.wall_until = now + P["bump_push"]
            elif now - self.wall_at > P["bump_max"]:
                side, why = self.pick_side(now)
                self.begin_turn(now, side, P["turn90"], f"벽을 확인하지 못한 채 {P['bump_max']:.1f}초 직진함 ({why})")
                return pivot_command(self.wall_dir, pivot)
            else:
                self.state = "선이 안 보여 벽까지 직진"
                return straight
        if self.wall_state == "push":
            if now < self.wall_until:
                self.state = "벽에 붙는 중"
                return straight
            if self.wall_turns >= 4:
                self.wall_state = "none"
                return self.stop_active("벽에서 네 번 돌아도 선을 찾지 못해 멈췄습니다")
            side, why = self.pick_side(now)
            self.wall_reversed = False
            self.begin_turn(now, side, P["turn90"], f"벽에 닿음 ({why})")

        elapsed = now - self.wall_turn_at
        if valid and abs(error) < 0.5 and self.wall_regain >= 2 and elapsed >= 0.6 * self.wall_turn_len:
            self.wall_state = "none"
            self.reset_follower()
            self.note("회전 중에 선이 보여 이어갑니다")
            return None
        if elapsed < self.wall_turn_len:
            self.state = f"{'오른쪽' if self.wall_dir > 0 else '왼쪽'}으로 회전"
            return pivot_command(self.wall_dir, pivot)
        self.wall_state = "seek"                  # 회전 끝: 옆으로 움직이며 선을 찾는다
        self.wall_at = now
        self.wall_after_turn = now
        self.seek_at = now
        self.wall_hits = 0
        self.wall_turns += 1
        self.open_score = 0.0
        self.burst_until = 0.0
        self.state = "회전 끝, 선을 찾는 중"
        return "S"

    def stop_active(self, reason):
        self.active = False
        self.state = "정지"
        self.reason = reason
        self.note(reason)
        return "S"

    def line_view(self, now):
        """(따라갈 수 있는가, 좌우 위치 -1..1, 추적 결과). 굵기로 고른 두꺼운 중앙선만 쓴다 (얇은 테두리 선은 쓰지 않는다)."""
        F = self.front
        line = F.line if now - F.at < 0.5 else None
        if line is None:
            return False, None, None
        return line["conf"] >= self.params["min_conf"], line["x"], line

    def burst_hold(self, now):
        """짧은 움직임(burst)이 진행 중이면 그 명령, 막 끝났으면 멈춘 뒤에 찍힌 화면이 올 때까지 정지. 아니면 None."""
        if now < self.burst_until:
            self.state = self.burst_state
            return self.burst_cmd
        if self.burst_until and self.front.at < self.burst_until + 0.25 and now - self.burst_until < 1.5:
            self.state = self.burst_state + " (멈추고 화면 확인)"
            return "S"
        return None

    def start_burst(self, now, seconds, command, state):
        self.burst_until = now + seconds
        self.burst_cmd, self.burst_state, self.state = command, state, state
        return command

    def center_step(self, now, line):
        """두꺼운 중앙선 위에 로버를 맞춘다 (메카넘): 옆으로 벗어난 만큼 게걸음, 방향이 틀어진 만큼 회전.
        몸을 선과 나란히 둔 채 옆으로 옮기므로, 방향을 틀어서 맞출 때처럼 모서리가 옆 선으로 나가지 않는다.
        크게 벗어났을 때는 전진하지 않고, 짧게 움직인 뒤 멈춰서 새 화면으로 다시 본다 (화면이 0.2~0.4초 늦어
        계속 움직이면 지나치기 때문이다. 손 추적에서 쓰는 것과 같은 방법)."""
        P = self.params
        if self.f_off is None or self.burst_until:
            self.f_off, self.f_ang = line["offset"], line["angle"]      # 처음이거나 방금 멈춘 뒤: 새 값으로 시작
            self.burst_until = 0.0
        else:
            self.f_off += 0.5 * (line["offset"] - self.f_off)
            self.f_ang += 0.5 * (line["angle"] - self.f_ang)
        off, ang = self.f_off, self.f_ang
        base = P["base"]
        if abs(line["x"]) > 0.5 or abs(off) > P["shift_at"]:
            self.chase_until = now + 3.0      # 선이 옆으로 멀리 있다: 잠깐 짧게 보여도 코너로 치지 않고 계속 그쪽으로 간다
        if abs(ang) > 0.35:
            # 방향이 20도 넘게 틀어졌다: 제자리에서 선 방향으로 돈다 (비스듬히 전진하면 옆 선을 밟는다).
            # 90도 도는 시간 설정에서 필요한 시간을 어림하고, 지나치지 않게 그 7할만 돈다.
            seconds = clamp(0.7 * abs(ang) / (math.pi / 2) * P["turn90"], 0.1, 0.6)
            return self.start_burst(now, seconds, pivot_command(1 if ang > 0 else -1, int(P["pivot"])),
                                    f"선 방향으로 회전 ({'오른쪽' if ang > 0 else '왼쪽'})")
        if abs(off) > P["shift_at"]:
            # 옆으로 많이 벗어났다: 전진하지 않고 게걸음으로 선 위로 온다.
            # 한 번에 움직이는 시간은 지난번 결과를 보고 정한다: 거의 못 움직였으면 길게, 선을 지나쳤으면 짧게
            # (2026-11-25 기록: 0.2초씩으로는 8번을 해도 제자리였다. 바닥과 배터리에 따라 달라진다)
            if self.shift_from is not None:
                if self.shift_from * off < 0:
                    self.shift_time = max(0.12, self.shift_time * 0.5)
                elif abs(off) > 0.8 * abs(self.shift_from):
                    self.shift_time = min(1.2, self.shift_time * 1.5)
            self.shift_from = off
            self.chase_until = now + 3.0
            return self.start_burst(now, self.shift_time, strafe_command(1 if off > 0 else -1, int(P["shift_pwm"])),
                                    f"선 쪽으로 게걸음 ({'오른쪽' if off > 0 else '왼쪽'})")
        self.shift_from = None
        turn = clamp(-P["turn_gain"] * ang, -0.5 * base, 0.5 * base)      # + = 반시계(왼쪽)
        side = clamp(-P["strafe_gain"] * off, -0.6 * base, 0.6 * base)    # + = 왼쪽
        forward = base * (1 - P["slow"] * min(1.0, max(abs(off) / P["shift_at"], abs(ang) / 0.35)))
        self.state = "선을 따라가는 중"
        wheel = {"FL": forward - side - turn, "FR": forward + side + turn,
                 "RL": forward + side - turn, "RR": forward - side + turn}
        top = max(255.0, max(abs(v) for v in wheel.values()))
        return "W " + " ".join(str(int(round(wheel[name] * 255.0 / top))) for name in WHEEL_ORDER)

    def line_step(self, now):
        P, p = self.params, self.percept
        fresh = (now - self.percept_at < 0.6 and bool(p.get("camera")) and now - self.frame_fresh_at < 0.6
                 and now - self.front.at < 0.6)
        if not fresh:
            # 인식이 끊기면 바로 멈추고, 3초 넘게 끊기면 주행을 끝낸다
            if self.stale_since is None:
                self.stale_since = now
            self.valid_at = None
            if now - self.stale_since > 3.0:
                return self.stop_active("카메라 인식이 3초 넘게 끊겨 멈췄습니다")
            self.state = "카메라 인식을 기다리는 중"
            return "S"
        self.stale_since = None
        junction = (p.get("road") or {}).get("junction") or "none"       # 갈림길 표시만 스택에서 받는다
        valid, error, line = self.line_view(now)
        if line is not None and line["bend"] and self.wall_state == "none":
            self.bend, self.bend_at = line["bend"], now                  # 앞에서 선이 꺾이는 쪽 (코너에서 돌 방향)
        pivot = int(P["pivot"])

        # STOP 표지판이 보이면 바로 자율 주행을 끝낸다. 우회나 벽 회전 중이어도, 다른 표지판보다 먼저 본다
        stop_sign = self.stop_ahead(now)
        if stop_sign is not None:
            return self.stop_active(
                f"STOP 표지판이 보여 자율 주행을 끝냈습니다 (확신도 {stop_sign['confidence'] * 100:.0f}%)")

        near, confirmed = self.find_near(now)
        if confirmed and self.line_state != "detour":
            name = "유턴 금지" if near["kind"] == "nouturn" else SIGN_KO.get(near["label"], near["label"])
            if near["action"] == "turn":
                # 파란 유턴 표지판 (설정에서 켰을 때만): 제자리에서 180도 돈 뒤 직진하며 선을 찾는다
                self.sign_ignore_until = now + P["uturn_time"] + 3.0
                self.line_state = "follow"
                self.wall_reversed = True
                self.begin_turn(now, -1 if P["turn"] == "left" else 1, P["uturn_time"], f"{name} 표지판")
            else:
                self.start_detour(now, near["center"], f"{name} 표지판이 가깝습니다")

        # 동물이 서 있는 선으로는 가지 않고, 인식한 동물은 절대 치지 않는다.
        #  - 갈림길에서 다른 선이 보이면 추적기가 동물이 없는 선을 고른다 (find_line 의 blocked).
        #  - 옆 길에 서 있으면 그쪽을 기억해 두고, 다음 코너·갈림길에서 그쪽으로 돌지 않는다 (pick_side).
        #  - 내가 가는 길(몸통 폭) 위에 있으면: 한 번만 인식돼도 그 방향으로는 더 전진하지 않고, 돌아서 다른 선을 찾는다.
        #    2026-11-25 18:16 기록: 코끼리를 한 번 인식하고 멈췄지만, 너무 가까워서 다음 인식이 안 되자 "없다"고 보고
        #    다시 전진해 코끼리를 밀었다. 가까워질수록 몸이 화면 밖으로 잘려 인식이 끊긴다. 그래서 다시 확인하지 않는다.
        animal_action = P["animal_action"]
        steady = self.line_state == "follow" and self.wall_state in ("none", "approach")
        if animal_action != "ignore" and steady:
            self.front.blocked = [a["box"] for a in self.animals_shown(now)]
        else:
            self.front.blocked = []       # 도는 중에는 박스가 이미 다른 곳을 가리킨다 (인식이 1초쯤 늦다)
        if animal_action != "ignore" and self.line_state != "detour" and self.wall_state != "turn":
            animal, stamp = self.animal_ahead(now, line)
            if stamp != self.animal_stamp:
                self.animal_stamp = stamp
                if stamp > self.animal_after:
                    self.animal_recent.append(animal)
                    self.animal_at = now
                    if steady and now > self.burst_until + 0.5:
                        side = self.animal_beside(now, line)
                        if side:
                            if side != self.animal_side or now - self.animal_side_at > 12.0:
                                self.note(f"{'오른쪽' if side > 0 else '왼쪽'} 길에 동물이 보입니다: 그쪽으로는 돌지 않습니다")
                            self.animal_side, self.animal_side_at = side, now
            elif self.animal_recent and now - self.animal_at > 3.5:
                self.animal_recent.clear()        # 다음 인식 결과가 오지 않는다
            seen = [a for a in self.animal_recent if a]
            need = 1 if P["animal_trust"] == "once" else 2
            if len(seen) >= need and animal_action == "detour":
                target = seen[-1]
                self.animal_recent.clear()
                self.start_detour(now, target["center"], "가는 길 위에 동물이 서 있습니다")
                if target["foot"] >= 0.75 or target["tall"] >= 0.30:
                    # 코앞이다: 제자리에서 돌면 앞 모서리로 칠 수 있으니 먼저 조금 물러난다
                    self.detour_back_until = now + 0.5
                    self.detour_at = now + 0.5
            elif seen:
                # 기다림 설정이거나 한 번 더 확인하는 설정: 최근 세 번의 인식 중 한 번이라도 보였으면 서 있는다
                self.state = "동물이 길 위에 있어 기다리는 중" if animal_action == "wait" else "가는 길 위에 동물이 보여 확인하는 중"
                self.valid_at = None
                return "S"

        if self.line_state == "detour":
            if now < self.detour_back_until:
                self.state = "동물이 코앞이라 조금 물러나는 중"
                back = int(round(P["base"] * 0.8))
                return f"M {-back} {-back}"
            elapsed = now - self.detour_at
            if elapsed > P["detour_max"]:
                return self.stop_active("우회할 다른 길을 찾지 못해 멈췄습니다")
            if elapsed < P["detour_min"]:
                # 먼저 그 길에서 돌아선다: 이 시간 동안은 보지 않고 계속 돈다
                self.detour_looking = False
                self.state = "우회: 돌아서는 중"
                return pivot_command(self.detour_side, pivot)
            # 그 뒤로는 조금 돌고 멈춰서 또렷한 화면으로 선을 찾는다. 계속 돌면 화면이 번져 선이 지나가도 못 잡는다
            # (2026-11-25 19:28 기록: 6초 동안 한 바퀴를 돌며 선을 여러 번 지나쳤다)
            if not self.detour_looking:
                self.detour_looking = True
                self.start_burst(now, 0.0, "S", "우회: 조금씩 돌며 다른 길을 찾는 중")
            hold = self.burst_hold(now)
            if hold is not None:
                return hold
            # 도는 방향의 반대쪽 끝에 걸린 선은 방금 떠나온 선일 수 있어 받지 않는다 (새 선은 도는 쪽에서 들어온다)
            if not (valid and near is None and error * self.detour_side > -0.3):
                return self.start_burst(now, 0.35, pivot_command(self.detour_side, pivot),
                                        "우회: 조금씩 돌며 다른 길을 찾는 중")
            self.burst_until = 0.0
            self.chase_until = now + 3.0          # 선이 옆에 있으면 그쪽으로 게걸음해서 올라탄다
            self.reset_follower()
            self.line_state = "follow"
            self.seen = True
            self.sign_ignore_until = now + 2.0
            self.animal_after = now + 0.9         # 도는 동안 찍힌 화면의 결과는 버리고, 새 결과부터 다시 본다
            self.animal_recent.clear()
            self.note("다른 길의 선을 찾아 이어갑니다")

        if self.wall_state != "none":
            command = self.wall_step(now, valid, error)
            if command is not None:
                return command

        hold = self.burst_hold(now)
        if hold is not None:
            return hold
        if not valid and now < self.chase_until and line is not None and line["n"] >= 4 and abs(line["x"]) > 0.3:
            # 옆에 있는 선으로 가던 중인데 선이 화면 끝에 걸려 잠깐 짧게 보인다: 코너로 치지 말고 계속 그쪽으로 간다
            side = 1 if line["x"] > 0 else -1
            self.chase_until = now + 3.0
            return self.start_burst(now, 0.5, strafe_command(side, int(P["shift_pwm"])),
                                    f"선이 옆에 보여 그쪽으로 게걸음 ({'오른쪽' if side > 0 else '왼쪽'})")

        if valid and self.searching and (self.regain < 1 or abs(error) > 0.7):
            # 선이 다시 보이기 시작: 화면 가운데로 올 때까지 선 쪽으로 더 돈다
            self.regain += 1
            direction = (1 if error > 0 else -1) if abs(error) > 0.7 else self.turning
            self.state = "선을 찾아 회전"
            return pivot_command(direction, pivot)
        if valid:
            dt = clamp(now - self.valid_at, 0.0, 0.3) if self.valid_at is not None else PERIOD
            self.valid_at = now
            if self.filtered is None or self.lost_at is not None:
                self.filtered = error
                self.integral = 0.0
            else:
                self.filtered += 0.6 * (error - self.filtered)
            self.lost_at = None
            self.searching = False
            self.regain = 0
            self.seen = True
            self.wall_turns = 0
            self.last_good = self.filtered
            if junction in ("left", "right"):
                self.hint = -1 if junction == "left" else 1
                self.hint_at = now
            if P["steer"] == "strafe":
                return self.center_step(now, line)
            self.integral = clamp(self.integral + self.filtered * dt, -0.4, 0.4)
            turn = clamp(P["kp"] * self.filtered + P["ki"] * self.integral, -P["tmax"], P["tmax"])
            speed = P["base"] * (1 - P["slow"] * min(1.0, abs(self.filtered) / 0.4))
            left = int(round(clamp(speed * (1 + turn), -255, 255)))
            right = int(round(clamp(speed * (1 - turn), -255, 255)))
            self.state = "선을 따라가는 중"
            return f"M {left} {right}"

        self.valid_at = None
        self.regain = 0
        if not self.seen:
            self.state = "선이 보이기를 기다리는 중"
            return "S"
        if self.lost_at is None:
            self.lost_at = now
        if P["corner_mode"] == "wall":
            self.wall_state = "approach"
            self.wall_at = now
            self.wall_after_turn = 0.0
            self.wall_reversed = False
            self.wall_regain = 0
            self.wall_hits = 0
            self.open_score = 0.0
            self.note("선이 안 보여 벽에 닿을 때까지 직진합니다")
            return self.wall_step(now, valid, error)
        lost_for = now - self.lost_at
        coast, search = P["coast"], P["search"]
        if lost_for <= coast and not self.searching:
            speed = int(round(P["base"] * 0.8))
            self.state = "선이 끊겨 조금 더 직진"
            return f"M {speed} {speed}"
        if pivot > 0 and lost_for <= coast + 3 * search:
            if not self.searching:
                self.searching = True
                if self.hint_at is not None and now - self.hint_at < 4.0:
                    self.side = self.hint
                elif self.last_good is not None and abs(self.last_good) >= 0.04:
                    self.side = 1 if self.last_good > 0 else -1
                else:
                    self.side = -1 if P["turn"] == "left" else 1
                self.note(f"선을 놓쳐 {'오른쪽' if self.side > 0 else '왼쪽'}으로 돌며 찾습니다")
            back = lost_for > coast + search      # 한쪽에서 못 찾으면 반대쪽으로 두 배 돌아본다
            self.turning = -self.side if back else self.side
            self.state = "선을 찾아 반대쪽으로 회전" if back else "선을 찾아 회전"
            return pivot_command(self.turning, pivot)
        return self.stop_active("선을 찾지 못해 멈췄습니다")

    # ---------- 손 추적 ----------
    def reset_hand(self):
        self.hand_burst_until = 0.0
        self.hand_burst_cmd = "S"
        self.hand_settle_at = 0.0
        self.hand_arrived = False
        self.hand_turn_until = 0.0
        self.hand_uturn_ignore = 0.0
        self.hand_sign_key = None
        self.hand_sign_hits = 0
        self.hand_sign_seq = object()

    def hand_sign(self, now):
        """손 추적 중 표지판: ("uturn" 또는 "stop" 또는 None, 이름). 같은 판정이 인식 2회 연속일 때만 알린다."""
        found = None
        if self.params["sign_rules"]:
            for sign in self.signs_now(now):
                if not sign["sure"] or sign["tall"] < 0.10:
                    continue
                if sign["kind"] == "nouturn":
                    found = ("stop", "유턴 금지")
                    break
                if sign["kind"] == "stop" and sign["near"] and self.params["stop_action"] == "stop":
                    found = ("stop", "STOP")
                    break
                if sign["kind"] == "uturn" and now >= self.hand_uturn_ignore:
                    found = ("uturn", "유턴")
        seq = (self.percept.get("vision_timing") or {}).get("sign_sequence")
        if seq is None:
            seq = int(now * 4)
        if found is None:
            self.hand_sign_key, self.hand_sign_hits = None, 0
        elif found != self.hand_sign_key:
            self.hand_sign_key, self.hand_sign_hits = found, 1
        elif seq != self.hand_sign_seq:
            self.hand_sign_hits += 1
        self.hand_sign_seq = seq
        return found if (found and self.hand_sign_hits >= 2) else (None, "")

    def hand_step(self, now):
        P = self.params
        if now < self.hand_turn_until:
            self.state = "유턴 표지판: 180도 도는 중"
            return pivot_command(-1, int(P["pivot"]))
        event, name = self.hand_sign(now)
        if event == "stop":
            return self.stop_active(f"{name} 표지판이 보여 멈췄습니다")
        if event == "uturn":
            # 파란 유턴 표지판: 왼쪽으로 180도 돈 뒤, 손이 다시 보이면 이어서 따라간다
            self.hand_turn_until = now + P["uturn_time"]
            self.hand_uturn_ignore = now + P["uturn_time"] + 4.0
            self.hand_burst_until = 0.0
            self.hand_arrived = False
            self.hand_sign_key, self.hand_sign_hits = None, 0
            self.state = "유턴 표지판: 180도 도는 중"
            self.note("유턴 표지판이 보여 왼쪽으로 180도 돕니다")
            return pivot_command(-1, int(P["pivot"]))
        if now < self.hand_burst_until:
            self.state = "손 쪽으로 회전"
            return self.hand_burst_cmd
        hand = self.extra.get("hand")
        stamp = self.extra.get("hand_time", 0)
        if hand is None or now - stamp > 0.7:
            self.state = "손바닥을 찾는 중"
            self.hand_arrived = False
            return "S"
        center, area = hand["center_x"], hand["area"]
        # 도착한 뒤에는 손이 꽤 멀어져야 다시 움직인다 (앞뒤로 떨지 않게)
        if area >= P["hand_stop_area"] * (0.7 if self.hand_arrived else 1.0):
            if not self.hand_arrived:
                self.hand_arrived = True
                self.note("손 앞에 도착했습니다")
            self.state = "손 앞에 도착 (손이 멀어지면 다시 따라갑니다)"
            return "S"
        self.hand_arrived = False
        if abs(center) > P["hand_center"]:
            # 짧게 돌고, 멈춘 뒤에 찍힌 화면으로 다시 판단한다 (인식에 0.25~0.5초 걸려 계속 돌면 지나친다)
            captured = stamp - self.extra.get("hand_ms", 0) / 1000.0
            if captured < self.hand_settle_at:
                self.state = "손 쪽으로 회전 (다음 화면 대기)"
                return "S"
            direction = 1 if center > 0 else -1
            length = clamp(0.3 * abs(center), 0.1, 0.3)
            self.hand_burst_until = now + length
            self.hand_settle_at = now + length + 0.12
            self.hand_burst_cmd = pivot_command(direction, int(P["hand_pivot"]))
            self.state = "손 쪽으로 회전"
            return self.hand_burst_cmd
        turn = clamp(1.2 * center, -0.6, 0.6)
        base = P["hand_base"]
        self.state = "손을 따라가는 중"
        return f"M {int(round(base * (1 + turn)))} {int(round(base * (1 - turn)))}"

    # ---------- 기록 재생 ----------
    def replay_step(self, now):
        elapsed = now - self.replay_start
        while self.replay_index < len(self.replay_events) and self.replay_events[self.replay_index][0] <= elapsed:
            self.replay_cmd = self.replay_events[self.replay_index][1]
            self.replay_index += 1
        if self.replay_index >= len(self.replay_events):
            return self.stop_active("기록 재생이 끝났습니다")
        total = self.replay_events[-1][0]
        self.state = f"기록 재생 중 ({elapsed:.0f} / {total:.0f}초)"
        return self.replay_cmd

    def camera_watchdog(self):
        """카메라 화면이 멈추면 인식 서비스를 다시 시작한다 (2026-11-25: 충격·전압 저하 뒤 카메라가 멈춘 채로 있었다)."""
        import subprocess
        started = time.time()
        last = 0.0
        while True:
            time.sleep(2.0)
            now = time.time()
            try:
                age = now - os.stat(FRAME_FILE).st_mtime
            except OSError:
                age = 1e9
            if age > 8.0 and now - started > 45.0 and now - last > 45.0:
                last = now
                with self.lock:
                    self.note("카메라 화면이 멈춰 인식 서비스를 다시 시작합니다")
                try:
                    subprocess.run(["systemctl", "--user", "restart", "rover-perception.service"], timeout=60, check=False)
                except Exception as problem:
                    print("camera watchdog:", problem, flush=True)

    # ---------- 제어 루프 ----------
    def control_loop(self):
        idle_sent = 0
        beat = 0.0
        while True:
            started = time.time()
            self.read_extra()
            try:
                with self.lock:
                    if self.active and self.mode == "line":
                        command = self.line_step(started)
                    elif self.active and self.mode == "hand":
                        command = self.hand_step(started)
                    elif self.active and self.mode == "replay":
                        command = self.replay_step(started)
                    elif self.mode == "manual" and started < self.manual_until:
                        command = self.manual_cmd
                        self.state = "수동 조종 중"
                    else:
                        command = "S"
                        if self.state != "정지":
                            self.state = "수동 조종 대기" if self.view == "manual" else "대기"
                    self.command = command
                    live = self.params["motors"]
                    self.front.fast = self.active and self.mode in ("line", "hand")
                    if not (self.active and self.mode == "line"):
                        self.front.blocked = []
                    self.write_mode()
            except Exception as problem:      # 계산이 실패하면 멈춘다
                with self.lock:
                    self.active = False
                    self.state, self.command = "정지", "S"
                    self.reason = f"내부 오류로 멈췄습니다: {problem!r}"[:160]
                    self.note(self.reason)
                command, live = "S", False
            if command != "S" and live:
                self.uno.send(command)
                idle_sent = 0
            elif idle_sent < 6 or started - beat > 2.0:
                self.uno.send("S")            # 멈춤은 여러 번 보내고, 그 뒤로는 2초마다 연결 확인 겸 보낸다
                idle_sent += 1
                beat = started
            time.sleep(max(0.0, PERIOD - (time.time() - started)))

    # ---------- 주행 기록 ----------
    def record_loop(self):
        """자율 주행·손 추적을 하는 동안 상태(초당 10번)와 화면(초당 3장)을 ~/rover_runs/auto/<시각>/ 에 남긴다.
        주행이 이상했을 때 무엇을 보고 어떤 명령을 냈는지 나중에 보기 위한 것이다. 실패해도 주행에는 영향이 없다."""
        folder, log, last_frame, frames = None, None, 0.0, 0
        while True:
            time.sleep(0.1)
            try:
                with self.lock:
                    on = self.active and self.mode in ("line", "hand")
                    if not on and log is None:
                        continue
                    F, now = self.front, time.time()
                    stack = self.percept.get("road") or {}
                    row = {
                        "t": round(now, 2), "mode": self.mode, "state": self.state, "cmd": self.command,
                        "line": {k: F.line[k] for k in ("x", "row", "offset", "angle", "span", "straight", "bend", "n", "conf")} if F.line else None,
                        "left": F.left, "right": F.right, "fill": F.fill, "rows": dict(F.rows), "age": round(now - F.at, 2),
                        "stack": [stack.get("error"), stack.get("confidence"), stack.get("junction")],
                        "signs": [[g.get("label"), g.get("confidence"), g.get("box")] for g in self.percept.get("signs") or []],
                        "step": [self.line_state, self.wall_state], "reason": "" if on else self.reason,
                        "animals": [[a.get("looks_like"), a.get("confidence"), a.get("box"), a.get("view")] for a in self.extra.get("animals") or []],
                        "animal_age": round(now - self.extra.get("animals_time", 0), 2),
                        "blocked": bool(F.line and F.line.get("blocked")), "avoided": bool(F.line and F.line.get("avoided")),
                        "animal_side": self.animal_side if now - self.animal_side_at < 12.0 else 0,
                    }
                if log is None:
                    os.makedirs(AUTO_DIR, exist_ok=True)
                    old = sorted(name for name in os.listdir(AUTO_DIR) if os.path.isdir(os.path.join(AUTO_DIR, name)))
                    for name in old[:-11]:           # 최근 12번만 남긴다
                        shutil.rmtree(os.path.join(AUTO_DIR, name), ignore_errors=True)
                    folder = os.path.join(AUTO_DIR, time.strftime("%Y%m%d_%H%M%S"))
                    os.makedirs(folder, exist_ok=True)
                    log, frames = open(os.path.join(folder, "log.jsonl"), "w"), 0
                if now - last_frame >= 0.33 and frames < 600:
                    last_frame, frames = now, frames + 1
                    row["frame"] = f"{frames:04d}.jpg"
                    shutil.copyfile(FRAME_FILE, os.path.join(folder, row["frame"]))
                log.write(json.dumps(row, ensure_ascii=False) + "\n")
                if not on:
                    log.close()
                    log = None
            except Exception:
                try:
                    if log is not None:
                        log.close()
                except Exception:
                    pass
                log = None
                time.sleep(1.0)

    # ---------- 명령 ----------
    def handle(self, body):
        action = body.get("action")
        with self.lock:
            P = self.params
            if action == "start":
                mode = body.get("mode")
                if mode not in ("line", "hand"):
                    return {"ok": False, "error": "mode 는 line 또는 hand 입니다"}
                if P["motors"] and not self.uno.ok:
                    return {"ok": False, "error": "아두이노가 연결되지 않아 시작할 수 없습니다: " + self.uno.error}
                self.mode, self.view, self.active, self.reason = mode, mode, True, ""
                self.state = "시작"
                self.reset_line()
                self.reset_hand()
                self.write_mode()
                self.note("자율 주행 시작" if mode == "line" else "손 추적 시작")
            elif action == "replay":
                name = os.path.basename(str(body.get("file", "")))
                if self.active:
                    return {"ok": False, "error": "먼저 정지를 누르세요"}
                if P["motors"] and not self.uno.ok:
                    return {"ok": False, "error": "아두이노가 연결되지 않아 시작할 수 없습니다: " + self.uno.error}
                try:
                    with open(os.path.join(RUN_DIR, name)) as handle:
                        events = [json.loads(line) for line in handle if line.strip()]
                    events = [(float(e["t"]), str(e["cmd"])) for e in events]
                except (OSError, ValueError, KeyError, TypeError):
                    return {"ok": False, "error": "기록 파일을 읽을 수 없습니다"}
                if not events or any(not COMMAND.match(cmd) for _, cmd in events):
                    return {"ok": False, "error": "기록 파일 내용이 올바르지 않습니다"}
                self.replay_events, self.replay_index, self.replay_cmd = events, 0, "S"
                self.replay_start = time.time()
                self.mode, self.view, self.active, self.reason = "replay", "manual", True, ""
                self.write_mode()
                self.note(f"기록 재생 시작: {name} ({events[-1][0]:.0f}초)")
            elif action == "stop":
                was = self.active or time.time() < self.manual_until
                self.active = False
                self.mode = "idle"
                self.manual_until = 0.0
                self.state, self.reason = "대기", ""
                if was:
                    self.reason = "정지 버튼으로 멈췄습니다"
                    self.note(self.reason)
            elif action == "view":
                view = body.get("view")
                if view not in ("line", "hand", "manual"):
                    return {"ok": False, "error": "view 는 line, hand, manual 입니다"}
                owner = "manual" if self.mode == "replay" else self.mode
                if self.active and view != owner:
                    self.active = False       # 다른 탭으로 옮기면 안전하게 멈춘다
                    self.state, self.reason = "대기", "탭을 바꿔서 멈췄습니다"
                    self.note(self.reason)
                if not self.active:
                    self.mode = "idle"
                    self.manual_until = 0.0
                    if self.state == "정지" and view != self.view:
                        self.state, self.reason = "대기", ""
                self.view = view
                self.write_mode()
            elif action == "drive":
                if self.active:
                    return {"ok": False, "error": "주행 중에는 수동 조종을 할 수 없습니다. 먼저 정지를 누르세요"}
                move = body.get("move")
                if move in MOVES:
                    vx, vy, w = MOVES[move]
                    pwm = P["manual_turn"] if w else P["manual_strafe"] if vy else P["manual_speed"]
                    self.manual_cmd = "W " + " ".join(str(int(round(r * pwm))) for r in mecanum(vx, vy, w))
                    self.manual_until = time.time() + 0.45
                    self.mode = "manual"
                    self.reason = ""
                else:
                    self.manual_until = 0.0
            elif action == "set":
                problem = self.apply(body.get("key"), body.get("value"))
                if problem:
                    return {"ok": False, "error": problem}
                if body.get("key") == "motors":
                    if P["motors"] and self.active:
                        # 모터를 켜는 것만으로 갑자기 움직이지 않게: 시작을 다시 눌러야 한다
                        self.active = False
                        self.mode = "idle"
                        self.state, self.reason = "대기", "모터 출력을 켰습니다. 움직이려면 시작을 다시 누르세요"
                    self.note("모터 출력 켬" if P["motors"] else "모터 출력 끔 (시험 모드: 계산만 합니다)")
                else:
                    self.save_params()
            else:
                return {"ok": False, "error": "모르는 명령입니다"}
        return {"ok": True}

    def snapshot(self):
        now = time.time()
        with self.lock:
            P, p, extra = self.params, self.percept, self.extra
            vision = now - self.percept_at < 1.0 and now - self.frame_fresh_at < 1.0
            road = None
            F = self.front
            if vision and now - F.at < 1.5:
                line = F.line               # 굵기로 고른 중앙선 (인식 스택의 선이 아니다)
                road = {"seen": bool(line) and line["conf"] >= P["min_conf"], "error": line["x"] if line else 0.0,
                        "confidence": line["conf"] if line else 0.0, "junction": (p.get("road") or {}).get("junction"),
                        "offset": line["offset"] if line else None,
                        "angle": round(math.degrees(line["angle"]), 1) if line else None,
                        "bend": line["bend"] if line else 0,
                        "blocked": bool(line and line.get("blocked")), "avoided": bool(line and line.get("avoided")),
                        "points": line["points"] if line else [], "thin": F.thin, "left": F.left, "right": F.right,
                        "ms": F.line_ms}
            extra_ok = bool(extra) and now - extra.get("time", 0) < 3.0
            animals = self.animals_shown(now, 4.0) if extra_ok else []
            hand = extra.get("hand") if extra_ok and now - extra.get("hand_time", 0) < 1.5 else None
            try:
                camera = now - os.stat(FRAME_FILE).st_mtime < 2.0
            except OSError:
                camera = False
            try:
                with open("/sys/class/thermal/thermal_zone0/temp") as handle:
                    temp = round(int(handle.read()) / 1000, 1)
            except (OSError, ValueError):
                temp = None
            return {
                "mode": self.mode, "active": self.active, "view": self.view,
                "state": self.state, "reason": self.reason, "command": self.command,
                "uno": {"ok": self.uno.ok, "error": self.uno.error, "motors": P["motors"]},
                "camera": camera, "vision": vision,
                "vision_error": self.percept_error or ("" if vision else "카메라 화면이 새로 들어오지 않습니다"),
                "road": road, "signs": self.signs_now(now), "animals": animals, "hand": hand,
                "front": {"ok": now - self.front.at < 1.5, "rows": dict(self.front.rows), "fill": self.front.fill,
                          "error": self.front.error},
                "extra": {"mode": "hand" if "hand" in (extra.get("mode") or []) else "animals", "ok": extra_ok},
                "params": dict(P), "temp": temp, "size": [WIDTH, HEIGHT],
                "log": list(self.events)[-12:], "runs": self.list_runs(now),
            }


def load_pin():
    if "--no-pin" in sys.argv:
        return ""
    try:
        with open(PIN_FILE) as handle:
            pin = handle.read().strip()
        if pin:
            return "" if pin.lower() == "off" else pin
    except OSError:
        pass
    pin = f"{secrets.randbelow(10000):04d}"
    with open(PIN_FILE, "w") as handle:
        handle.write(pin + "\n")
    os.chmod(PIN_FILE, 0o600)
    return pin


def make_handler(rover, pin):
    fails = {}        # 주소 -> [틀린 횟수, 처음 틀린 시각]

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        timeout = 20

        def access(self):
            address = self.client_address[0]
            if not pin or address in ("127.0.0.1", "::1"):
                return 200
            now = time.time()
            record = fails.get(address)
            if record and now - record[1] >= 60:
                fails.pop(address, None)
                record = None
            if record and record[0] >= 8:
                return 429
            given = None
            for part in self.headers.get("Cookie", "").split(";"):
                name, _, value = part.strip().partition("=")
                if name == "rover_pin":
                    given = value
            if given is None:
                return 401
            if secrets.compare_digest(given.encode(), pin.encode()):
                return 200
            if record:
                record[0] += 1
            else:
                fails[address] = [1, now]
            return 401

        def reply(self, code, data, kind="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                with open(PAGE_FILE, "rb") as handle:
                    self.reply(200, handle.read(), "text/html; charset=utf-8")
                return
            code = self.access()
            if code != 200:
                self.reply(code, b'{"error":"pin"}')
            elif path == "/api/state":
                self.reply(200, json.dumps(rover.snapshot()).encode())
            elif path == "/frame.jpg":
                try:
                    if time.time() - os.stat(FRAME_FILE).st_mtime > 3.0:
                        raise OSError("stale")
                    with open(FRAME_FILE, "rb") as handle:
                        self.reply(200, handle.read(), "image/jpeg")
                except OSError:
                    self.reply(503, b"")
            else:
                self.reply(404, b"{}")

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = -1
            if not 0 <= length <= 4096:
                self.close_connection = True
                self.reply(413, b"{}")
                return
            raw = self.rfile.read(length)
            code = self.access()
            if code != 200:
                self.reply(code, b'{"error":"pin"}')
            elif self.path.split("?", 1)[0] != "/api/cmd":
                self.reply(404, b"{}")
            elif self.headers.get("X-Rover") != "1":       # 다른 웹사이트가 몰래 보내는 요청을 막는다
                self.reply(403, b"{}")
            else:
                try:
                    body = json.loads(raw or b"{}")
                    result = rover.handle(body) if isinstance(body, dict) else {"ok": False, "error": "형식 오류"}
                except (ValueError, TypeError) as problem:
                    result = {"ok": False, "error": str(problem)[:80]}
                self.reply(200, json.dumps(result).encode())

        def log_message(self, *_):
            pass

    return Handler


def main():
    port = int(sys.argv[sys.argv.index("--port") + 1]) if "--port" in sys.argv else 8000
    pin = load_pin()
    rover = Rover(dry="--dry" in sys.argv)
    threading.Thread(target=rover.poll_percept, daemon=True).start()
    threading.Thread(target=rover.front.run, daemon=True).start()
    if "--no-watchdog" not in sys.argv:
        threading.Thread(target=rover.camera_watchdog, daemon=True).start()
    threading.Thread(target=rover.control_loop, daemon=True).start()
    if "--no-record" not in sys.argv:
        threading.Thread(target=rover.record_loop, daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(rover, pin))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    print(f"rover ui on port {port}, motors {'on' if rover.params['motors'] else 'OFF (dry)'}, "
          f"pin {'on' if pin else 'off'}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            rover.uno.send("S")
            time.sleep(0.03)


if __name__ == "__main__":
    main()
