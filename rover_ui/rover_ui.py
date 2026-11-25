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
import os
import re
import secrets
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
    "sign_rules": True, "stop_action": "stop", "sign_conf": 0.75, "sign_near": 0.25,
    "detour_min": 1.2, "detour_max": 6.0,
    "animal_action": "detour", "animal_size": 0.16, "uturn_line": "ignore", "uturn_time": 2.4,
    "hand_base": 130, "hand_pivot": 200, "hand_stop_area": 0.075, "hand_center": 0.35,
    "manual_speed": 150, "manual_turn": 220, "manual_strafe": 200,
    "motors": True,
}
LIMITS = {
    "base": (0, 255), "kp": (0, 5), "ki": (0, 3), "tmax": (0.1, 1.5), "slow": (0, 0.9), "coast": (0, 2),
    "pivot": (0, 255), "search": (0.5, 8), "min_conf": (0.2, 0.95),
    "turn90": (0.3, 3.0), "bump_push": (0, 2.5), "bump_max": (1, 6), "wall_fill": (0.6, 0.98),
    "sign_conf": (0.3, 0.99), "sign_near": (0.05, 0.9), "detour_min": (0.3, 4), "detour_max": (2, 12),
    "animal_size": (0.05, 0.6), "uturn_time": (0.6, 6.0),
    "hand_base": (0, 255), "hand_pivot": (0, 255), "hand_stop_area": (0.01, 0.5), "hand_center": (0.1, 0.9),
    "manual_speed": (0, 255), "manual_turn": (0, 255), "manual_strafe": (0, 255),
}
CHOICES = {"turn": ("left", "right"), "stop_action": ("stop", "detour", "ignore"), "corner_mode": ("wall", "search"),
           "animal_action": ("detour", "wait", "ignore"), "uturn_line": ("ignore", "turn")}
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
                    self.pixels = pixels
                    self.rows, self.fill, self.at, self.error = rows, fill, time.time(), ""
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
        """자율 주행에서 가까워졌을 때 할 일: stop(멈춤), detour(그 길로 가지 않고 다른 선), turn(180도), ignore."""
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
            tall = h / HEIGHT
            # 화면 끝에 걸린 박스는 쓰지 않는다 (일부만 보이는 물체: 빨간 의자를 정지 표지판으로 잡은 적이 있다)
            cut = x <= 4 or y <= 4 or x + w >= WIDTH - 4 or y + h >= HEIGHT - 4
            sure = confidence >= P["sign_conf"] and not cut
            box = [round(x), round(y), round(x + w), round(y + h)]
            kind = "stop" if label in STOP_SIGNS else "uturn" if label == UTURN_SIGN else "avoid"
            if label == UTURN_SIGN:
                # 유턴 화살표 모양인데 파랗지 않고 빨간 표지판이면 유턴 금지로 본다 (모델에는 그 종류가 없다)
                colors = self.front.colors(box)
                if colors is not None and colors[1] >= 0.10 and colors[1] > colors[0]:
                    kind = "nouturn"
            out.append({
                "label": label, "confidence": round(confidence, 3), "box": box,
                "tall": round(tall, 3), "center": round((x + w / 2) / WIDTH * 2 - 1, 3),
                "cut": cut, "sure": sure, "near": sure and tall >= P["sign_near"],
                # 유턴 금지: 자율 주행에서는 다른 표지판처럼 그 길을 피하고, 손 추적에서는 hand_sign 이 멈춘다
                "kind": kind, "action": "detour" if kind == "nouturn" else self.sign_action(label),
            })
        return out

    def find_near(self, now):
        """가까운 표지판 중 가장 큰 것. 인식 결과가 2회 연속으로 가까울 때만 confirmed."""
        near = None
        if self.params["sign_rules"] and now >= self.sign_ignore_until:
            for sign in self.signs_now(now):
                if sign["near"] and sign["action"] != "ignore" and (near is None or sign["tall"] > near["tall"]):
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

    def animal_ahead(self, now):
        """(가는 길 앞에 가까이 있는 동물 중 가장 큰 것 또는 None, 그 인식 결과의 시각)."""
        P = self.params
        stamp = self.extra.get("animals_time", 0)
        if now - stamp > 2.5:
            return None, stamp
        best = None
        for animal in self.extra.get("animals") or []:
            x1, y1, x2, y2 = animal["box"]
            center = ((x1 + x2) / 2) / WIDTH * 2 - 1
            tall = (y2 - y1) / HEIGHT
            # 화면 가운데 쪽에 있고, 발이 화면 아래쪽 절반에 닿아 있으며(바닥 위 가까운 곳), 충분히 크게 보일 때
            if abs(center) < 0.55 and y2 / HEIGHT >= 0.45 and tall >= P["animal_size"]:
                if best is None or tall > best["tall"]:
                    best = {"center": center, "tall": tall}
        return best, stamp

    def start_detour(self, now, center, what):
        """그쪽으로 가지 않는다: 보이는 쪽의 반대로 제자리 회전하며 다른 선을 찾는다 (정면이면 기본 방향)."""
        P = self.params
        self.detour_side = 1 if center < -0.1 else -1 if center > 0.1 else (-1 if P["turn"] == "left" else 1)
        self.line_state = "detour"
        self.detour_at = now
        self.regain = 0
        self.wall_state = "none"
        self.note(f"{what}: 그 길로 가지 않고 {'오른쪽' if self.detour_side > 0 else '왼쪽'}으로 돌아 다른 선을 찾습니다")

    # ---------- 자율 주행 (선 + 표지판) ----------
    def reset_follower(self):
        self.filtered = None
        self.integral = 0.0
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
        self.line_state = "follow"
        self.sign_ignore_until = 0.0
        self.near_hits = 0
        self.near_seq = object()
        self.detour_side = 1
        self.detour_at = 0.0
        self.stale_since = None
        self.animal_stamp = None
        self.animal_hits = 0
        self.animal_at = 0.0
        self.animal_ignore_until = 0.0
        self.wall_state = "none"      # none | approach | push | turn
        self.wall_at = 0.0
        self.wall_until = 0.0
        self.wall_dir = 1
        self.wall_turn_at = 0.0
        self.wall_turn_len = 0.0
        self.wall_regain = 0
        self.wall_turns = 0
        self.wall_after_turn = 0.0
        self.wall_reversed = False
        self.wall_hits = 0
        self.open_score = 0.0
        self.open_at = 0.0

    # ---------- 코너: 선이 안 보이면 벽에 닿을 때까지 직진 → 90도 회전 → 직진 ----------
    def begin_turn(self, now, direction, length, why):
        self.wall_state = "turn"
        self.wall_dir = direction
        self.wall_turn_at = now
        self.wall_turn_len = length
        self.wall_regain = 0
        angle = "180도" if length > self.params["turn90"] * 1.5 else "90도"
        self.state = f"{'오른쪽' if direction > 0 else '왼쪽'}으로 회전"
        self.note(f"{why}: {'오른쪽' if direction > 0 else '왼쪽'}으로 {angle} 회전")

    def pick_side(self, now):
        """도는 방향과 그 근거. +1 = 오른쪽."""
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
        self.wall_regain = self.wall_regain + 1 if (valid and abs(error) < 0.6) else 0

        if self.wall_state in ("approach", "push"):
            if self.wall_regain >= 4:              # 선이 다시 또렷이 보인다
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
        if valid and abs(error) < 0.35 and self.wall_regain >= 2 and elapsed >= 0.6 * self.wall_turn_len:
            self.wall_state = "none"
            self.reset_follower()
            self.note("회전 중에 선이 보여 이어갑니다")
            return None
        if elapsed < self.wall_turn_len:
            self.state = f"{'오른쪽' if self.wall_dir > 0 else '왼쪽'}으로 회전"
            return pivot_command(self.wall_dir, pivot)
        self.wall_state = "approach"              # 회전 끝: 직진하면서 선을 찾는다
        self.wall_at = now
        self.wall_after_turn = now
        self.wall_hits = 0
        self.wall_turns += 1
        self.open_score = 0.0
        self.state = "회전 끝, 직진"
        return straight

    def stop_active(self, reason):
        self.active = False
        self.state = "정지"
        self.reason = reason
        self.note(reason)
        return "S"

    def line_step(self, now):
        P, p = self.params, self.percept
        fresh = now - self.percept_at < 0.6 and bool(p.get("camera")) and now - self.frame_fresh_at < 0.6
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
        road = p.get("road") or {}
        conf = road.get("confidence") or 0.0
        error = road.get("error")
        junction = road.get("junction") or "none"
        valid = error is not None and conf >= P["min_conf"]
        pivot = int(P["pivot"])

        near, confirmed = self.find_near(now)
        if confirmed and self.line_state != "detour":
            name = "유턴 금지" if near["kind"] == "nouturn" else SIGN_KO.get(near["label"], near["label"])
            if near["action"] == "stop":
                return self.stop_active(f"{name} 표지판 앞에서 멈췄습니다")
            if near["action"] == "turn":
                # 파란 유턴 표지판 (설정에서 켰을 때만): 제자리에서 180도 돈 뒤 직진하며 선을 찾는다
                self.sign_ignore_until = now + P["uturn_time"] + 3.0
                self.line_state = "follow"
                self.wall_reversed = True
                self.begin_turn(now, -1 if P["turn"] == "left" else 1, P["uturn_time"], f"{name} 표지판")
            else:
                self.start_detour(now, near["center"], f"{name} 표지판이 가깝습니다")

        # 길 위에 동물이 보이면 그 길로 가지 않는다. 인식이 느려(약 1초에 한 번) 처음 보이면 멈춰서 한 번 더 확인한다
        animal_action = P["animal_action"]
        if animal_action != "ignore" and self.line_state != "detour" and now >= self.animal_ignore_until:
            animal, stamp = self.animal_ahead(now)
            if stamp != self.animal_stamp:
                self.animal_stamp = stamp
                self.animal_hits = self.animal_hits + 1 if animal else 0
                self.animal_at = now
            elif self.animal_hits and now - self.animal_at > 3.5:
                self.animal_hits = 0              # 다음 인식 결과가 오지 않는다: 다시 간다
            if self.animal_hits >= 2 and animal and animal_action == "detour":
                self.animal_hits = 0
                self.animal_ignore_until = now + 4.0
                self.start_detour(now, animal["center"], "길에 동물이 있습니다")
            elif self.animal_hits >= 1:
                self.state = "동물이 길에 있어 기다리는 중" if self.animal_hits >= 2 else "동물이 보여 확인하는 중"
                self.valid_at = None
                return "S"

        if self.line_state == "detour":
            elapsed = now - self.detour_at
            if elapsed > P["detour_max"]:
                return self.stop_active("우회할 다른 길을 찾지 못해 멈췄습니다")
            if elapsed >= P["detour_min"] and valid and abs(error) < 0.6 and near is None:
                self.regain += 1
            else:
                self.regain = 0
            if self.regain < 2:
                self.state = "우회: 제자리에서 돌며 다른 길을 찾는 중"
                return pivot_command(self.detour_side, pivot)
            self.reset_follower()
            self.line_state = "follow"
            self.seen = True
            self.sign_ignore_until = now + 2.0
            self.animal_ignore_until = max(self.animal_ignore_until, now + 2.0)
            self.note("다른 길의 선을 찾아 이어갑니다")

        if self.wall_state != "none":
            command = self.wall_step(now, valid, error)
            if command is not None:
                return command

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
                if sign["label"] in STOP_SIGNS and sign["near"]:
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
            if vision:
                raw = p.get("road") or {}
                error, conf = raw.get("error"), raw.get("confidence") or 0.0
                road = {"seen": error is not None and conf >= P["min_conf"], "error": round(error or 0.0, 3),
                        "confidence": round(conf, 3), "junction": raw.get("junction")}
            extra_ok = bool(extra) and now - extra.get("time", 0) < 3.0
            animals = (extra.get("animals") or []) if extra_ok and now - extra.get("animals_time", 0) < 4.0 else []
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
