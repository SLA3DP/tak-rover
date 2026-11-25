"""가상 트랙에서 실제 주행 코드(Rover.line_step)를 돌려 본다: 가상 카메라 화면 -> 추적기 -> 제어 -> 바퀴 -> 로버 위치.
로버의 실제 힘과 미끄러짐은 모르므로, 부호와 안정성(지연이 있어도 흔들리지 않는지)을 보는 용도다.
노트북에서 돌린다 (numpy, PIL 필요). 로버와 통신하지 않는다.
사용: python3 tools/sim/sim_drive.py [결과를 둘 폴더] [그림 파일]"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
S = sys.argv[1] if len(sys.argv) > 1 else tempfile.mkdtemp(prefix="tak-sim-")
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "..", "rover_ui"))
import rover_ui as R
import track_render as T

os.makedirs(S + "/sim/out", exist_ok=True)
os.makedirs(S + "/sim/runs", exist_ok=True)
R.EXTRA_DIR = S + "/sim/out"
R.EXTRA_FILE = S + "/sim/out/extra.json"
R.MODE_FILE = S + "/sim/out/mode.txt"
R.PARAM_FILE = S + "/sim/params_drive.json"
R.RUN_DIR = S + "/sim/runs"
R.AUTO_DIR = S + "/sim/runs/auto"
R.FRAME_FILE = S + "/sim/latest.jpg"
R.SERIAL = "/dev/does-not-exist"
if os.path.exists(R.PARAM_FILE):
    os.remove(R.PARAM_FILE)

DT = 0.066
DELAY = 3                 # 화면이 찍히고 명령이 나가기까지 약 0.2초
CAM = 0.5                 # 카메라가 회전 중심보다 앞에 있는 거리 (카메라 높이 단위, 추정)
HALF_W, HALF_L = 0.52, 0.62        # 로버 반폭·반길이 (추정: 22 x 26 cm, 카메라 높이 21 cm)
HSV = ((15, 0, 170), (45, 255, 255))


def wheel_speed(pwm, dead=40.0):
    return math.copysign(max(0.0, abs(pwm) - dead) * (1.2 / 110.0), pwm)


def body_motion(command, strafe_eff=0.7, lever=1.5):
    """명령 문자열 -> (전진, 왼쪽, 반시계 각속도). 단위: 카메라 높이/초, rad/초."""
    parts = command.split()
    if parts[0] == "M":
        left, right = float(parts[1]), float(parts[2])
        w = {"FL": left, "RL": left, "FR": right, "RR": right}
    elif parts[0] == "W":
        w = dict(zip(R.WHEEL_ORDER, (float(v) for v in parts[1:5])))
    else:
        w = {"FL": 0, "FR": 0, "RL": 0, "RR": 0}
    v = {k: wheel_speed(p) for k, p in w.items()}
    forward = (v["FL"] + v["FR"] + v["RL"] + v["RR"]) / 4
    left = (-v["FL"] + v["FR"] + v["RL"] - v["RR"]) / 4 * strafe_eff
    spin = (-v["FL"] + v["FR"] - v["RL"] + v["RR"]) / (4 * lever)
    return forward, left, spin


def inside_corridor(px, pz, corridor):
    """corridor: [(x_min, x_max, z_min, z_max), ...] 직사각형들의 합집합 (벽 안쪽)."""
    return any(a <= px <= b and c <= pz <= d for a, b, c, d in corridor)


def wall_view(cam, corridor):
    """벽 밑단이 화면에 보이는 높이(왼쪽·가운데·오른쪽)와 화면 가림을 거칠게 흉내 낸다."""
    x, z, yaw = cam
    rows = {}
    for name, turn in (("L", 0.35), ("C", 0.0), ("R", -0.35)):
        hx, hz = -math.sin(yaw + turn), math.cos(yaw + turn)
        d = 0.0
        while d < 8.0 and inside_corridor(x + hx * d, z + hz * d, corridor):
            d += 0.05
        zc = d * math.cos(turn)
        if zc >= 7.9:
            rows[name] = None
            continue
        v = T.CY + T.FOCAL * (-zc * math.sin(T.PITCH) + math.cos(T.PITCH)) / (zc * math.cos(T.PITCH) + math.sin(T.PITCH))
        rows[name] = round(min(1.0, max(0.0, v / (T.H - 1.0))), 3)
    base = (rows["C"] or 0.0) * (T.H - 1)
    fill = min(1.0, max(0.0, (base - 0.45 * T.H) / (0.55 * T.H)))
    return {k: (v if v is not None and v >= 0.25 else None) for k, v in rows.items()}, round(fill, 3)


def simulate(center, start, steer, seconds=12.0, settings=None, quiet=False, branches=(), corridor=None):
    """start: (선에서 오른쪽으로 벗어난 거리, 왼쪽으로 돌아간 각도(도)). 돌려주는 값: 기록 목록."""
    r = R.Rover(dry=True)
    r.params.update({"steer": steer, "sign_rules": False, "animal_action": "ignore"})
    r.params.update(settings or {})
    x, z, yaw = start[0], 0.0, math.radians(start[1])          # 몸통 중심
    r.handle({"action": "start", "mode": "line"})
    queue, log, t = [], [], 1000.0
    seq = 0
    for step in range(int(seconds / DT)):
        cam = (x - math.sin(yaw) * CAM, z + math.cos(yaw) * CAM, yaw)
        img = T.render(center, cam, branches=branches)
        queue.append((R.find_line(img, np, HSV, None), wall_view(cam, corridor) if corridor else r.front.analyse(img, np)))
        found, (rows, fill) = queue[max(0, len(queue) - 1 - DELAY)]
        seq += 1
        r.percept = {"camera": True, "road": {"junction": "none"}, "signs": [],
                     "vision_timing": {"frame_sequence": seq, "sign_sequence": seq, "sign_age_ms": 100}}
        r.percept_at = r.frame_fresh_at = t
        F = r.front
        F.line, F.left, F.right, F.thin, F.rows, F.fill, F.at = found["line"], found["left"], found["right"], found["thin"], rows, fill, t
        command = r.line_step(t)
        forward, left, spin = body_motion(command)
        hx, hz = -math.sin(yaw), math.cos(yaw)                  # 앞 방향
        lx, lz = -math.cos(yaw), -math.sin(yaw)                 # 왼쪽 방향
        nx = x + (forward * hx + left * lx) * DT
        nz = z + (forward * hz + left * lz) * DT
        # 벽은 통과하지 못한다: 몸통 앞머리가 벽 밖으로 나가면 그 자리에 선다 (벽을 미는 중)
        if corridor is None or inside_corridor(nx + HALF_L * hx, nz + HALF_L * hz, corridor):
            x, z = nx, nz
        yaw += spin * DT
        # 몸통 네 모서리가 중앙선에서 떨어진 거리 (테두리는 LANE)
        corners = [(x + a * HALF_L * hx - b * HALF_W * lx, z + a * HALF_L * hz - b * HALF_W * lz) for a in (1, -1) for b in (1, -1)]
        px = np.array([[c[0] for c in corners] + [x]], dtype=np.float32)
        pz = np.array([[c[1] for c in corners] + [z]], dtype=np.float32)
        d = T.dist_to_polyline(px, pz, center)[0]
        log.append({"t": step * DT, "dev": float(d[4]), "corner": float(d[:4].max()), "yaw": yaw, "cmd": command,
                    "state": r.state, "x": x, "z": z, "line": found["line"]})
        if not r.active:
            break
        t += DT
    return log, r


def summary(name, log, r):
    late = [e for e in log if e["t"] >= 4.0] or log
    worst_corner = max(e["corner"] for e in log)
    states = []
    for e in log:
        if not states or states[-1] != e["state"]:
            states.append(e["state"])
    stepped = worst_corner >= T.LANE - T.THIN_W / 2
    print(f"{name:44s} 중심 벗어남 최대 {max(e['dev'] for e in log):.2f} / 4초 뒤 {max(e['dev'] for e in late):.2f} | "
          f"모서리 최대 {worst_corner:.2f} (테두리 {T.LANE:.2f}) {'밟음' if stepped else '안 밟음'} | 간 거리 {log[-1]['z']:.1f}"
          + ("" if r.active else f" | 끝: {r.reason}"))
    print("      " + " → ".join(states[:8]))
    return stepped


if __name__ == "__main__":
    straight = T.straight(40.0)
    arc = [(0.0, -2.0), (0.0, 3.0)] + [(-6.0 * (1 - math.cos(a)), 3.0 + 6.0 * math.sin(a)) for a in np.linspace(0.1, 1.0, 12)]
    arc.append((arc[-1][0] - 20 * math.sin(1.0), arc[-1][1] + 20 * math.cos(1.0)))
    cases = [
        ("직선, 가운데에서 출발", straight, (0.0, 0.0)),
        ("직선, 오른쪽으로 0.3 벗어나 출발", straight, (0.3, 0.0)),
        ("직선, 왼쪽으로 0.3 벗어나 출발", straight, (-0.3, 0.0)),
        ("직선, 왼쪽으로 15도 틀어져 출발", straight, (0.0, 15.0)),
        ("직선, 오른쪽 0.25 + 오른쪽으로 12도", straight, (0.25, -12.0)),
        ("직선, 오른쪽으로 0.6 벗어나 출발", straight, (0.6, 0.0)),
        ("직선, 왼쪽으로 35도 틀어져 출발", straight, (0.0, 35.0)),
        ("완만한 왼쪽 굽은 길 (반지름 6)", arc, (0.0, 0.0)),
    ]
    results = {}
    for steer in ("strafe", "turn"):
        print(f"=== 따라가는 방식: {'게걸음으로 가운데 맞춤' if steer == 'strafe' else '방향만 틀기 (예전)'}")
        for name, center, start in cases:
            log, r = simulate(center, start, steer)
            results[(steer, name)] = (log, summary(name, log, r))
    if len(sys.argv) > 2:
        from PIL import Image, ImageDraw
        im = Image.new("RGB", (900, 520), (20, 22, 28))
        draw = ImageDraw.Draw(im)
        for col, steer in enumerate(("strafe", "turn")):
            for row, name in enumerate(n for n, _, _ in cases[:6]):
                log = results[(steer, name)][0]
                ox, oy = 10 + col * 450, 10 + row * 85
                draw.text((ox, oy), f"{steer}: {name}", fill=(220, 220, 220))
                mid = oy + 48
                draw.line((ox, mid, ox + 430, mid), fill=(230, 200, 40), width=2)
                for lane in (-1, 1):
                    draw.line((ox, mid + lane * 30, ox + 430, mid + lane * 30), fill=(120, 105, 30), width=1)
                pts = [(ox + min(430, e["z"] * 22), mid + e["x"] / T.LANE * 30) for e in log]
                draw.line(pts, fill=(90, 220, 255), width=2)
        im.save(sys.argv[2])
