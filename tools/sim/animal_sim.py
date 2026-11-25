"""가상 트랙 위에 동물을 세우고 실제 주행 코드를 돌려 본다 (인식은 1초에 한 번, 0.5초 늦게 들어온다고 가정).
노트북에서 돌린다. 사용: python3 tools/sim/animal_sim.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.argv = sys.argv[:1]                    # sim_drive 가 임시 폴더를 쓰게 한다
import sim_drive as D

R, T = D.R, D.T


def box_of(animal, cam, blind=0.0):
    """세계 좌표의 동물 (x, z, 키, 몸길이) -> 640x480 화면 박스 또는 None.
    blind: 이보다 가까우면 인식기가 못 잡는다고 친다 (가까워지면 몸이 화면 밖으로 잘려 인식이 끊긴다)."""
    ax, az, tall, wide = animal
    x, z, yaw = cam
    dx, dz = ax - x, az - z
    X = dx * math.cos(yaw) + dz * math.sin(yaw)
    Z = -dx * math.sin(yaw) + dz * math.cos(yaw)
    depth = Z * math.cos(T.PITCH) + math.sin(T.PITCH)
    if Z < max(0.3, blind) or depth <= 0.1:
        return None
    u = T.CX + T.FOCAL * X / depth
    v = T.CY + T.FOCAL * (-Z * math.sin(T.PITCH) + math.cos(T.PITCH)) / depth
    hh, ww = T.FOCAL * tall / depth, T.FOCAL * wide / depth
    b = [2 * (u - ww / 2), 2 * (v - hh), 2 * (u + ww / 2), 2 * v]
    if b[2] < 0 or b[0] > 640 or b[3] > 480 or b[3] < 0:
        return None
    return [int(max(0, b[0])), int(max(0, b[1])), int(min(639, b[2])), int(b[3])]


def run(name, center, animal, seconds=14.0, branches=(), start=(0.0, 0.0), blind=0.0, far=9.0, settings=None):
    """far: 이보다 멀면 인식기가 못 잡는다 (작게 보여서)."""
    r = R.Rover(dry=True)
    r.params.update({"steer": "strafe", "sign_rules": False})
    r.params.update(settings or {})
    x, z, yaw = start[0], 0.0, math.radians(start[1])
    r.handle({"action": "start", "mode": "line"})
    queue, t, seq, shots, closest, last_shot = [], 1000.0, 0, [], 9.0, -9.0
    states = []
    for step in range(int(seconds / D.DT)):
        cam = (x - math.sin(yaw) * D.CAM, z + math.cos(yaw) * D.CAM, yaw)
        img = T.render(center, cam, branches=branches)
        if t - last_shot >= 1.0:                      # 인식기: 1초에 한 번, 0.5초 뒤에 결과가 나온다
            last_shot = t
            away = math.hypot(animal[0] - cam[0], animal[1] - cam[1])
            shots.append((t + 0.5, box_of(animal, cam, blind) if away <= far else None))
        while shots and shots[0][0] <= t:
            _, b = shots.pop(0)
            r.extra = {"animals": [{"box": b, "confidence": 0.4, "looks_like": "dog"}] if b else [], "animals_time": t, "time": t}
        boxes = [(b[0] / 2, b[1] / 2, b[2] / 2, b[3] / 2) for b in r.front.blocked]
        queue.append((R.find_line(img, np, D.HSV, None, boxes), r.front.analyse(img, np)))
        found, (rows, fill) = queue[max(0, len(queue) - 1 - D.DELAY)]
        seq += 1
        r.percept = {"camera": True, "road": {"junction": "none"}, "signs": [],
                     "vision_timing": {"frame_sequence": seq, "sign_sequence": seq, "sign_age_ms": 100}}
        r.percept_at = r.frame_fresh_at = t
        F = r.front
        F.line, F.left, F.right, F.thin, F.rows, F.fill, F.at = found["line"], found["left"], found["right"], found["thin"], rows, fill, t
        command = r.line_step(t)
        forward, left, spin = D.body_motion(command)
        hx, hz = -math.sin(yaw), math.cos(yaw)
        lx, lz = -math.cos(yaw), -math.sin(yaw)
        x += (forward * hx + left * lx) * D.DT
        z += (forward * hz + left * lz) * D.DT
        yaw += spin * D.DT
        closest = min(closest, math.hypot(animal[0] - x, animal[1] - z))
        if not states or states[-1] != r.state:
            states.append(r.state)
        if not r.active:
            break
        t += D.DT
    print(f"{name}: 동물과 가장 가까웠던 거리 {closest:.2f} (몸통 반길이 {D.HALF_L}) | 끝 위치 x {x:+.1f} z {z:.1f} 방향 {math.degrees(yaw):+.0f}도"
          + ("" if r.active else f" | 끝: {r.reason}"))
    print("    " + " → ".join(states[:9]))
    return closest, (x, z, yaw), r


if __name__ == "__main__":
    line = T.straight(40.0)
    beast = (0.30, 0.45)                       # 키, 몸길이 (카메라 높이 단위)
    fork = [(0.0, 4.0), (-1.2, 6.5), (-2.4, 12.0), (-2.4, 30.0)]      # z=4 에서 왼쪽으로 갈라지는 선
    cases = [
        ("직선, 선 위에 동물 (z=7)", dict(center=line, animal=(0.0, 7.0) + beast)),
        ("직선, 선 위 동물, 3.5 안쪽에서만 잡히고 2.5 안쪽은 못 잡음", dict(center=line, animal=(0.0, 7.0) + beast, blind=2.5, far=3.5)),
        ("직선, 선 위 동물, 한 번 더 확인 설정 + 2.5 안쪽은 못 잡음", dict(center=line, animal=(0.0, 7.0) + beast, blind=2.5, far=3.5,
                                                    settings={"animal_trust": "twice"})),
        ("직선, 오른쪽 옆 멀리 (x=+1.3)", dict(center=line, animal=(1.3, 7.0) + beast)),
        ("직선, 오른쪽 옆 가까이 (x=+0.5, 몸통에 걸림)", dict(center=line, animal=(0.5, 7.0) + beast)),
        ("갈림길, 직진 선 위에 동물 (z=8)", dict(center=line, animal=(0.0, 8.0) + beast, branches=(fork,))),
        ("갈림길, 왼쪽 가지 위에 동물", dict(center=line, animal=(-2.0, 9.0) + beast, branches=(fork,))),
    ]
    for name, kw in cases:
        closest, pose, r = run(name, **kw)
        touched = closest < D.HALF_L + beast[1] / 2
        print("    =>", "동물에 닿음!!" if touched else "안 닿음")
