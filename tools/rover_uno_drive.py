#!/usr/bin/env python3
"""tak 로버: 우노(L298 펌웨어 rover_uno_l298)로 바퀴를 굴리는 도구. Pi 에서 실행.

  python3 rover_uno_drive.py wheels [pwm] [초]      # 모터 0,1,2,3 을 차례로 한 개씩 (방향 확인용)
  python3 rover_uno_drive.py pair <왼> <오> [초]    # 왼쪽/오른쪽 쌍을 주어진 PWM(-255..255)으로
  python3 rover_uno_drive.py mec <전진> <왼쪽> <반시계> [pwm] [초]   # 메카넘 이동 시험 (각 -1..1)
  python3 rover_uno_drive.py dir a b c d            # 모터 0..3 방향 부호(+1/-1) 저장
  python3 rover_uno_drive.py teleop [--speed 150]   # WASD 직접 조종 + 입력 기록 (~/rover_runs)
  python3 rover_uno_drive.py replay [파일]          # 기록한 조종을 그대로 재생 (기본: 마지막 기록)
  python3 rover_uno_drive.py follow [옵션]          # 카메라 라인 추종
      --secs 4       주행 시간(초). 0 이면 Ctrl+C 로 멈출 때까지
      --base 130     직진 PWM
      --kp 1.0       조향 세기 (오차 1.0 에서 좌우 차이 = base * kp)
      --ki 0.0       오차 누적 보정 (한쪽으로 계속 치우칠 때)
      --tmax 1.5     조향 상한. 작게 주면(예 0.4) 곡선을 따라 꺾지 않고 거의 직진만 한다
      --slow 0.45    오차가 클 때 줄일 속도 비율
      --coast 0.5    선이 끊긴 뒤 그대로 더 직진하는 시간(초)
      --pivot 0      그 다음 제자리 회전하는 PWM (0 = 회전 없이 정지). 선이 다시 보일 때까지 돈다
      --search 3.5   제자리 회전으로 선을 찾는 최대 시간(초)
      --turn right   회전 방향을 알 수 없을 때 쓸 기본 방향 (left/right)
      --find right   출발할 때 선이 안 보이면 그 방향으로 제자리 회전하며 찾는다
      --strafe 0     메카넘 게걸음 보정 세기 (0 이면 좌우 속도차로만 조향)
      --min-conf 0.5 중앙선 신뢰도 하한
      --state URL    대시보드 상태 주소 (기본 http://127.0.0.1:8080/state)
      --dry          모터 명령을 보내지 않고 계산값만 출력

모터 번호: 0 = 왼쪽 A(IN1/IN2), 1 = 왼쪽 B(IN3/IN4), 2 = 오른쪽 A, 3 = 오른쪽 B.
라인 추종은 Pi 의 최신 rover 코드가 내놓는 인식 결과(대시보드 /state 의 road)를 읽어서 쓴다.
펌웨어는 명령이 0.3초 끊기면 스스로 멈춘다. 이 프로그램도 끝날 때 항상 정지 명령을 보낸다.
"""
import json
import sys
import time
import urllib.request

import serial

PORT = "/dev/ttyACM0"
NAMES = ["0 왼쪽A(IN1/IN2)", "1 왼쪽B(IN3/IN4)", "2 오른쪽A(IN1/IN2)", "3 오른쪽B(IN3/IN4)"]
# 2026-11-25 실측: 모터 0 = 왼쪽 뒤, 1 = 왼쪽 앞, 2 = 오른쪽 앞, 3 = 오른쪽 뒤 (방향 부호 -1 -1 +1 +1 은 펌웨어에 저장)
WHEEL_ORDER = ["RL", "FL", "FR", "RR"]


def clamp(value, low, high):
    return max(low, min(high, value))


def mecanum(vx, vy, w):
    """vx 전진, vy 왼쪽, w 반시계 (각 -1..1) -> 모터 0..3 순서의 비율."""
    mix = {"FL": vx - vy - w, "FR": vx + vy + w, "RL": vx + vy - w, "RR": vx - vy + w}
    top = max(1.0, max(abs(v) for v in mix.values()))
    return [mix[n] / top for n in WHEEL_ORDER]


class Uno:
    def __init__(self, port=PORT):
        self.s = serial.Serial(port, 115200, timeout=0.2, write_timeout=0.5)
        deadline = time.time() + 4.0
        seen = b""
        while time.time() < deadline and b"L298 READY" not in seen:
            seen += self.s.read(64)
        if b"L298 READY" not in seen:
            self.s.close()
            raise SystemExit(f"우노가 L298 펌웨어로 응답하지 않습니다: {seen!r}")

    def send(self, text):
        self.s.write((text + "\n").encode("ascii"))

    def query(self, text, wait=0.4):
        self.s.reset_input_buffer()
        self.send(text)
        time.sleep(wait)
        return self.s.read(200).decode("ascii", errors="replace").strip()

    def stop(self):
        for _ in range(3):
            try:
                self.send("S")
            except Exception:
                pass
            time.sleep(0.03)

    def close(self):
        self.stop()
        self.s.close()


def hold(uno, command, seconds):
    """watchdog(0.3초)보다 자주 같은 명령을 보내며 seconds 동안 유지."""
    end = time.time() + seconds
    while time.time() < end:
        uno.send(command)
        time.sleep(0.08)
    uno.stop()


def cmd_wheels(args):
    pwm = int(args[0]) if args else 150
    secs = float(args[1]) if len(args) > 1 else 0.8
    pwm = clamp(pwm, -255, 255)
    secs = clamp(secs, 0.1, 3.0)
    uno = Uno()
    try:
        print("status:", uno.query("?"), flush=True)
        for i in range(4):
            values = [0, 0, 0, 0]
            values[i] = pwm
            print(f"모터 {NAMES[i]} -> PWM {pwm:+d} 로 {secs}초", flush=True)
            hold(uno, "W " + " ".join(map(str, values)), secs)
            time.sleep(1.2)
    finally:
        uno.close()
        print("정지", flush=True)


def cmd_pair(args):
    left, right = int(args[0]), int(args[1])
    secs = clamp(float(args[2]) if len(args) > 2 else 1.0, 0.1, 5.0)
    uno = Uno()
    try:
        print(f"왼쪽 {left:+d} / 오른쪽 {right:+d} 로 {secs}초", flush=True)
        hold(uno, f"M {clamp(left, -255, 255)} {clamp(right, -255, 255)}", secs)
    finally:
        uno.close()
        print("정지", flush=True)


def cmd_mec(args):
    vx, vy, w = float(args[0]), float(args[1]), float(args[2])
    pwm = clamp(int(args[3]) if len(args) > 3 else 150, 0, 255)
    secs = clamp(float(args[4]) if len(args) > 4 else 1.0, 0.1, 5.0)
    values = [int(round(r * pwm)) for r in mecanum(vx, vy, w)]
    uno = Uno()
    try:
        print(f"전진 {vx:+.1f} 왼쪽 {vy:+.1f} 반시계 {w:+.1f} -> 모터 {values} 로 {secs}초", flush=True)
        hold(uno, "W " + " ".join(map(str, values)), secs)
    finally:
        uno.close()
        print("정지", flush=True)


def cmd_dir(args):
    signs = [int(a) for a in args[:4]]
    if len(signs) != 4 or any(s not in (1, -1) for s in signs):
        raise SystemExit("dir 는 +1/-1 네 개가 필요합니다")
    uno = Uno()
    try:
        print(uno.query("D " + " ".join(map(str, signs))), flush=True)
        print("status:", uno.query("?"), flush=True)
    finally:
        uno.close()


def option(args, name, default, kind=float):
    if name in args:
        return kind(args[args.index(name) + 1])
    return default


def fetch(url):
    with urllib.request.urlopen(url, timeout=0.5) as reply:
        return json.load(reply)


def cmd_follow(args):
    secs = option(args, "--secs", 4.0)
    base = clamp(int(option(args, "--base", 130)), 0, 255)
    kp = option(args, "--kp", 1.0)
    ki = option(args, "--ki", 0.0)
    tmax = option(args, "--tmax", 1.5)
    slow = clamp(option(args, "--slow", 0.45), 0.0, 0.9)
    pivot = clamp(int(option(args, "--pivot", 0)), 0, 255)
    search = option(args, "--search", 3.5)
    coast = option(args, "--coast", 0.5)
    default_turn = option(args, "--turn", "", str)
    find = option(args, "--find", "", str)
    strafe = option(args, "--strafe", 0.0)
    min_conf = option(args, "--min-conf", 0.5)
    url = option(args, "--state", "http://127.0.0.1:8080/state", str)
    dry = "--dry" in args
    uno = None if dry else Uno()

    last_seq = None
    fresh_at = 0.0
    start = time.time()
    last_print = 0.0
    reason = "time limit"
    filtered = None        # 부드럽게 한 오차
    integral = 0.0
    last_good = None       # 선이 보이던 마지막 오차
    valid_at = None
    lost_at = None
    seen = False
    hint = 0               # 인식기가 알려준 길의 방향 (-1 왼쪽, +1 오른쪽)
    hint_at = None
    searching = False
    side = 0
    regain = 0
    if find in ("left", "right"):
        # 출발할 때 선이 안 보이면 이 방향으로 제자리 회전하며 찾는다
        seen = True
        last_good = -0.3 if find == "left" else 0.3
        lost_at = start - coast - 0.01
    try:
        while secs == 0 or time.time() - start < secs:
            now = time.time()
            error = conf = None
            junction = "none"
            why = ""
            try:
                state = fetch(url)
            except Exception as problem:
                state = None
                why = f"state unavailable ({type(problem).__name__})"
            if state is not None:
                road = state.get("road") or {}
                seq = (state.get("vision_timing") or {}).get("frame_sequence")
                if seq is not None and seq != last_seq:
                    last_seq = seq
                    fresh_at = now
                conf = road.get("confidence") or 0.0
                error = road.get("error")
                junction = road.get("junction") or "none"
                if not state.get("camera"):
                    why = "camera stale"
                elif now - fresh_at > 0.5:
                    why = "vision stale"
                elif error is None or conf < min_conf:
                    why = f"line lost (conf {conf:.2f})"

            left = right = 0
            slide = 0.0
            mode = "stop"
            if not why and searching and (regain < 1 or abs(error) > 0.7):
                # 회전 중에 선이 보이기 시작함: 한 번 더 확인될 때까지 계속 돈다
                regain += 1
                left, right = side * pivot, -side * pivot
                mode = "search"
            elif not why:
                dt = clamp(now - valid_at, 0.0, 0.3) if valid_at is not None else 0.066
                valid_at = now
                if filtered is None or lost_at is not None:
                    filtered = error          # 처음이거나 선을 다시 찾은 직후
                    integral = 0.0
                else:
                    filtered += 0.6 * (error - filtered)
                lost_at = None
                searching = False
                regain = 0
                seen = True
                last_good = filtered
                if junction in ("left", "right"):
                    hint = -1 if junction == "left" else 1
                    hint_at = now
                integral = clamp(integral + filtered * dt, -0.4, 0.4)
                turn = clamp(kp * filtered + ki * integral, -tmax, tmax)
                mode = "follow"
                # 오차가 크면 감속. 조향이 1 을 넘으면 안쪽 바퀴가 거꾸로 돈다
                speed = base * (1 - slow * min(1.0, abs(filtered) / 0.4))
                left = int(round(clamp(speed * (1 + turn), -255, 255)))
                right = int(round(clamp(speed * (1 - turn), -255, 255)))
                slide = strafe * filtered * speed    # +면 오른쪽으로 게걸음 (메카넘)
            else:
                valid_at = None
                regain = 0
                if lost_at is None:
                    lost_at = now
                lost_for = now - lost_at
                line_gone = seen and last_good is not None and why.startswith("line lost")
                can_search = line_gone and pivot
                if line_gone and lost_for <= coast and not searching:
                    # 선이 끊긴 직후: 조금 더 직진 (갈림길 가로선을 넘거나, 코너 끝까지 들어감)
                    left = right = int(round(base * 0.8))
                    mode = "coast"
                elif can_search and lost_for <= coast + search:
                    if not searching:
                        searching = True
                        if hint_at is not None and now - hint_at < 4.0:
                            side = hint                         # 인식기가 알려준 방향
                        elif abs(last_good) >= 0.04 or default_turn not in ("left", "right"):
                            side = 1 if last_good > 0 else -1   # 선이 마지막에 치우쳐 있던 쪽
                        else:
                            side = -1 if default_turn == "left" else 1
                    left, right = side * pivot, -side * pivot   # +: 시계 방향(오른쪽) 제자리 회전
                    mode = "search"
                elif lost_for > (coast + search if can_search else max(coast, 1.0)) and now - start > 1.5:
                    reason = why
                    break

            if mode == "follow" and strafe:
                # 메카넘: 오른쪽 게걸음 = 왼쪽앞+ 오른쪽뒤+ / 오른쪽앞- 왼쪽뒤-
                wheels = {"RL": left - slide, "FL": left + slide, "FR": right - slide, "RR": right + slide}
                command = "W " + " ".join(str(int(round(clamp(wheels[n], -255, 255)))) for n in WHEEL_ORDER)
            else:
                command = f"M {left} {right}"
            if uno is not None:
                uno.send(command)
            if now - last_print >= 0.25:
                last_print = now
                detail = why or f"err {error:+.3f} conf {conf:.2f} {junction}"
                print(f"{now - start:5.1f}s  {mode:7s} L {left:4d}  R {right:4d}  {detail}", flush=True)
            time.sleep(max(0.0, 0.066 - (time.time() - now)))
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    finally:
        if uno is not None:
            uno.close()
        print(f"정지 ({reason})", flush=True)


RUN_DIR = "/home/takyeji/rover_runs"
TELEOP_KEYS = {  # 키: (전진, 왼쪽, 반시계)
    "w": (1, 0, 0), "s": (-1, 0, 0),
    "a": (0, 0, 1), "d": (0, 0, -1),
    "q": (0, 1, 0), "e": (0, -1, 0),
}
TELEOP_NAMES = {"w": "전진", "s": "후진", "a": "좌회전", "d": "우회전", "q": "왼쪽 게걸음", "e": "오른쪽 게걸음"}


def teleop_command(key, speed, turn_pwm, strafe_pwm):
    if key is None:
        return "S"
    vx, vy, w = TELEOP_KEYS[key]
    pwm = turn_pwm if w else strafe_pwm if vy else speed
    return "W " + " ".join(str(int(round(r * pwm))) for r in mecanum(vx, vy, w))


def cmd_teleop(args):
    """WASD 로 직접 조종하면서 보낸 명령을 시간과 함께 기록한다."""
    import os
    import select
    import termios
    import tty

    speed = clamp(int(option(args, "--speed", 150)), 0, 255)
    turn_pwm = clamp(int(option(args, "--turn-pwm", 220)), 0, 255)
    strafe_pwm = clamp(int(option(args, "--strafe-pwm", 200)), 0, 255)
    hold = option(args, "--hold", 0.5)
    latch = "--latch" in args                  # 기본: 누를 때마다 잠깐만 움직임 (--latch 면 계속 움직임)
    limit = option(args, "--max-seg", 8.0)      # 한 동작이 이 시간을 넘으면 스스로 정지
    os.makedirs(RUN_DIR, exist_ok=True)
    name = time.strftime("teleop_%Y%m%d_%H%M%S.jsonl")
    path = option(args, "--out", os.path.join(RUN_DIR, name), str)
    uno = Uno()
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    events = []
    current = "S"
    active = None
    last_key = 0.0
    start = None
    print("w 전진  s 후진  a 좌회전  d 우회전  q 왼쪽 게걸음  e 오른쪽 게걸음", flush=True)
    if latch:
        print(f"키를 한 번 누르면 계속 움직입니다.  스페이스 = 정지,  x = 끝내고 저장  ({limit:.0f}초 넘으면 자동 정지)", flush=True)
    else:
        print(f"키를 누를 때마다 {hold:.1f}초씩 움직입니다 (계속 누르면 이어서 움직임).  스페이스 = 정지,  x = 끝내고 저장", flush=True)
    print(f"속도: 직진 {speed}  회전 {turn_pwm}  게걸음 {strafe_pwm}   (첫 키를 누르는 순간부터 기록)", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            now = time.time()
            if ready:
                key = sys.stdin.read(1).lower()
                if key in ("x", "\x03"):
                    break
                if key == " ":
                    active = None
                elif key in TELEOP_KEYS:
                    active = key
                    last_key = now
                    if start is None:
                        start = now
            if active is not None and now - last_key > (limit if latch else hold):
                active = None
            command = teleop_command(active, speed, turn_pwm, strafe_pwm)
            if command != current:
                current = command
                if start is not None:
                    events.append({"t": round(now - start, 3), "cmd": command})
                    label = TELEOP_NAMES.get(active, "정지")
                    sys.stdout.write(f"\r{now - start:7.2f}s  {label:12s}\n")
                    sys.stdout.flush()
            uno.send(command)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        uno.close()
        if start is not None:
            if not events or events[-1]["cmd"] != "S":
                events.append({"t": round(time.time() - start, 3), "cmd": "S"})
            with open(path, "w") as handle:
                for event in events:
                    handle.write(json.dumps(event) + "\n")
            last = os.path.join(RUN_DIR, "last.jsonl")
            with open(last, "w") as handle:
                for event in events:
                    handle.write(json.dumps(event) + "\n")
            total = events[-1]["t"]
            print(f"\n정지. 기록 {len(events)}개, {total:.1f}초 -> {path}", flush=True)
        else:
            print("\n정지. 기록 없음", flush=True)


def cmd_replay(args):
    """teleop 으로 기록한 명령을 같은 시간 간격으로 다시 보낸다."""
    import os

    path = args[0] if args and not args[0].startswith("--") else os.path.join(RUN_DIR, "last.jsonl")
    rate = option(args, "--rate", 1.0)        # 1 보다 크면 각 구간을 그만큼 길게
    with open(path) as handle:
        events = [json.loads(line) for line in handle if line.strip()]
    if not events:
        raise SystemExit("기록이 비어 있습니다")
    total = events[-1]["t"] * rate
    print(f"재생: {path}  명령 {len(events)}개, {total:.1f}초   (Ctrl+C 로 정지)", flush=True)
    uno = Uno()
    if "--wait" in args:
        try:
            input("로봇을 출발 위치에 같은 방향으로 놓고, 준비되면 엔터를 누르세요 (취소: Ctrl+C): ")
        except KeyboardInterrupt:
            uno.close()
            print("\n취소", flush=True)
            return
    current = "S"
    index = 0
    start = time.time()
    try:
        while True:
            now = time.time() - start
            while index < len(events) and events[index]["t"] * rate <= now:
                current = events[index]["cmd"]
                print(f"{now:7.2f}s  {current}", flush=True)
                index += 1
            if index >= len(events):
                break
            uno.send(current)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Ctrl+C", flush=True)
    finally:
        uno.close()
        print("정지", flush=True)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    command, args = sys.argv[1], sys.argv[2:]
    handlers = {"wheels": cmd_wheels, "pair": cmd_pair, "mec": cmd_mec, "dir": cmd_dir, "follow": cmd_follow,
                "teleop": cmd_teleop, "replay": cmd_replay}
    if command not in handlers:
        print(__doc__)
        return 2
    handlers[command](args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
