#!/usr/bin/env python3
"""Arduino Uno (ATmega328P + Optiboot) 플래시 도구 - avrdude 없이 pyserial 만 사용.

  python3 uno_flash.py listen [초]            # 리셋 후 우노가 보내는 글자 보기 (HELLO 도 보냄)
  python3 uno_flash.py dump   out.bin          # 현재 들어 있는 프로그램을 읽어 저장 (쓰기 없음)
  python3 uno_flash.py upload sketch.hex       # HEX 올리고 되읽어 검증
  python3 uno_flash.py restore backup.bin      # dump 로 받은 bin 을 다시 올림
옵션: --port /dev/ttyACM0
STK500v1 (부트로더 115200 baud, 페이지 128 바이트). 부트로더 영역(0x7E00~)은 건드리지 않는다.
"""
import sys
import time

import serial

PAGE = 128
APP_END = 0x7E00  # Optiboot 512 B 부트로더 앞까지
SYNC, EOP, INSYNC, OK = 0x30, 0x20, 0x14, 0x10


class Uno:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.3, write_timeout=1.0)

    def reset(self):
        self.s.dtr = False
        self.s.rts = False
        time.sleep(0.25)
        self.s.dtr = True
        self.s.rts = True
        time.sleep(0.05)
        self.s.reset_input_buffer()

    def cmd(self, payload, reply_len=0):
        self.s.write(bytes(payload) + bytes([EOP]))
        need = reply_len + 2
        data = b""
        deadline = time.time() + 2.0
        while len(data) < need and time.time() < deadline:
            data += self.s.read(need - len(data))
        if len(data) != need or data[0] != INSYNC or data[-1] != OK:
            raise IOError(f"bootloader reply {data.hex() or 'none'} to command 0x{payload[0]:02x}")
        return data[1:-1]

    def enter(self):
        """리셋 후 부트로더와 동기화. 실패하면 몇 번 다시 리셋."""
        for attempt in range(4):
            self.reset()
            for _ in range(8):
                self.s.write(bytes([SYNC, EOP]))
                got = self.s.read(2)
                if got == bytes([INSYNC, OK]):
                    self.s.reset_input_buffer()
                    sig = self.cmd([0x75], 3)
                    if sig != b"\x1e\x95\x0f":
                        raise IOError(f"unexpected chip signature {sig.hex()} (ATmega328P = 1e950f)")
                    self.cmd([0x50])
                    return
                self.s.reset_input_buffer()
        raise IOError("no bootloader sync (is this an Uno with the stock bootloader?)")

    def leave(self):
        try:
            self.cmd([0x51])
        except IOError:
            pass

    def addr(self, byte_address):
        word = byte_address // 2
        self.cmd([0x55, word & 0xFF, (word >> 8) & 0xFF])

    def read_page(self, byte_address):
        self.addr(byte_address)
        return self.cmd([0x74, 0x00, PAGE, ord("F")], PAGE)

    def write_page(self, byte_address, data):
        self.addr(byte_address)
        self.cmd([0x64, 0x00, PAGE, ord("F")] + list(data))

    def close(self):
        self.s.close()


def parse_hex(path):
    image = {}
    upper = 0
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line[0] != ":":
                raise ValueError("not an Intel HEX file")
            raw = bytes.fromhex(line[1:])
            if sum(raw) & 0xFF:
                raise ValueError("HEX checksum error")
            count, address, kind = raw[0], (raw[1] << 8) | raw[2], raw[3]
            body = raw[4:4 + count]
            if kind == 0:
                for i, value in enumerate(body):
                    image[upper + address + i] = value
            elif kind == 1:
                break
            elif kind == 4:
                upper = ((body[0] << 8) | body[1]) << 16
            elif kind == 2:
                upper = ((body[0] << 8) | body[1]) << 4
    if not image:
        raise ValueError("empty HEX file")
    size = max(image) + 1
    if size > APP_END:
        raise ValueError(f"sketch too large: {size} bytes")
    data = bytearray([0xFF] * size)
    for address, value in image.items():
        data[address] = value
    return bytes(data)


def pad(data):
    extra = (-len(data)) % PAGE
    return data + bytes([0xFF] * extra)


def do_dump(uno, path):
    uno.enter()
    out = bytearray()
    for address in range(0, APP_END, PAGE):
        out += uno.read_page(address)
    uno.leave()
    used = len(out)
    while used and out[used - 1] == 0xFF:
        used -= 1
    with open(path, "wb") as handle:
        handle.write(out[:used])
    print(f"dumped {used} bytes -> {path}")
    return bytes(out[:used])


def do_program(uno, data):
    data = pad(data)
    uno.enter()
    for address in range(0, len(data), PAGE):
        uno.write_page(address, data[address:address + PAGE])
    bad = 0
    for address in range(0, len(data), PAGE):
        if uno.read_page(address) != data[address:address + PAGE]:
            bad += 1
    uno.leave()
    if bad:
        raise IOError(f"verify FAILED on {bad} page(s)")
    print(f"uploaded and verified {len(data)} bytes ({len(data) // PAGE} pages)")


def do_listen(uno, seconds):
    uno.reset()
    start = time.time()
    data = b""
    sent = 0
    while time.time() - start < seconds:
        data += uno.s.read(256)
        if time.time() - start > 2.2 + sent * 0.6 and sent < 3:
            uno.s.write(b"HELLO\n")
            sent += 1
    print(f"{len(data)} bytes after reset (HELLO sent {sent}x):")
    print(data.decode("ascii", errors="replace") if data else "(nothing)")


def read_lines(uno, until, lines, state):
    while time.time() < until:
        state["buf"] += uno.s.read(256)
        while b"\n" in state["buf"]:
            raw, _, rest = state["buf"].partition(b"\n")
            state["buf"] = rest
            lines.append(raw.decode("ascii", errors="replace").strip())


def do_ra2(uno, seconds):
    """리셋 후 RA2 핸드셰이크(HELLO, CFG)를 해 보고 텔레메트리를 요약한다."""
    uno.reset()
    state = {"buf": b""}
    lines = []
    read_lines(uno, time.time() + 2.2, lines, state)
    print("boot  ->", lines)
    lines.clear()
    uno.s.write(b"HELLO\n")
    read_lines(uno, time.time() + 0.6, lines, state)
    print("HELLO ->", lines)
    lines.clear()
    uno.s.write(b"CFG 1 1 1 1\n")
    read_lines(uno, time.time() + 0.4 + seconds, lines, state)
    telemetry = [l for l in lines if l.startswith("S ")]
    other = [l for l in lines if not l.startswith("S ")]
    print("CFG   ->", other[:3])
    print(f"telemetry: {len(telemetry)} lines in {0.4 + seconds:.1f}s")
    for l in telemetry[:2] + telemetry[-2:]:
        print("   ", l)


def do_send(uno, text, seconds):
    """리셋 후 부팅을 기다렸다가 한 줄을 보내고 응답을 그대로 보여준다 (RAW, MAP 등)."""
    uno.reset()
    state = {"buf": b""}
    lines = []
    read_lines(uno, time.time() + 2.2, lines, state)
    lines.clear()
    for part in text.split(";"):
        uno.s.write(part.strip().encode("ascii") + b"\n")
        time.sleep(0.15)
    end = time.time() + seconds
    shown = 0
    while time.time() < end:
        read_lines(uno, min(end, time.time() + 0.5), lines, state)
        for l in lines[shown:]:
            print(l, flush=True)
        shown = len(lines)


def main():
    args = sys.argv[1:]
    port = "/dev/ttyACM0"
    if "--port" in args:
        i = args.index("--port")
        port = args[i + 1]
        del args[i:i + 2]
    if not args:
        print(__doc__)
        return 2
    uno = Uno(port)
    try:
        if args[0] == "listen":
            do_listen(uno, float(args[1]) if len(args) > 1 else 5.0)
        elif args[0] == "dump":
            do_dump(uno, args[1])
        elif args[0] == "upload":
            do_program(uno, parse_hex(args[1]))
        elif args[0] == "restore":
            with open(args[1], "rb") as handle:
                do_program(uno, handle.read())
        elif args[0] == "ra2":
            do_ra2(uno, float(args[1]) if len(args) > 1 else 2.0)
        elif args[0] == "send":
            do_send(uno, args[1], float(args[2]) if len(args) > 2 else 3.0)
        else:
            print(__doc__)
            return 2
    finally:
        uno.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
