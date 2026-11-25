"""가상 트랙을 로버 카메라 시점(반 크기 320x240)으로 그린다. 추적기의 부호와 크기를 검증하는 데 쓴다.
길이 단위는 카메라 높이(h = 1). 로버 좌표: x 오른쪽, z 앞."""
import math

import numpy as np

W, H = 320, 240
FOCAL, CX, CY, HORIZON = 314.0, 160.0, 120.0, 7.0
PITCH = math.atan((CY - HORIZON) / FOCAL)
LINE_W = 0.143            # 두꺼운 중앙선의 폭 (카메라 높이 단위)
THIN_W = 0.032            # 얇은 테두리 선의 폭
LANE = 0.87               # 중앙선에서 테두리까지


def ground_grid():
    """각 픽셀이 보는 바닥 점 (로버 기준 X, Z). 지평선 위는 nan."""
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    dx, dy = (u - CX) / FOCAL, (v - CY) / FOCAL
    down = dy * math.cos(PITCH) + math.sin(PITCH)
    forward = -dy * math.sin(PITCH) + math.cos(PITCH)
    t = np.where(down > 1e-3, 1.0 / np.maximum(down, 1e-3), np.nan)
    return t * dx, t * forward


GX, GZ = ground_grid()


def dist_to_polyline(px, pz, pts):
    best = np.full(px.shape, 1e9, dtype=np.float32)
    for (x1, z1), (x2, z2) in zip(pts[:-1], pts[1:]):
        vx, vz = x2 - x1, z2 - z1
        L2 = vx * vx + vz * vz
        t = np.clip(((px - x1) * vx + (pz - z1) * vz) / L2, 0, 1)
        d = np.hypot(px - (x1 + t * vx), pz - (z1 + t * vz))
        best = np.minimum(best, d)
    return best


def offset_polyline(pts, d):
    """폴리라인을 왼쪽(+)으로 d 만큼 옮긴 것 (모서리는 단순 처리)."""
    out = []
    for i, (x, z) in enumerate(pts):
        a = pts[max(0, i - 1)]
        b = pts[min(len(pts) - 1, i + 1)]
        tx, tz = b[0] - a[0], b[1] - a[1]
        n = math.hypot(tx, tz)
        out.append((x - tz / n * d, z + tx / n * d))
    return out


def render(center, pose, borders=True, branches=(), noise=0.0, rng=None):
    """center: 세계 좌표의 중앙선 폴리라인 [(x, z)...]. pose: (x, z, yaw) 로버(카메라) 위치와 방향(rad, + = 반시계).
    RGB int16 화면을 돌려준다 (어두운 매트 위 노란 선)."""
    x0, z0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    # 로버 기준 바닥 점 -> 세계 좌표. yaw + 는 반시계(왼쪽으로 회전).
    wx = x0 + GX * c - GZ * s
    wz = z0 + GX * s + GZ * c
    yellow = dist_to_polyline(wx, wz, center) <= LINE_W / 2
    for branch in branches:
        yellow |= dist_to_polyline(wx, wz, branch) <= LINE_W / 2
    if borders:
        for side in (LANE, -LANE):
            yellow |= dist_to_polyline(wx, wz, offset_polyline(center, side)) <= THIN_W / 2
    yellow &= ~np.isnan(GX)
    img = np.empty((H, W, 3), dtype=np.int16)
    img[...] = (60, 66, 82)                       # 매트
    img[np.isnan(GX)] = (150, 120, 80)            # 지평선 위 (벽)
    img[yellow] = (250, 215, 40)
    if noise and rng is not None:
        img = np.clip(img + rng.normal(0, noise, img.shape), 0, 255).astype(np.int16)
    return img


def straight(length=12.0):
    return [(0.0, -2.0), (0.0, length)]


def corner(z_turn=4.0, direction=-1, radius=0.8, after=6.0):
    """z_turn 에서 direction(-1 왼쪽, +1 오른쪽)으로 90도 도는 길 (둥근 모서리)."""
    pts = [(0.0, -2.0), (0.0, z_turn - radius)]
    for i in range(1, 9):
        a = math.pi / 2 * i / 8
        pts.append((direction * radius * (1 - math.cos(a)), z_turn - radius + radius * math.sin(a)))
    pts.append((direction * (radius + after), z_turn))
    return pts
