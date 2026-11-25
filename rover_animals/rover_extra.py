#!/usr/bin/env python3
"""tak 로버 보조 인식기: 동물(일반 사물 모델) + 손바닥(로버 코드의 PalmDetector).

rover-jazzy 도커 이미지(ultralytics + OpenCV) 안에서 실행한다.
  python3 rover_extra.py <카메라 JPEG> <출력 폴더> <COCO 모델.pt> <손바닥 모델.onnx>

- <출력 폴더>/mode.txt 에 적힌 단어에 따라 동작한다: "animals", "hand", 둘 다, 또는 "off".
- 결과는 <출력 폴더>/extra.json 에 쓴다 (그림은 그리지 않는다. 화면 표시는 웹 UI 가 한다).
- 동물: COCO 의 동물 종류(+곰인형)를 전부 "animal" 로 묶는다. 장난감 모형은 확신도가 낮아 기준을 낮게 둔다.
  작은 피규어는 화면 높이의 16~20% 로 보일 때만 잡혀서(2026-11-25 코끼리·호랑이 피규어로 확인), 화면 전체를 줄여 보는 것과
  길 앞쪽 가운데를 줄이지 않고 잘라 보는 것(1.67배 크게 보인다)을 번갈아 한다. animals 는 방금 본 것만,
  animals_shown 은 최근 두 번(전체 + 확대)을 합친 것이다.
- 손바닥: ~/robot_ws 의 rover_autonomy.vision.hands.PalmDetector 를 그대로 쓴다.
"""
import json
import os
import sys
import time

import cv2

ANIMAL_NAMES = {"bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "teddy bear"}


def read_mode(path):
    try:
        with open(path) as handle:
            return set(handle.read().replace(",", " ").split())
    except OSError:
        return {"animals"}


def main():
    source, out_dir, coco_path, palm_path = sys.argv[1:5]
    threshold = float(os.environ.get("ANIMAL_CONF", "0.10"))
    size = int(os.environ.get("ANIMAL_IMGSZ", "384"))
    animal_period = float(os.environ.get("ANIMAL_PERIOD", "1.0"))
    palm_conf = float(os.environ.get("PALM_CONF", "0.60"))
    # 확대해서 볼 영역 (640x480 기준 x1,y1,x2,y2). 384x288 이라 모델 입력 크기 그대로다. "off" 면 쓰지 않는다
    zoom_text = os.environ.get("ANIMAL_ZOOM", "128,60,512,348")
    zoom = None if zoom_text == "off" else tuple(int(v) for v in zoom_text.split(","))
    os.makedirs(out_dir, exist_ok=True)
    mode_file = os.path.join(out_dir, "mode.txt")
    result_file = os.path.join(out_dir, "extra.json")

    from ultralytics import YOLO
    # 같은 모델을 ncnn 으로 바꿔 둔 것(<모델 이름>_ncnn_model 폴더)이 있으면 그것을 쓴다. Pi 4 에서 .pt 보다 빠르다.
    # 만드는 법: 컨테이너 안에서 YOLO("yolo11n_coco.pt").export(format="ncnn", imgsz=(288, 384))
    fast_path = os.path.splitext(coco_path)[0] + "_ncnn_model"
    model, kind = None, "pt"
    if os.path.isdir(fast_path) and os.environ.get("ANIMAL_NCNN", "1") != "0":
        try:
            import numpy
            model = YOLO(fast_path, task="detect")
            model.predict(numpy.zeros((288, 384, 3), dtype=numpy.uint8), imgsz=(288, 384), verbose=False)
            size, kind = (288, 384), "ncnn"        # ncnn 모델은 입력 크기가 고정이다 (세로, 가로)
        except Exception as problem:
            print(f"ncnn model failed, using the .pt model: {problem!r}", flush=True)
            model = None
    if model is None:
        model = YOLO(coco_path)
    names = model.names
    ids = sorted(i for i, n in names.items() if n in ANIMAL_NAMES)
    from rover_autonomy.vision.hands import PalmDetector
    palm = PalmDetector(palm_path, confidence=palm_conf)
    print(f"ready: animal model {kind} {size}, classes {[names[i] for i in ids]}, palm confidence {palm_conf}", flush=True)

    state = {"animals": [], "animals_shown": [], "animals_view": "full", "animals_time": 0.0, "animals_ms": 0,
             "hand": None, "hand_time": 0.0, "hand_ms": 0}
    passes = {"full": ([], 0.0), "zoom": ([], 0.0)}       # 보는 방식별 마지막 결과와 시각
    turn = 0
    last_stamp = None
    last_animals = 0.0
    last_print = 0.0
    last_write = 0.0
    width, height = 640, 480
    while True:
        mode = read_mode(mode_file)
        now = time.time()
        off = "off" in mode
        want_hand = "hand" in mode and not off
        want_animals = "animals" in mode and not off
        # "fast" 가 같이 적혀 있으면(자율 주행 중) 쉬지 않고 돌린다
        # 쉬는 시간은 한 번 도는 시간에 더해진다 (last_animals 는 시작 시각): 선 추적과 카메라에 CPU 를 남겨 둔다
        run_animals = want_animals and now - last_animals >= (0.9 if "fast" in mode else animal_period)
        frame = None
        if want_hand or run_animals:            # 필요할 때만 화면을 읽는다
            try:
                stamp = os.stat(source).st_mtime_ns
            except FileNotFoundError:
                stamp = None
            if stamp is not None and stamp != last_stamp:
                frame = cv2.imread(source)
                if frame is not None:
                    last_stamp = stamp
        if frame is not None:
            height, width = frame.shape[:2]
            if want_hand:
                started = time.time()
                found = palm.detect(frame)
                state["hand_ms"] = round((time.time() - started) * 1000)
                state["hand_time"] = round(time.time(), 3)
                if found is None:
                    state["hand"] = None
                else:
                    x, y, w, h = found.box
                    state["hand"] = {"center_x": round(found.center_x, 3), "area": round(found.area, 4),
                                     "confidence": round(found.confidence, 3), "box": [x, y, x + w, y + h]}
            if run_animals:
                last_animals = now
                started = time.time()
                turn += 1
                view = "zoom" if zoom is not None and turn % 2 == 1 else "full"
                ox, oy = (zoom[0], zoom[1]) if view == "zoom" else (0, 0)
                picture = frame[zoom[1]:zoom[3], zoom[0]:zoom[2]] if view == "zoom" else frame
                result = model.predict(picture, imgsz=size, conf=threshold, classes=ids, iou=0.5,
                                       agnostic_nms=True, verbose=False)[0]
                animals = []
                for box, confidence, cls in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                                                result.boxes.cls.tolist()):
                    x1, y1, x2, y2 = (int(v) for v in box)
                    if y2 - y1 >= 0.92 * picture.shape[0]:
                        continue      # 본 화면의 높이를 다 덮는 박스는 피규어가 아니다 (벽, 바닥, 사람 다리를 잘못 읽은 것)
                    animals.append({"label": "animal", "looks_like": names[int(cls)], "view": view,
                                    "confidence": round(confidence, 3), "box": [x1 + ox, y1 + oy, x2 + ox, y2 + oy]})
                passes[view] = (animals, time.time())
                shown = list(animals)
                other, other_time = passes["full" if view == "zoom" else "zoom"]
                if time.time() - other_time < 3.0:
                    for extra in other:               # 다른 방식으로 방금 본 것 중 겹치지 않는 것을 더한다
                        ax1, ay1, ax2, ay2 = extra["box"]
                        apart = True
                        for seen in animals:
                            bx1, by1, bx2, by2 = seen["box"]
                            iw, ih = min(ax2, bx2) - max(ax1, bx1), min(ay2, by2) - max(ay1, by1)
                            if iw > 0 and ih > 0 and iw * ih > 0.3 * min((ax2 - ax1) * (ay2 - ay1), (bx2 - bx1) * (by2 - by1)):
                                apart = False
                        if apart:
                            shown.append(extra)
                state["animals"] = animals
                state["animals_shown"] = shown
                state["animals_view"] = view
                state["animals_ms"] = round((time.time() - started) * 1000)
                state["animals_time"] = round(time.time(), 3)
        if not want_hand:
            state["hand"] = None
        if not want_animals:
            state["animals"] = []
            state["animals_shown"] = []
        if frame is None and now - last_write < 0.5:
            time.sleep(0.02 if want_hand else 0.1)
            continue
        last_write = now
        payload = dict(state, time=round(time.time(), 3), width=width, height=height, mode=sorted(mode))
        with open(result_file + ".tmp", "w") as handle:
            json.dump(payload, handle)
        os.replace(result_file + ".tmp", result_file)
        if frame is None:
            time.sleep(0.02 if want_hand else 0.1)
        if time.time() - last_print > 5:
            last_print = time.time()
            hand = state["hand"]
            print(f"mode {sorted(mode)} animals {len(state['animals'])} ({state['animals_ms']} ms) "
                  f"hand {'yes' if hand else 'no'} ({state['hand_ms']} ms)", flush=True)


if __name__ == "__main__":
    main()
