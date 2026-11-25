#!/usr/bin/env python3
"""tak 로버 보조 인식기: 동물(일반 사물 모델) + 손바닥(로버 코드의 PalmDetector).

rover-jazzy 도커 이미지(ultralytics + OpenCV) 안에서 실행한다.
  python3 rover_extra.py <카메라 JPEG> <출력 폴더> <COCO 모델.pt> <손바닥 모델.onnx>

- <출력 폴더>/mode.txt 에 적힌 단어에 따라 동작한다: "animals", "hand", 둘 다, 또는 "off".
- 결과는 <출력 폴더>/extra.json 에 쓴다 (그림은 그리지 않는다. 화면 표시는 웹 UI 가 한다).
- 동물: COCO 의 동물 종류(+곰인형)를 전부 "animal" 로 묶는다. 장난감 모형은 확신도가 낮아 기준을 낮게 둔다.
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
    os.makedirs(out_dir, exist_ok=True)
    mode_file = os.path.join(out_dir, "mode.txt")
    result_file = os.path.join(out_dir, "extra.json")

    from ultralytics import YOLO
    model = YOLO(coco_path)
    names = model.names
    ids = sorted(i for i, n in names.items() if n in ANIMAL_NAMES)
    from rover_autonomy.vision.hands import PalmDetector
    palm = PalmDetector(palm_path, confidence=palm_conf)
    print(f"ready: animal classes {[names[i] for i in ids]}, palm confidence {palm_conf}", flush=True)

    state = {"animals": [], "animals_time": 0.0, "animals_ms": 0, "hand": None, "hand_time": 0.0, "hand_ms": 0}
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
        run_animals = want_animals and now - last_animals >= (0.2 if "fast" in mode else animal_period)
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
                result = model.predict(frame, imgsz=size, conf=threshold, classes=ids, iou=0.5,
                                       agnostic_nms=True, verbose=False)[0]
                animals = []
                for box, confidence, cls in zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(),
                                                result.boxes.cls.tolist()):
                    x1, y1, x2, y2 = (int(v) for v in box)
                    animals.append({"label": "animal", "looks_like": names[int(cls)],
                                    "confidence": round(confidence, 3), "box": [x1, y1, x2, y2]})
                state["animals"] = animals
                state["animals_ms"] = round((time.time() - started) * 1000)
                state["animals_time"] = round(time.time(), 3)
        if not want_hand:
            state["hand"] = None
        if not want_animals:
            state["animals"] = []
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
