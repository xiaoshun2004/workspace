#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纸杯检测：YOLO（COCO 'cup'）为主，HSV 浅色兜底。"""
import cv2
import numpy as np

from config import (
    DETECT_METHOD, HSV_LOWER, HSV_UPPER, MIN_AREA,
    YOLO_CLASSES, YOLO_CONF, YOLO_WEIGHTS,
)

_MODEL = None


def _get_model():
    global _MODEL
    if _MODEL is None:
        from ultralytics import YOLO
        _MODEL = YOLO(YOLO_WEIGHTS)
    return _MODEL


def detect_yolo(color, conf=YOLO_CONF, classes=YOLO_CLASSES):
    """YOLO 检测纸杯，返回 {'cx','cy','conf','method'} 或 None。"""
    model = _get_model()
    results = model.predict(cv2.cvtColor(color, cv2.COLOR_BGR2RGB),
                            conf=conf, classes=classes, verbose=False)
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        return None
    i = int(boxes.conf.argmax())  # 多目标时取置信度最高
    x1, y1, x2, y2 = boxes.xyxy[i].tolist()
    return {
        "cx": (x1 + x2) / 2.0,
        "cy": (y1 + y2) / 2.0,
        "conf": float(boxes.conf[i]),
        "method": "yolo",
    }


def detect_hsv(color, lower=HSV_LOWER, upper=HSV_UPPER, min_area=MIN_AREA):
    """HSV 浅色兜底：最大浅色连通域的中心。返回 dict 或 None。"""
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    cnt = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(cnt)
    if area < min_area:
        return None
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        return None
    return {
        "cx": M["m10"] / M["m00"],
        "cy": M["m01"] / M["m00"],
        "area": area,
        "method": "hsv",
    }


def detect_cup(color, method=DETECT_METHOD):
    """统一入口。返回 {'cx','cy',...} 或 None。"""
    if method == "yolo":
        return detect_yolo(color)
    if method == "hsv":
        return detect_hsv(color)
    raise ValueError(f"未知检测方法: {method}")


def main():
    """自测：实时显示检测结果。按 h 切换 yolo/hsv，按 q 退出。"""
    import argparse

    from camera import D405Camera

    p = argparse.ArgumentParser(description="纸杯检测自测")
    p.add_argument("--method", default=DETECT_METHOD, choices=["yolo", "hsv"])
    p.add_argument("--conf", type=float, default=YOLO_CONF,
                   help="YOLO 置信度阈值(默认 0.25, 识别不稳可降到 0.15)")
    args = p.parse_args()

    cam = D405Camera()
    cam.start()
    method = args.method
    try:
        while True:
            color, _ = cam.get_frame()
            res = (detect_yolo(color, conf=args.conf) if method == "yolo"
                   else detect_hsv(color))
            disp = color.copy()
            if res:
                cv2.circle(disp, (int(res["cx"]), int(res["cy"])), 8,
                            (0, 0, 255), 2)
                tag = f"{res['method']} ({res['cx']:.0f},{res['cy']:.0f})"
                if "conf" in res:
                    tag += f" conf={res['conf']:.2f}"
                cv2.putText(disp, tag, (int(res["cx"]) + 12, int(res["cy"]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(disp, f"method={method} [h]切换 [q]退出", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("detect", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("h"):
                method = "hsv" if method == "yolo" else "yolo"
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
