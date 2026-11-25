#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""现场检查：D405 出流 + 保存当前帧 + YOLO/HSV 双方法检测。

用法:
  python3 check_camera.py              # 抓 10 帧, 保存图片并打印检测结果
  python3 check_camera.py --frames 3   # 少抓几帧
  python3 check_camera.py --conf 0.15  # 调低 YOLO 置信度
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from camera import D405Camera
from config import YOLO_CONF
from detect import detect_hsv, detect_yolo


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--frames", type=int, default=10)
    p.add_argument("--conf", type=float, default=YOLO_CONF)
    p.add_argument("--out", default=None)
    args = p.parse_args()

    out_dir = Path(args.out) if args.out else (
        Path(__file__).resolve().parent.parent / "data")
    out_dir.mkdir(parents=True, exist_ok=True)

    cam = D405Camera()
    cam.start()
    try:
        color = None
        for i in range(args.frames):
            c, depth = cam.get_frame()
            color = c
            nonzero = np.mean(depth > 0) * 100
            print(f"帧{i}: color={c.shape} depth={depth.shape} "
                  f"depth有效比例={nonzero:.1f}%")
    finally:
        cam.stop()

    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    print(f"图像亮度均值={gray.mean():.1f} 标准差={gray.std():.1f}")

    yolo = detect_yolo(color, conf=args.conf)
    hsv = detect_hsv(color)
    print("YOLO检测:", yolo)
    print("HSV检测:", hsv)

    # 标注图：HSV 圈浅色区域, YOLO 画框
    annotated = color.copy()
    if hsv:
        cx, cy = int(hsv["cx"]), int(hsv["cy"])
        cv2.circle(annotated, (cx, cy), 8, (0, 255, 0), 2)
        cv2.putText(annotated, f"HSV({cx},{cy})", (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if yolo:
        cx, cy = int(yolo["cx"]), int(yolo["cy"])
        cv2.circle(annotated, (cx, cy), 8, (0, 0, 255), 2)
        cv2.putText(annotated, f"YOLO conf={yolo['conf']:.2f}", (cx + 12, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    raw_path = out_dir / "check_frame.png"
    ann_path = out_dir / "check_frame_annotated.png"
    cv2.imwrite(str(raw_path), color)
    cv2.imwrite(str(ann_path), annotated)
    print(f"已保存: {raw_path}")
    print(f"已保存: {ann_path}")


if __name__ == "__main__":
    main()
