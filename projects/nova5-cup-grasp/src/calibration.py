#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""2D 单应标定（像素 ↔ 机器人 XY），适用于固定俯拍 + 平面取放。"""
import json

import cv2
import numpy as np


class HomographyCalib:
    def __init__(self, image_size=None):
        self.image_size = image_size
        self.pixels = []
        self.robot_pts = []
        self.H = None

    def add_point(self, u, v, x, y):
        self.pixels.append((float(u), float(v)))
        self.robot_pts.append((float(x), float(y)))

    def compute(self):
        if len(self.pixels) < 4:
            raise ValueError(f"至少需要 4 个标定点, 当前 {len(self.pixels)} 个")
        src = np.array(self.pixels, dtype=np.float32)
        dst = np.array(self.robot_pts, dtype=np.float32)
        self.H, _ = cv2.findHomography(src, dst)
        return self.H

    def pixel_to_robot(self, u, v):
        if self.H is None:
            raise RuntimeError("尚未标定, 先调用 compute()")
        p = np.array([u, v, 1.0])
        r = self.H @ p
        return float(r[0] / r[2]), float(r[1] / r[2])

    def reproj_error(self):
        if self.H is None:
            return None
        errs = []
        for (u, v), (x, y) in zip(self.pixels, self.robot_pts):
            rx, ry = self.pixel_to_robot(u, v)
            errs.append(np.hypot(rx - x, ry - y))
        return float(np.mean(errs)), float(np.max(errs))

    def save(self, path):
        json.dump({
            "image_size": self.image_size,
            "pixels": self.pixels,
            "robot_pts": self.robot_pts,
            "H": self.H.tolist() if self.H is not None else None,
        }, open(path, "w", encoding="utf-8"), indent=2)

    @classmethod
    def load(cls, path):
        d = json.load(open(path, encoding="utf-8"))
        obj = cls(d.get("image_size"))
        obj.pixels = [tuple(map(float, p)) for p in d["pixels"]]
        obj.robot_pts = [tuple(map(float, p)) for p in d["robot_pts"]]
        if d.get("H"):
            obj.H = np.array(d["H"], dtype=np.float64)
        return obj


if __name__ == "__main__":
    calib = HomographyCalib()
    calib.add_point(100, 100, 0, 0)
    calib.add_point(1100, 100, 300, 0)
    calib.add_point(100, 500, 0, 150)
    calib.add_point(1100, 500, 300, 150)
    calib.compute()
    x, y = calib.pixel_to_robot(600, 300)
    mean, mx = calib.reproj_error()
    print(f"中心像素(600,300) -> 机器人({x:.1f},{y:.1f}) mm, 期望 (150,75)")
    print(f"标定残差: 平均 {mean:.2f}mm / 最大 {mx:.2f}mm")
