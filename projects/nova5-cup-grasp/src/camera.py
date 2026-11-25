#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RealSense D405 深度相机模块（v0 只使用 RGB 流）。"""
import cv2
import numpy as np
import pyrealsense2 as rs

from config import CAM_ROTATE


class D405Camera:
    def __init__(self, width=1280, height=720, fps=30, rotate=CAM_ROTATE):
        self.width = width
        self.height = height
        self.fps = fps
        self.rotate = rotate
        self.pipeline = None
        self.align = None

    def start(self):
        sw, sh = self.width, self.height
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, sw, sh,
                             rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, sw, sh,
                             rs.format.z16, self.fps)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        for _ in range(5):  # 预热, 等自动曝光稳定
            self.pipeline.wait_for_frames()
        if self.rotate in ("cw90", "ccw90"):
            self.width, self.height = sh, sw
        print(f"[D405] 已启动 {self.width}x{self.height}@{self.fps}")

    def get_frame(self):
        """返回 (color_bgr, depth_mm)。"""
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        color = np.asanyarray(frames.get_color_frame().get_data())
        depth = np.asanyarray(frames.get_depth_frame().get_data())
        if self.rotate == "cw90":
            color = cv2.rotate(color, cv2.ROTATE_90_CLOCKWISE)
            depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate == "ccw90":
            color = cv2.rotate(color, cv2.ROTATE_90_COUNTERCLOCKWISE)
            depth = cv2.rotate(depth, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate == "180":
            color = cv2.rotate(color, cv2.ROTATE_180)
            depth = cv2.rotate(depth, cv2.ROTATE_180)
        return color, depth

    def get_color(self):
        return self.get_frame()[0]

    def stop(self):
        if self.pipeline:
            self.pipeline.stop()
            self.pipeline = None
            print("[D405] 已停止")


if __name__ == "__main__":
    cam = D405Camera()
    cam.start()
    try:
        while True:
            color, _ = cam.get_frame()
            cv2.imshow("D405 RGB", color)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()
