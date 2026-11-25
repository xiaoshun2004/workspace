#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 D405 出厂相机内参导出成 JSON, 供 charuco_handeye.py 使用。

用法:
    python3 dump_intrinsics.py --out ../calib/d405_intrinsics.json

注意:
  - 手眼标定图片必须用"未旋转的原始图"(与出厂内参的像素坐标系一致)。
  - 只导出彩色流内参; D405 出厂标定精度通常足够 v0 使用。
"""
import argparse
import json
from pathlib import Path

import pyrealsense2 as rs


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path,
                   default=Path(__file__).resolve().parent.parent /
                   "calib" / "d405_intrinsics.json")
    args = p.parse_args()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    profile = pipeline.start(cfg)
    try:
        for _ in range(5):
            pipeline.wait_for_frames()
        intr = (profile.get_stream(rs.stream.color)
                .as_video_stream_profile().get_intrinsics())
    finally:
        pipeline.stop()

    data = {
        "camera": "D405",
        "stream": "color",
        "width": intr.width,
        "height": intr.height,
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "model": str(intr.model),
        "dist_coeffs": list(intr.coeffs),
        "camera_matrix": [[intr.fx, 0.0, intr.ppx],
                          [0.0, intr.fy, intr.ppy],
                          [0.0, 0.0, 1.0]],
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(f"已保存 D405 彩色流内参 -> {args.out}")
    print(f"  {intr.width}x{intr.height}  fx={intr.fx:.3f} fy={intr.fy:.3f} "
          f"ppx={intr.ppx:.2f} ppy={intr.ppy:.2f}")
    print(f"  畸变({intr.model}): {data['dist_coeffs']}")


if __name__ == "__main__":
    main()
