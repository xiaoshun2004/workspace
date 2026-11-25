#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ChArUco 标定板检测/标注 + 手眼标定（AX=XB）

背景:
  本机 OpenCV 5.0 已移除 cv2.aruco.calibrateCameraCharuco 和
  cv2.calibrateHandEye, 因此这里自己实现:
    - 内参: 用 ChArUco 角点对应关系跑 cv2.calibrateCamera
      (或加载 D405 出厂内参 JSON)
    - 手眼: AX=XB, 旋转用 Kabsch(轴对齐), 平移用最小二乘

标定板(默认, 与实体板一致):
  棋盘格内角点 5x7(横向5, 纵向7) -> CharucoBoard(4x6 格子)
  Checker 32mm / Marker 24mm / DICT_4X4

用法:
  1. 把 RealSense 拍的标定板图片放进 calib/handeye/images/
     (建议用未旋转的原始图, 与 D405 出厂内参对应)
  2. 把每张图对应的机械臂末端位姿按图片顺序写入
     calib/handeye/poses.txt, 每行:
         x, y, z, rx, ry, rz        (单位默认 mm + 度, Rz@Ry@Rx)
     或以 7 列: x, y, z, qw, qx, qy, qz (四元数, 用 --pose-format quat)
     图片按文件名排序后与 poses.txt 逐行对应
  3. 检测并标注:
         python3 charuco_handeye.py detect
     检查 out/overview.jpg, 每张图角点数应 >= 10
  4. 手眼标定(默认 eye-in-hand):
         python3 charuco_handeye.py calibrate

参数示例:
  python3 charuco_handeye.py calibrate --mode eye-in-hand \
      --images calib/handeye/images --poses calib/handeye/poses.txt \
      --intrinsics calib/d405_intrinsics.json --pos-unit mm --rpy-unit deg

eye-in-hand(眼在手上, 本项目): 相机装在机械臂末端, 标定板固定在工作台
  X = 相机坐标系 -> 机械臂末端坐标系
eye-to-hand(眼在手外): 相机固定, 标定板装在机械臂末端
  X = 相机坐标系 -> 机械臂基座坐标系
"""

import argparse
import json
import re
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 标定板参数(与实体板一致, 需要时可改)
# ---------------------------------------------------------------------------
BOARD_CORNERS = (5, 7)          # 内角点数量 (cols, rows)
BOARD_SQUARE_MM = 32.0          # 棋盘格边长 mm
BOARD_MARKER_MM = 24.0          # ArUco 码边长 mm
BOARD_DICT = "DICT_4X4_250"     # 与实体板字典一致; 若不对改成 _50/_100/_1000

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_IMAGES = PROJECT_ROOT / "calib" / "handeye" / "images"
DEFAULT_POSES = PROJECT_ROOT / "calib" / "handeye" / "poses.txt"
DEFAULT_OUT = PROJECT_ROOT / "calib" / "handeye" / "out"

IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
def natural_key(path: Path):
    """按文件名中的数字自然排序: img2 < img10"""
    return [int(t) if t.isdigit() else t.lower()
            for t in re.split(r"(\d+)", path.stem)]


def list_images(folder: Path):
    return sorted([p for p in Path(folder).iterdir()
                   if p.suffix.lower() in IMG_EXTS and not p.name.startswith(".")],
                  key=natural_key)


def make_board():
    """创建 CharucoBoard: 5x7 内角点 -> 4x6 个棋盘格"""
    squares = (BOARD_CORNERS[0] - 1, BOARD_CORNERS[1] - 1)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, BOARD_DICT))
    board = cv2.aruco.CharucoBoard(
        squares, BOARD_SQUARE_MM / 1000.0, BOARD_MARKER_MM / 1000.0, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector


def rvec_tvec_to_4x4(rvec, tvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).ravel()
    return T


def pose_to_matrix(pose, pos_unit, rpy_unit, pose_format):
    """把机械臂末端位姿转成 4x4 (基座->末端, 单位米)。"""
    scale = 1000.0 if pos_unit == "mm" else 1.0
    T = np.eye(4)
    if pose_format == "quat":
        x, y, z, qw, qx, qy, qz = (float(v) for v in pose)
        T[:3, 3] = [x / scale, y / scale, z / scale]
        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        q = q / np.linalg.norm(q)
        T[:3, :3] = cv2.Rodrigues(quat_to_rvec(q))[0]
        return T
    x, y, z, rx, ry, rz = (float(v) for v in pose)
    T[:3, 3] = [x / scale, y / scale, z / scale]
    if rpy_unit == "deg":
        rx, ry, rz = map(np.deg2rad, (rx, ry, rz))
    # 默认 Rz@Ry@Rx (与 leo038 仓库一致, 多数机械臂通用)
    Rx = cv2.Rodrigues(np.array([rx, 0, 0]))[0]
    Ry = cv2.Rodrigues(np.array([0, ry, 0]))[0]
    Rz = cv2.Rodrigues(np.array([0, 0, rz]))[0]
    T[:3, :3] = Rz @ Ry @ Rx
    return T


def quat_to_rvec(q):
    """w,x,y,z 四元数 -> 旋转向量"""
    w, x, y, z = q
    sin_half = np.sqrt(max(0.0, 1.0 - w * w))
    if sin_half < 1e-8:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(sin_half, w)
    return angle * np.array([x, y, z]) / sin_half


def load_poses(path: Path, pos_unit="mm", rpy_unit="deg", pose_format="xyzrpy"):
    """读取 poses.txt -> [(4x4 基座->末端)]。空行/# 开头行跳过, 逗号/空格分隔。"""
    poses = []
    for ln in Path(path).read_text(encoding="utf-8").splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        parts = [v for v in re.split(r"[,\s;]+", ln) if v]
        try:
            vals = [float(v) for v in parts]
        except ValueError:
            print(f"  [跳过] 无法解析的行: {ln!r}")
            continue
        n = 7 if pose_format == "quat" else 6
        if len(vals) < n:
            print(f"  [跳过] 列数不足({len(vals)}<{n}): {ln!r}")
            continue
        poses.append(pose_to_matrix(vals[:n], pos_unit, rpy_unit, pose_format))
    return poses


def save_json(path: Path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(
        json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"  已保存 {path}")


# ---------------------------------------------------------------------------
# 1) 检测 + 标注
# ---------------------------------------------------------------------------
def detect_images(images, board, detector, out_dir, intrinsics=None):
    """返回 [dict] 每张图的检测结果。"""
    results = []
    K = intrinsics["camera_matrix"] if intrinsics else None
    dist = intrinsics["dist_coeffs"] if intrinsics else None
    ann_dir = out_dir / "annotated"
    ann_dir.mkdir(parents=True, exist_ok=True)

    for idx, img_path in enumerate(images):
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [{idx:2d}] 读取失败: {img_path.name}")
            results.append({"file": img_path.name, "ok": False, "error": "read failed"})
            continue
        res = detector.detectBoard(img)
        corners, ids, marker_corners, marker_ids = res
        n_corners = 0 if corners is None else int(corners.shape[0])
        n_markers = 0 if marker_ids is None else int(marker_ids.shape[0])

        vis = img.copy()
        item = {"file": img_path.name, "ok": n_corners >= 4,
                "n_corners": n_corners, "n_markers": n_markers}

        if corners is not None and ids is not None:
            for (u, v), cid in zip(corners, ids):
                cv2.circle(vis, (int(u), int(v)), 6, (0, 0, 255), -1)
                cv2.putText(vis, str(int(cid)), (int(u) + 8, int(v)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if marker_corners:
            for mc, mid in zip(marker_corners, marker_ids):
                pts = np.asarray(mc, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(vis, [pts], True, (255, 0, 0), 2)
                cv2.putText(vis, f"m{int(mid)}", tuple(pts[0][0] + [4, -4]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # 有内参时解算板位姿 + 画坐标轴
        if K is not None and corners is not None and ids is not None and n_corners >= 4:
            obj = board.getChessboardCorners()[ids.ravel()]
            ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float32),
                                          K, dist)
            if ok:
                item["rvec"] = rvec.ravel().tolist()
                item["tvec"] = tvec.ravel().tolist()
                cv2.drawFrameAxes(vis, K, dist, rvec, tvec, 0.04)

        cv2.putText(vis, f"{idx}: {n_corners} corners / {n_markers} markers",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imwrite(str(ann_dir / f"{idx:02d}_{img_path.stem}_ann.jpg"), vis)
        results.append(item)
        flag = "OK " if item["ok"] else "BAD"
        print(f"  [{idx:2d}] {flag} {img_path.name}: "
              f"{n_corners} 角点 / {n_markers} markers")
    return results


def make_overview(images, results, out_dir):
    """把标注图拼成一张总览图, 便于快速检查。"""
    if not images:
        return
    ann_dir = out_dir / "annotated"
    cols = 4
    rows = (len(images) + cols - 1) // cols
    cell_w, cell_h = 320, 180
    canvas = np.full((rows * cell_h, cols * cell_w, 3), 32, dtype=np.uint8)
    for i, img_path in enumerate(images):
        ann = cv2.imread(str(ann_dir / f"{i:02d}_{img_path.stem}_ann.jpg"))
        if ann is None:
            continue
        ann = cv2.resize(ann, (cell_w, cell_h))
        r, c = divmod(i, cols)
        canvas[r * cell_h:(r + 1) * cell_h, c * cell_w:(c + 1) * cell_w] = ann
    save_path = out_dir / "overview.jpg"
    cv2.imwrite(str(save_path), canvas)
    print(f"  总览图已保存 {save_path}")


def cmd_detect(args):
    images = list_images(args.images)
    if not images:
        print(f"目录中没有图片: {args.images}")
        sys.exit(1)
    print(f"找到 {len(images)} 张图片")
    board, detector = make_board()
    print(f"标定板: {BOARD_CORNERS[0]}x{BOARD_CORNERS[1]} 内角点, "
          f"格子 {BOARD_SQUARE_MM}mm, marker {BOARD_MARKER_MM}mm, {BOARD_DICT}")

    intrinsics = None
    if args.intrinsics:
        d = json.loads(Path(args.intrinsics).read_text(encoding="utf-8"))
        intrinsics = {"camera_matrix": np.array(d["camera_matrix"], dtype=np.float64),
                      "dist_coeffs": np.array(d["dist_coeffs"], dtype=np.float64)}

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    results = detect_images(images, board, detector, out_dir, intrinsics)
    make_overview(images, results, out_dir)
    save_json(out_dir / "detections.json", {
        "board": {"corners": list(BOARD_CORNERS), "square_mm": BOARD_SQUARE_MM,
                  "marker_mm": BOARD_MARKER_MM, "dict": BOARD_DICT},
        "intrinsics_file": str(args.intrinsics) if args.intrinsics else None,
        "images": results,
    })
    ok = sum(1 for r in results if r.get("ok"))
    print(f"\n检测完成: {ok}/{len(images)} 张可用(角点>=4)。"
          f"建议全部 >=10 个角点再标定。")


# ---------------------------------------------------------------------------
# 2) 相机内参(自标定) + 板位姿
# ---------------------------------------------------------------------------
def self_calibrate_intrinsics(images, board, detector):
    """用 ChArUco 角点对应关系跑 cv2.calibrateCamera, 返回 K, dist, 各图板位姿。"""
    obj_pts, img_pts = [], []
    poses = []
    size = None
    for img_path in images:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        size = img.shape[1], img.shape[0]
        res = detector.detectBoard(img)
        corners, ids, _, _ = res
        if corners is None or ids is None or len(ids) < 8:
            print(f"  [跳过内参] {img_path.name}: 角点不足")
            poses.append(None)
            continue
        obj = board.getChessboardCorners()[ids.ravel()]
        obj_pts.append(obj.astype(np.float32))
        img_pts.append(corners.astype(np.float32))
        poses.append((img_path, corners, ids))

    if len(obj_pts) < 3:
        print("可用图太少, 无法自标定内参; 请提供 --intrinsics (D405 出厂内参)")
        sys.exit(1)
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, size, None, None)
    print(f"内参自标定完成(用 {len(obj_pts)} 张): "
          f"重投影误差 {ret:.3f} px")
    print("  相机内参 K =\n", K)
    print("  畸变 =", dist.ravel().tolist())

    # 重新为每张图解算板位姿(顺序与 images 对齐)
    board_poses = []
    for img_path in images:
        found = None
        for p, c, i in poses:
            if p == img_path:
                found = (c, i)
                break
        if found is None:
            board_poses.append(None)
            continue
        corners, ids = found
        obj = board.getChessboardCorners()[ids.ravel()]
        ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float32), K, dist)
        board_poses.append(rvec_tvec_to_4x4(rvec, tvec) if ok else None)
    return K, dist, board_poses


def load_intrinsics(path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"], dtype=np.float64)
    dist = np.array(d["dist_coeffs"], dtype=np.float64)
    return K, dist


# ---------------------------------------------------------------------------
# 3) 手眼标定 AX=XB
# ---------------------------------------------------------------------------
def solve_ax_xb(a_list, b_list):
    """
    解 AX=XB。
    约定: 传进来的 A_i 按"gripper2base"语义, B_i 为 target->cam。
      eye-in-hand: A_i = 基座->末端(FK),        X = cam->gripper
      eye-to-hand: A_i = 末端->基座(inv FK),    X = cam->base
    核心: 每对位姿满足 A_rel X = X B_rel,
      A_rel = inv(A_j) @ A_i,  B_rel = B_j @ inv(B_i)
    旋转: 所有 (i,j) 相对旋转轴做 Kabsch 对齐
    平移: (R_a - I) t_x = R_x t_b - t_a, 最小二乘
    """
    n = len(a_list)
    if n < 2:
        raise ValueError("至少需要 2 组位姿")

    # ---- 旋转: Kabsch 对齐轴 ----
    axes_a, axes_b = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A_rel = np.linalg.inv(a_list[j]) @ a_list[i]
            B_rel = b_list[j] @ np.linalg.inv(b_list[i])
            ra = cv2.Rodrigues(A_rel[:3, :3])[0].ravel()
            rb = cv2.Rodrigues(B_rel[:3, :3])[0].ravel()
            ta, tb = np.linalg.norm(ra), np.linalg.norm(rb)
            if ta < 1e-6 or tb < 1e-6:
                continue
            if abs(ta - tb) > np.pi:      # 2pi 歧义, 翻转 B 的轴
                rb = -rb
                tb = np.linalg.norm(rb)
            axes_a.append(ra / ta)
            axes_b.append(rb / tb)
    if len(axes_a) < 3:
        raise ValueError(f"有效轴约束太少({len(axes_a)}), "
                         "位姿旋转变化不足; 请让末端多变换姿态")
    P = np.vstack(axes_a)      # Nx3
    Q = np.vstack(axes_b)
    H = P.T @ Q
    U, S, Vt = np.linalg.svd(H)
    # 最优正交解: R = U @ Vt (Kabsch/Procrustes)
    R_x = U @ Vt
    if np.linalg.det(R_x) < 0:
        Vt[-1, :] *= -1
        R_x = U @ Vt

    # ---- 平移: 用同样的相对运动方程做最小二乘 ----
    A_mat, b_vec = [], []
    for i in range(n):
        for j in range(i + 1, n):
            A_rel = np.linalg.inv(a_list[j]) @ a_list[i]
            B_rel = b_list[j] @ np.linalg.inv(b_list[i])
            Ra = A_rel[:3, :3]
            ta = A_rel[:3, 3]
            Rb = B_rel[:3, :3]
            tb = B_rel[:3, 3]
            A_mat.append(Ra - np.eye(3))
            b_vec.append(R_x @ tb - ta)
    A_mat = np.vstack(A_mat)
    b_vec = np.hstack(b_vec)
    t_x, *_ = np.linalg.lstsq(A_mat, b_vec, rcond=None)

    X = np.eye(4)
    X[:3, :3] = R_x
    X[:3, 3] = t_x
    return X


def residual_stats(a_list, b_list, X):
    """手眼不变量: C_i = A_i · X · B_i 应为恒定矩阵。
    统计 C_i 的旋转/平移散度(相对均值)。"""
    refs = []
    for i in range(len(a_list)):
        refs.append(a_list[i] @ X @ b_list[i])
    cref = np.mean(refs, axis=0)
    c_rot, c_trans = [], []
    for C in refs:
        rvec = cv2.Rodrigues(cref[:3, :3].T @ C[:3, :3])[0].ravel()
        c_rot.append(np.rad2deg(np.linalg.norm(rvec)))
        c_trans.append(np.linalg.norm(cref[:3, 3] - C[:3, 3]) * 1000.0)
    return {
        "consistency_rot_deg": c_rot, "consistency_trans_mm": c_trans,
    }


def matrix_to_rpy_deg(R):
    """旋转矩阵 -> [rx, ry, rz] 度 (Rz@Ry@Rx 分解)"""
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-9:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0.0
    return np.rad2deg([rx, ry, rz])


def cmd_calibrate(args):
    images = list_images(args.images)
    if not images:
        print(f"目录中没有图片: {args.images}")
        sys.exit(1)
    board, detector = make_board()
    print(f"图片: {len(images)} 张")

    # 内参
    if args.intrinsics:
        K, dist = load_intrinsics(args.intrinsics)
        print(f"使用内参: {args.intrinsics}")
        # 用给定内参重新解算每张图的板位姿
        board_poses = []
        for img_path in images:
            img = cv2.imread(str(img_path))
            res = detector.detectBoard(img)
            corners, ids, _, _ = res
            if corners is None or ids is None or len(ids) < 4:
                board_poses.append(None)
                continue
            obj = board.getChessboardCorners()[ids.ravel()]
            ok, rvec, tvec = cv2.solvePnP(obj, corners.astype(np.float32), K, dist)
            board_poses.append(rvec_tvec_to_4x4(rvec, tvec) if ok else None)
    else:
        K, dist, board_poses = self_calibrate_intrinsics(images, board, detector)

    # 机械臂位姿
    arm_poses = load_poses(args.poses, args.pos_unit, args.rpy_unit, args.pose_format)
    print(f"机械臂位姿: {len(arm_poses)} 行 (图片 {len(images)} 张)")
    if len(arm_poses) != len(images):
        print(f"  [警告] 位姿数量与图片数量不一致, 将按 min 截断对齐")
    n = min(len(arm_poses), len(board_poses))

    # 构造 A_i(按模式) 和 B_i
    a_list, b_list, used = [], [], []
    for i in range(n):
        if board_poses[i] is None:
            continue
        A = arm_poses[i]
        if args.mode == "eye-to-hand":
            A = np.linalg.inv(A)          # 末端->基座
        a_list.append(A)
        b_list.append(board_poses[i])     # 板->相机
        used.append(i)
    print(f"可用位姿对: {len(used)}/{n}")
    if len(used) < 3:
        print("可用位姿太少, 无法标定")
        sys.exit(1)

    X = solve_ax_xb(a_list, b_list)
    stats = residual_stats(a_list, b_list, X)

    print("\n" + "=" * 60)
    print(f"模式: {args.mode}")
    print("X (相机坐标系 -> %s 坐标系):" %
          ("基座" if args.mode == "eye-to-hand" else "末端"))
    print(np.round(X, 6))
    print(f"平移: {X[:3, 3] * 1000.0} mm")
    print(f"旋转 RPY(Rz@Ry@Rx): {matrix_to_rpy_deg(X[:3, :3])} deg")
    print("=" * 60)
    cr = np.array(stats["consistency_rot_deg"])
    ct = np.array(stats["consistency_trans_mm"])
    print(f"C=AXB 一致性: 旋转 平均{cr.mean():.3f}deg/最大{cr.max():.3f}deg, "
          f"平移 平均{ct.mean():.2f}mm/最大{ct.max():.2f}mm")
    print("经验参考: 旋转残差 <1deg、平移残差 <5mm 视为可用; "
          "如果残差巨大, 先检查位姿单位/旋转顺序/模式")

    save_json(args.out / "handeye_result.json", {
        "mode": args.mode,
        "board": {"corners": list(BOARD_CORNERS), "square_mm": BOARD_SQUARE_MM,
                  "marker_mm": BOARD_MARKER_MM, "dict": BOARD_DICT},
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.tolist(),
        "X_matrix": X.tolist(),
        "X_translation_mm": (X[:3, 3] * 1000.0).tolist(),
        "X_rpy_deg": matrix_to_rpy_deg(X[:3, :3]).tolist(),
        "used_images": [images[i].name for i in used],
        "stats": {k: (float(np.mean(v)), float(np.max(v)))
                  for k, v in stats.items()},
    })


def cmd_info(_args):
    board, _ = make_board()
    print(f"标定板: {BOARD_CORNERS[0]}x{BOARD_CORNERS[1]} 内角点")
    print(f"  CharucoBoard squares = "
          f"{BOARD_CORNERS[0]-1}x{BOARD_CORNERS[1]-1} (格子)")
    print(f"  棋盘格 {BOARD_SQUARE_MM}mm / marker {BOARD_MARKER_MM}mm / {BOARD_DICT}")
    print(f"  全板角点坐标(内部 {board.getChessboardCorners().shape[0]} 个):")
    print(board.getChessboardCorners())


def main():
    p = argparse.ArgumentParser(description="ChArUco 检测标注 + 手眼标定")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("detect", help="检测/标注标定板图片")
    d.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    d.add_argument("--out", type=Path, default=DEFAULT_OUT)
    d.add_argument("--intrinsics", type=Path, default=None,
                   help="D405 出厂内参 JSON(可选, 有则画坐标轴)")
    d.set_defaults(func=cmd_detect)

    c = sub.add_parser("calibrate", help="手眼标定 AX=XB")
    c.add_argument("--images", type=Path, default=DEFAULT_IMAGES)
    c.add_argument("--poses", type=Path, default=DEFAULT_POSES)
    c.add_argument("--out", type=Path, default=DEFAULT_OUT)
    c.add_argument("--mode", choices=["eye-to-hand", "eye-in-hand"],
                   default="eye-in-hand",
                   help="eye-in-hand=相机在末端(默认, 本项目); eye-to-hand=相机固定")
    c.add_argument("--intrinsics", type=Path, default=None,
                   help="D405 出厂内参 JSON; 不填则用图片自标定内参")
    c.add_argument("--pos-unit", choices=["mm", "m"], default="mm")
    c.add_argument("--rpy-unit", choices=["deg", "rad"], default="deg")
    c.add_argument("--pose-format", choices=["xyzrpy", "quat"], default="xyzrpy")
    c.set_defaults(func=cmd_calibrate)

    i = sub.add_parser("info", help="打印标定板参数")
    i.set_defaults(func=cmd_info)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
