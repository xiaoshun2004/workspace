# -*- coding: utf-8 -*-
"""
transforms.py
-------------
Small, well-tested SE(3) helpers. Pure math: numpy + cv2 only.

Naming convention (used everywhere):
    T_a2b maps a point expressed in frame a into frame b:
        p_b = T_a2b @ p_a
    Equivalently, T_a2b is the pose of frame a expressed in frame b.

This is the same direction OpenCV uses for its calibrateHandEye / solvePnP
argument names (gripper2base, target2cam, cam2gripper), which is why we adopt
it: it removes a whole class of "inverted transform" bugs.
"""

from __future__ import annotations

import cv2
import numpy as np


def make_se3(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a 3x3 rotation and 3-vector."""
    rotation = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(translation, dtype=np.float64).reshape(3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rotation
    T[:3, 3] = translation
    return T


def se3_inverse(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform (transpose-based, no full inverse)."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def se3_compose(*transforms: np.ndarray) -> np.ndarray:
    """Chain transforms left to right: se3_compose(A, B) == A @ B."""
    out = np.eye(4, dtype=np.float64)
    for T in transforms:
        out = out @ np.asarray(T, dtype=np.float64).reshape(4, 4)
    return out


def transform_point(T_a2b: np.ndarray, p_a: np.ndarray) -> np.ndarray:
    """Map a single 3D point from frame a into frame b."""
    T_a2b = np.asarray(T_a2b, dtype=np.float64).reshape(4, 4)
    p_a = np.asarray(p_a, dtype=np.float64).reshape(3)
    ph = np.ones(4, dtype=np.float64)
    ph[:3] = p_a
    return (T_a2b @ ph)[:3]


def transform_points(T_a2b: np.ndarray, pts_a: np.ndarray) -> np.ndarray:
    """Map an (N, 3) array of points from frame a into frame b."""
    T_a2b = np.asarray(T_a2b, dtype=np.float64).reshape(4, 4)
    pts_a = np.asarray(pts_a, dtype=np.float64).reshape(-1, 3)
    ph = np.hstack([pts_a, np.ones((pts_a.shape[0], 1), dtype=np.float64)])
    return (ph @ T_a2b.T)[:, :3]


def rvec_tvec_to_se3(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert an OpenCV (rvec, tvec) pair into a 4x4 transform.

    solvePnP returns (rvec, tvec) describing target2cam, so this yields T_target2cam.
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return make_se3(R, np.asarray(tvec, dtype=np.float64).reshape(3))


def se3_to_rvec_tvec(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Inverse of rvec_tvec_to_se3. Returns (rvec (3,1), tvec (3,1))."""
    T = np.asarray(T, dtype=np.float64).reshape(4, 4)
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    tvec = T[:3, 3].reshape(3, 1)
    return rvec, tvec


def se3_translation(T: np.ndarray) -> np.ndarray:
    """Return the translation component (3,) of a transform."""
    return np.asarray(T, dtype=np.float64).reshape(4, 4)[:3, 3].copy()


def se3_to_euler_deg(T: np.ndarray) -> np.ndarray:
    """Return rotation as (rx, ry, rz) Euler angles in degrees, for display only.

    Uses cv2.RQDecomp3x3, which returns extrinsic XYZ Euler angles in degrees.
    """
    R = np.asarray(T, dtype=np.float64).reshape(4, 4)[:3, :3]
    angles, _, _, _, _, _ = cv2.RQDecomp3x3(R)
    return np.asarray(angles, dtype=np.float64).reshape(3)


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices, in degrees."""
    R_a = np.asarray(R_a, dtype=np.float64).reshape(3, 3)
    R_b = np.asarray(R_b, dtype=np.float64).reshape(3, 3)
    R_err = R_a.T @ R_b
    cos_theta = (np.trace(R_err) - 1.0) / 2.0
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_theta)))
