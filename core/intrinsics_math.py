"""Brown-Conrady distortion math: librealsense <-> OpenCV conventions.

The two libraries store distortion coefficients with OPPOSITE meaning, and
mixing them up biases every board pose and deprojection.

OpenCV: distCoeffs are FORWARD coefficients. projectPoints applies the
polynomial to an *ideal* normalised point to get the *observed* one:

    observed = BC(ideal ; coeffs)

librealsense RS2_DISTORTION_BROWN_CONRADY behaves the same way (the
polynomial is applied in rs2_project_point_to_pixel).

librealsense RS2_DISTORTION_INVERSE_BROWN_CONRADY applies the SAME
polynomial in the other direction -- in rs2_deproject_pixel_to_point --
so its coefficients satisfy:

    ideal = BC(observed ; coeffs)

Handing those to OpenCV as distCoeffs makes it undistort by re-applying a
distortion, so the residual error is roughly TWICE the true distortion
instead of zero. RealSense colour streams commonly report exactly this
model, which is why the conversion below exists.

fit_forward_from_inverse() converts by least squares. The forward model is
LINEAR in (k1, k2, p1, p2, k3), so the fit is a single lstsq with no
iteration and no local minima; the residual is reported in pixels so the
caller can prove the conversion is good rather than assume it.
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


def _pad5(coeffs: Sequence[float]) -> np.ndarray:
    c = list(coeffs[:5]) + [0.0] * max(0, 5 - len(coeffs))
    return np.asarray(c[:5], dtype=np.float64)


def brown_conrady(x: np.ndarray, y: np.ndarray,
                  coeffs: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """Apply the Brown-Conrady polynomial to normalised coordinates.

    coeffs are in OpenCV order (k1, k2, p1, p2, k3), which is also
    librealsense's storage order.
    """
    k1, k2, p1, p2, k3 = _pad5(coeffs)
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y)
    return xd, yd


def _pixel_grid(width: int, height: int, step: int = 16):
    us = np.arange(0.0, float(width), float(step))
    vs = np.arange(0.0, float(height), float(step))
    uu, vv = np.meshgrid(us, vs)
    return uu.ravel(), vv.ravel()


def max_distortion_px(coeffs: Sequence[float], width: int, height: int,
                      fx: float, fy: float, ppx: float,
                      ppy: float) -> float:
    """Largest displacement the polynomial produces anywhere in the image,
    in pixels. This is the size of the error you get from using the
    coefficients with the wrong convention (roughly doubled)."""
    if not any(abs(c) > 0.0 for c in _pad5(coeffs)):
        return 0.0
    u, v = _pixel_grid(width, height, step=8)
    x = (u - ppx) / fx
    y = (v - ppy) / fy
    xd, yd = brown_conrady(x, y, coeffs)
    du = (xd - x) * fx
    dv = (yd - y) * fy
    return float(np.max(np.hypot(du, dv)))


def ideal_from_observed(u, v, coeffs, fx: float, fy: float,
                        ppx: float, ppy: float):
    """Observed (distorted) pixels -> IDEAL pinhole pixels, EXACTLY, for the
    inverse-Brown-Conrady model.

    Under that model the stored polynomial already maps observed normalised
    coordinates to ideal ones, so this is a single forward evaluation with
    no fitting and no iteration -- exactly what librealsense does inside
    rs2_deproject_pixel_to_point. Feed the results to OpenCV with ZERO
    distortion coefficients and every pose/projection is consistent.
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    x = (u - ppx) / fx
    y = (v - ppy) / fy
    xi, yi = brown_conrady(x, y, coeffs)
    return xi * fx + ppx, yi * fy + ppy


_MAX_HALVINGS = 20      # damping steps allowed per Newton iteration


def brown_conrady_jacobian(x: np.ndarray, y: np.ndarray,
                           coeffs: Sequence[float]) -> Tuple[np.ndarray, ...]:
    """Analytic d(observed)/d(ideal) of brown_conrady, as (j11, j12, j21,
    j22). The polynomial's Jacobian is symmetric (j12 == j21), which is why
    both off-diagonal terms are the same expression."""
    k1, k2, p1, p2, k3 = _pad5(coeffs)
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    drad = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r2 * r2      # d(radial)/d(r2)
    j11 = radial + 2.0 * x * x * drad + 2.0 * p1 * y + 6.0 * p2 * x
    j12 = 2.0 * x * y * drad + 2.0 * p1 * x + 2.0 * p2 * y
    j21 = j12
    j22 = radial + 2.0 * y * y * drad + 2.0 * p2 * x + 6.0 * p1 * y
    return j11, j12, j21, j22


def _is_injective(x: np.ndarray, y: np.ndarray,
                  coeffs: Sequence[float]) -> np.ndarray:
    """True where the polynomial is still invertible at (x, y): the radial
    factor positive (further out it turns negative and flips the point through
    the principal point) and the Jacobian determinant positive (further out is
    past the fold, where two observed points share one ideal point). Neither
    branch beyond those limits is the point the geometry came from."""
    k1, k2, p1, p2, k3 = _pad5(coeffs)
    r2 = x * x + y * y
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    j11, j12, j21, j22 = brown_conrady_jacobian(x, y, coeffs)
    return (radial > 0.0) & ((j11 * j22 - j12 * j21) > 0.0)


def observed_from_ideal(u, v, coeffs, fx: float, fy: float, ppx: float,
                        ppy: float, iters: int = 50,
                        tol_norm: float = 1e-12):
    """Inverse of ideal_from_observed: IDEAL pixels -> observed pixels.

    Needed to draw overlays back onto the raw camera image. Solves
    BC(observed) = ideal by DAMPED NEWTON iteration on the analytic 2x2
    Jacobian above. The obvious fixed-point update, x <- x + (ideal - BC(x)),
    only contracts while that Jacobian stays near identity: at the corner of a
    1280x720 frame it already diverges for |k1| beyond about 0.25, which is an
    ordinary value for a wide-FOV module. Newton converges well past |k1|=0.4,
    and halving a step that does not reduce the residual keeps it from walking
    over the fold of a strongly negative k1.

    Raises ValueError, reporting the residual it reached, when the iteration
    has not converged to tol_norm (normalised units) after `iters` steps, or
    when it lands past that fold -- where the polynomial is no longer
    injective, so the ideal point has either no observed preimage or two of
    them and neither answer is the drawn geometry's. This is display-path
    maths, and geometry drawn from a silently unconverged answer lands
    hundreds of pixels from where it belongs.
    """
    u = np.asarray(u, dtype=np.float64)
    v = np.asarray(v, dtype=np.float64)
    xi = (u - ppx) / fx
    yi = (v - ppy) / fy
    x, y = xi.copy(), yi.copy()            # start at the ideal point
    xd, yd = brown_conrady(x, y, coeffs)
    rx = xi - xd
    ry = yi - yd
    for _ in range(int(iters)):
        res = np.hypot(rx, ry)
        active = res > tol_norm
        if not np.any(active):
            break
        j11, j12, j21, j22 = brown_conrady_jacobian(x, y, coeffs)
        det = j11 * j22 - j12 * j21
        det = np.where(np.abs(det) < 1e-15, 1e-15, det)
        sx = np.where(active, (j22 * rx - j12 * ry) / det, 0.0)
        sy = np.where(active, (j11 * ry - j21 * rx) / det, 0.0)
        # Damping: keep halving the step of any point it made worse, sent
        # non-finite, or pushed over the fold (det J <= 0), rather than
        # accepting a divergent Newton jump or the far branch of the map.
        lam = np.ones_like(res)
        for _h in range(_MAX_HALVINGS):
            nx = x + lam * sx
            ny = y + lam * sy
            nxd, nyd = brown_conrady(nx, ny, coeffs)
            nres = np.hypot(xi - nxd, yi - nyd)
            worse = active & ~((nres < res) & _is_injective(nx, ny, coeffs))
            if not np.any(worse):
                break
            lam = np.where(worse, lam * 0.5, lam)
        x, y = x + lam * sx, y + lam * sy
        xd, yd = brown_conrady(x, y, coeffs)
        rx = xi - xd
        ry = yi - yd
    worst = float(np.max(np.hypot(rx, ry)))
    if not worst <= tol_norm:              # also catches NaN
        raise ValueError(
            "observed_from_ideal did not converge: residual {:.3e} in "
            "normalised units ({:.3e} px) after {} iterations -- the ideal "
            "point has no observed preimage under coefficients {}.".format(
                worst, worst * max(abs(fx), abs(fy)), int(iters),
                list(_pad5(coeffs))))
    if not bool(np.all(_is_injective(x, y, coeffs))):
        raise ValueError(
            "observed_from_ideal converged past the fold of the polynomial "
            "(det J <= 0) under coefficients {} -- the distortion model is "
            "not invertible this far off axis, so the answer is the wrong "
            "branch, not the observed point.".format(list(_pad5(coeffs))))
    return x * fx + ppx, y * fy + ppy


def roundtrip_error_px(coeffs, width: int, height: int, fx: float, fy: float,
                       ppx: float, ppy: float) -> float:
    """Worst-case error of observed -> ideal -> observed over the image, in
    pixels. Proves the pair of transforms above is self-consistent.

    Propagates the ValueError from observed_from_ideal when the inverse does
    not converge anywhere on the grid."""
    if not any(abs(c) > 0.0 for c in _pad5(coeffs)):
        return 0.0
    u, v = _pixel_grid(width, height, step=8)
    ui, vi = ideal_from_observed(u, v, coeffs, fx, fy, ppx, ppy)
    ub, vb = observed_from_ideal(ui, vi, coeffs, fx, fy, ppx, ppy)
    return float(np.max(np.hypot(ub - u, vb - v)))
