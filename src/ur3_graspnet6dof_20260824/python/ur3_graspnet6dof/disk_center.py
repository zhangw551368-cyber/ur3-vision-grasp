"""Detect the outer boundary and centre of a circular metal disk on white paper."""

import cv2
import numpy as np


def _ellipse_pixels(ellipse, count=180):
    (cx, cy), (axis_a, axis_b), angle_deg = ellipse
    angles = np.linspace(0.0, 2.0 * np.pi, int(count), endpoint=False)
    local = np.column_stack(
        (0.5 * axis_a * np.cos(angles), 0.5 * axis_b * np.sin(angles))
    )
    angle = np.deg2rad(float(angle_deg))
    rotation = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]],
        dtype=float,
    )
    return local.dot(rotation.T) + np.array([cx, cy], dtype=float)


def detect_circular_disk(rgb, search_bbox, policy=None):
    """Locate a disk by its complete outer rim inside ``search_bbox``.

    Hough detection supplies a coarse radius. Multiple Canny thresholds then
    fit the real outer rim, avoiding the disk's centre hole and smaller bolt
    holes. Boundary samples are returned for metric plane unprojection.
    """
    policy = dict(policy or {})
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("rgb image must have shape HxWx3")
    height, width = image.shape[:2]
    x1, y1, x2, y2 = [int(round(value)) for value in search_bbox]
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    if x2 - x1 < 80 or y2 - y1 < 80:
        raise ValueError("disk search box is empty or too small")

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    roi = gray[y1:y2, x1:x2]
    blur_size = int(policy.get("median_blur_px", 5))
    if blur_size % 2 == 0:
        blur_size += 1
    blurred = cv2.medianBlur(roi, blur_size)
    min_radius = int(policy.get("min_radius_px", 30))
    max_radius = int(policy.get("max_radius_px", 70))
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=float(policy.get("min_circle_distance_px", 40)),
        param1=float(policy.get("hough_edge_threshold", 80)),
        param2=float(policy.get("hough_accumulator_threshold", 25)),
        minRadius=min_radius,
        maxRadius=max_radius,
    )
    if circles is None:
        raise RuntimeError("no circular disk rim found in the white-paper region")
    coarse = np.asarray(circles[0][0], dtype=float)
    coarse_center = coarse[:2]
    coarse_radius = float(coarse[2])

    annulus_inner = float(policy.get("outer_annulus_inner_ratio", 0.82))
    annulus_outer = float(policy.get("outer_annulus_outer_ratio", 1.16))
    min_axis_ratio = float(policy.get("min_outer_axis_ratio", 0.70))
    max_center_error = float(policy.get("max_center_error_ratio", 0.20)) * coarse_radius
    canny_pairs = policy.get(
        "canny_threshold_pairs", [[20, 60], [30, 90], [40, 120], [50, 150]]
    )
    yy_grid, xx_grid = np.indices(roi.shape)
    radial = np.hypot(xx_grid - coarse_center[0], yy_grid - coarse_center[1])
    ellipse_observations = []
    for low, high in canny_pairs:
        edges = cv2.Canny(
            cv2.GaussianBlur(roi, (5, 5), 0), int(low), int(high)
        )
        mask = (
            (edges > 0)
            & (radial >= annulus_inner * coarse_radius)
            & (radial <= annulus_outer * coarse_radius)
        )
        ys, xs = np.nonzero(mask)
        if len(xs) < 30:
            continue
        points = np.column_stack((xs, ys)).astype(np.float32).reshape(-1, 1, 2)
        ellipse = cv2.fitEllipse(points)
        ellipse_center = np.asarray(ellipse[0], dtype=float)
        axes = np.asarray(ellipse[1], dtype=float)
        axis_ratio = float(np.min(axes) / max(np.max(axes), 1.0e-6))
        if axis_ratio < min_axis_ratio:
            continue
        if np.linalg.norm(ellipse_center - coarse_center) > max_center_error:
            continue
        if not 1.55 * coarse_radius <= float(np.mean(axes)) <= 2.35 * coarse_radius:
            continue
        pixels = _ellipse_pixels(ellipse)
        pixels[:, 0] += x1
        pixels[:, 1] += y1
        ellipse_observations.append(
            {
                "canny_thresholds": [int(low), int(high)],
                "center": (ellipse_center + np.array([x1, y1])).tolist(),
                "axes_px": axes.tolist(),
                "angle_deg": float(ellipse[2]),
                "boundary_pixels": pixels.tolist(),
            }
        )
    if len(ellipse_observations) < 3:
        raise RuntimeError("fewer than three valid outer-disk rim fits")

    centers = np.asarray([item["center"] for item in ellipse_observations])
    center = np.median(centers, axis=0)
    return {
        "center": center.tolist(),
        "center_method": "median_multi_edge_outer_rim_ellipse_center",
        "coarse_hough_center": (coarse_center + np.array([x1, y1])).tolist(),
        "coarse_hough_radius_px": coarse_radius,
        "ellipse_observations": ellipse_observations,
        "boundary_pixel_sets": [
            {
                "threshold": item["canny_thresholds"],
                "pixels": item["boundary_pixels"],
            }
            for item in ellipse_observations
        ],
        "search_bbox": [x1, y1, x2, y2],
    }
