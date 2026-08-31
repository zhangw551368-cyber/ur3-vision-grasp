"""Detect the dark opening of the upright assembly tube."""

import cv2
import numpy as np


def polygon_centroid(vertices):
    """Return the area centroid of an ordered, non-self-intersecting polygon."""
    points = np.asarray(vertices, dtype=np.float64).reshape(-1, 2)
    if len(points) < 3:
        raise ValueError("a polygon requires at least three vertices")
    following = np.roll(points, -1, axis=0)
    cross = points[:, 0] * following[:, 1] - following[:, 0] * points[:, 1]
    twice_area = float(np.sum(cross))
    if abs(twice_area) < 1.0e-8:
        raise ValueError("polygon area is zero")
    center = np.sum((points + following) * cross[:, None], axis=0)
    center /= 3.0 * twice_area
    return center


def fit_circle_center_xy(points):
    """Fit a circle to planar XY points and return centre, radius and RMS error."""
    xy = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(xy) < 6:
        raise ValueError("circle fitting requires at least six boundary points")
    design = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    squared_radius_term = np.sum(xy * xy, axis=1)
    solution, _, rank, _ = np.linalg.lstsq(design, squared_radius_term, rcond=None)
    if rank < 3:
        raise ValueError("circle boundary points are degenerate")
    center = solution[:2]
    radius_squared = float(solution[2] + np.dot(center, center))
    if radius_squared <= 0.0:
        raise ValueError("fitted circle radius is invalid")
    radius = float(np.sqrt(radius_squared))
    radial_errors = np.linalg.norm(xy - center, axis=1) - radius
    rms = float(np.sqrt(np.mean(radial_errors * radial_errors)))
    return center, radius, rms


def detect_hexagonal_opening(rgb, search_bbox, policy=None):
    """Find a dark convex six-sided opening inside ``search_bbox``.

    The hexagon validates that the dark component is the tube aperture.  The
    release reference itself is the median centre of ellipse fits to the outer
    aperture boundary at several thresholds.  A circle viewed by an oblique
    camera projects to an ellipse, so this follows the complete aperture (the
    operator's red-circle reference) instead of the darkest, possibly offset
    patch inside it.
    """
    policy = dict(policy or {})
    image = np.asarray(rgb)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("rgb image must have shape HxWx3")
    height, width = image.shape[:2]
    x1, y1, x2, y2 = [int(round(value)) for value in search_bbox]
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    if x2 - x1 < 20 or y2 - y1 < 20:
        raise ValueError("hexagon search box is empty or too small")

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    thresholds = policy.get("gray_thresholds", [35, 45, 55, 65, 75])
    epsilons = policy.get(
        "polygon_epsilon_ratios", [0.020, 0.025, 0.030, 0.035, 0.040, 0.050]
    )
    min_area = float(policy.get("min_polygon_area_px", 180.0))
    max_area = float(policy.get("max_polygon_area_px", 2500.0))
    min_size = int(policy.get("min_bbox_size_px", 15))
    max_size = int(policy.get("max_bbox_size_px", 90))
    kernel_size = int(policy.get("morphology_kernel_px", 3))
    center_threshold_min = int(policy.get("aperture_center_min_threshold", 45))
    center_outlier_px = float(policy.get("aperture_center_outlier_px", 5.0))
    min_ellipse_axis_ratio = float(policy.get("min_ellipse_axis_ratio", 0.50))
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    search_mask = np.zeros((height, width), dtype=np.uint8)
    search_mask[y1:y2, x1:x2] = 255
    candidates = []

    for threshold_index, threshold in enumerate(thresholds):
        binary = ((gray < int(threshold)) & (search_mask > 0)).astype(np.uint8) * 255
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area < min_area * 0.65 or contour_area > max_area * 1.35:
                continue
            bx, by, bw, bh = cv2.boundingRect(contour)
            if not (min_size <= bw <= max_size and min_size <= bh <= max_size):
                continue
            aspect = float(bw) / float(max(bh, 1))
            if not 0.45 <= aspect <= 2.20:
                continue
            hull = cv2.convexHull(contour)
            hull_area = float(cv2.contourArea(hull))
            if hull_area <= 0.0 or contour_area / hull_area < 0.72:
                continue
            if len(contour) < 5:
                continue
            ellipse = cv2.fitEllipse(contour)
            ellipse_center = np.asarray(ellipse[0], dtype=np.float64)
            ellipse_axes = np.asarray(ellipse[1], dtype=np.float64)
            ellipse_axis_ratio = float(
                np.min(ellipse_axes) / max(np.max(ellipse_axes), 1.0e-6)
            )
            if ellipse_axis_ratio < min_ellipse_axis_ratio:
                continue
            perimeter = float(cv2.arcLength(hull, True))
            for epsilon in epsilons:
                polygon = cv2.approxPolyDP(
                    hull, float(epsilon) * perimeter, True
                ).reshape(-1, 2)
                if len(polygon) != 6 or not cv2.isContourConvex(
                    polygon.astype(np.int32)
                ):
                    continue
                polygon_area = float(cv2.contourArea(polygon.astype(np.float32)))
                if not min_area <= polygon_area <= max_area:
                    continue
                center = polygon_centroid(polygon)
                if not (x1 <= center[0] < x2 and y1 <= center[1] < y2):
                    continue
                next_points = np.roll(polygon.astype(np.float64), -1, axis=0)
                side_lengths = np.linalg.norm(next_points - polygon, axis=1)
                side_cv = float(np.std(side_lengths) / max(np.mean(side_lengths), 1e-6))
                # Prefer the darkest valid six-sided boundary. Area and side
                # consistency break ties without assuming a regular front view.
                score = (
                    1000.0
                    - 8.0 * float(threshold_index)
                    + 0.02 * polygon_area
                    - 30.0 * side_cv
                )
                candidates.append(
                    {
                        "center": center.tolist(),
                        "polygon_center": center.tolist(),
                        "ellipse_center": ellipse_center.tolist(),
                        "ellipse_axes_px": ellipse_axes.tolist(),
                        "ellipse_axis_ratio": ellipse_axis_ratio,
                        "boundary_pixels": contour.reshape(-1, 2).astype(int).tolist(),
                        "vertices": polygon.astype(int).tolist(),
                        "threshold": int(threshold),
                        "polygon_area_px": polygon_area,
                        "bbox": [int(bx), int(by), int(bx + bw), int(by + bh)],
                        "side_length_cv": side_cv,
                        "score": score,
                    }
                )

    if not candidates:
        raise RuntimeError("no dark convex hexagonal opening found in the search box")
    candidates.sort(key=lambda item: item["score"], reverse=True)

    # Keep one validated aperture observation per gray threshold.  Repeated
    # polygon epsilons often describe the same contour and must not overweight
    # one threshold in the median.
    per_threshold = {}
    for candidate in candidates:
        threshold = candidate["threshold"]
        if threshold < center_threshold_min:
            continue
        if threshold not in per_threshold:
            per_threshold[threshold] = candidate
    if not per_threshold:
        raise RuntimeError("no outer aperture boundary passed ellipse validation")

    observations = list(per_threshold.values())
    centers = np.asarray(
        [item["ellipse_center"] for item in observations], dtype=np.float64
    )
    initial_center = np.median(centers, axis=0)
    distances = np.linalg.norm(centers - initial_center, axis=1)
    inliers = distances <= center_outlier_px
    if not np.any(inliers):
        raise RuntimeError("aperture ellipse centres have no consistent observations")
    refined_center = np.median(centers[inliers], axis=0)

    representative = min(
        (item for item, keep in zip(observations, inliers) if keep),
        key=lambda item: np.linalg.norm(
            np.asarray(item["ellipse_center"], dtype=np.float64) - refined_center
        ),
    )
    best = dict(representative)
    best["center"] = refined_center.tolist()
    best["hex_centroid"] = best["polygon_center"]
    best["center_method"] = "median_outer_aperture_ellipse_center"
    best["ellipse_center_samples"] = centers.tolist()
    best["ellipse_center_inlier_count"] = int(np.count_nonzero(inliers))
    best["boundary_pixel_sets"] = [
        {
            "threshold": int(item["threshold"]),
            "pixels": item["boundary_pixels"],
        }
        for item, keep in zip(observations, inliers)
        if keep
    ]
    best["search_bbox"] = [x1, y1, x2, y2]
    best["candidate_count"] = len(candidates)
    return best
