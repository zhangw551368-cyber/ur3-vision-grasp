"""Conservative table-plane grasps for targets with missing reflective depth."""

import numpy as np

from ur3_graspnet6dof.geometry import normalize


def pixel_on_horizontal_plane(pixel, intrinsics, base_from_camera, plane_z):
    """Intersect one optical-frame pixel ray with the base z=plane_z plane."""
    u, v = [float(value) for value in pixel]
    fx, fy, cx, cy = [float(value) for value in intrinsics]
    transform = np.asarray(base_from_camera, dtype=float).reshape(4, 4)
    ray_camera = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=float)
    origin = transform[:3, 3]
    ray_base = transform[:3, :3].dot(ray_camera)
    if abs(float(ray_base[2])) < 1.0e-8:
        raise ValueError("camera ray is parallel to the support plane")
    scale = (float(plane_z) - float(origin[2])) / float(ray_base[2])
    if scale <= 0.0:
        raise ValueError("support plane is behind the camera ray")
    return origin + scale * ray_base


def opening_axis(policy, detection, intrinsics, base_from_camera, support_z):
    mode = str(policy.get("opening_axis_mode", "fixed_base"))
    fallback_xy = policy.get("opening_axis_base_xy", [0.0, 1.0])
    if mode == "fixed_base" or detection.get("major_axis_image") is None:
        return normalize([float(fallback_xy[0]), float(fallback_xy[1]), 0.0])
    if mode != "perpendicular_to_major":
        raise ValueError("unsupported opening_axis_mode: {}".format(mode))
    major = normalize(np.asarray(detection["major_axis_image"], dtype=float))
    image_opening = np.array([-major[1], major[0]], dtype=float)
    center = np.asarray(detection["center"], dtype=float)
    step = float(policy.get("axis_pixel_step", 30.0))
    point0 = pixel_on_horizontal_plane(
        center, intrinsics, base_from_camera, support_z
    )
    point1 = pixel_on_horizontal_plane(
        center + step * image_opening,
        intrinsics,
        base_from_camera,
        support_z,
    )
    delta = point1 - point0
    return normalize([float(delta[0]), float(delta[1]), 0.0])


def build_topdown_candidate(
    detection, policy, intrinsics, base_from_camera, support_z
):
    """Return the executor's prepared-candidate representation."""
    center = pixel_on_horizontal_plane(
        detection["center"], intrinsics, base_from_camera, support_z
    )
    center_height = float(policy["center_height"])
    center[2] = float(support_z) + center_height
    approach = np.array([0.0, 0.0, -1.0], dtype=float)
    opening = opening_axis(
        policy, detection, intrinsics, base_from_camera, support_z
    )
    lateral = normalize(np.cross(approach, opening))
    opening = normalize(np.cross(lateral, approach))
    rotation = np.column_stack((approach, opening, lateral))
    return {
        "source": {
            "id": -1,
            "score": float(policy.get("score", 1.0)),
            "width": float(policy["gripper_width"]),
            "method": "rgb_table_plane_fallback",
            "category": detection.get("category"),
        },
        "center": center,
        "grasp_rotation": rotation,
        "approach": approach,
        "height_above_plane": center_height,
        "opening_axis_error_deg": None,
    }
