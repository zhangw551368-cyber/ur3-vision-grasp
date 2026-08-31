import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from ur3_graspnet6dof.config import load_config, resolve_project_path
from ur3_graspnet6dof.backend import calibrated_workspace_masks
from ur3_graspnet6dof.geometry import (
    grasp_to_tool_rotation,
    matrix_to_quaternion,
    point_plane_distance,
    quaternion_to_matrix,
)
from ur3_graspnet6dof.ros_image import decode_color, decode_depth_metres, roi_mask
from ur3_graspnet6dof.rgb_fallback import (
    build_topdown_candidate,
    pixel_on_horizontal_plane,
)
from ur3_graspnet6dof.detection_tracking import DetectionStabilizer
from ur3_graspnet6dof.disk_center import detect_circular_disk
from ur3_graspnet6dof.target_validation import nearest_target
from ur3_graspnet6dof.hex_opening import (
    detect_hexagonal_opening,
    fit_circle_center_xy,
    polygon_centroid,
)


class CoreTests(unittest.TestCase):
    def test_calibrated_workspace_foreground_mask(self):
        cloud = np.array(
            [[[0.4, -0.2, 0.0], [0.4, -0.2, 0.02], [0.9, -0.2, 0.02]]],
            dtype=float,
        )
        valid = np.ones((1, 3), dtype=bool)
        scene, foreground, _, height, plane = calibrated_workspace_masks(
            cloud,
            valid,
            np.eye(4),
            {"x": [0.2, 0.7], "y": [-0.6, -0.04], "z": [-0.06, 0.16]},
            0.0,
            0.01,
            0.15,
        )
        np.testing.assert_array_equal(scene, [[True, True, False]])
        np.testing.assert_array_equal(foreground, [[False, True, False]])
        np.testing.assert_allclose(height, cloud[..., 2])
        np.testing.assert_allclose(plane, [0.0, 0.0, 1.0, 0.0])

    def test_config_is_isolated(self):
        config = load_config(PROJECT_ROOT / "config" / "right_arm_green_table.yaml")
        self.assertEqual(Path(config["_project_root"]), PROJECT_ROOT)
        checkpoint = resolve_project_path(config, config["paths"]["checkpoint"])
        self.assertTrue(str(checkpoint).startswith(str(PROJECT_ROOT)))
        self.assertFalse(config["execution"]["enabled"])

    def test_grasp_axis_mapping(self):
        result = grasp_to_tool_rotation(np.eye(3), opening_axis_flip=False)
        np.testing.assert_allclose(result[:, 2], [1.0, 0.0, 0.0])
        np.testing.assert_allclose(result[:, 1], [0.0, 1.0, 0.0])
        self.assertAlmostEqual(float(np.linalg.det(result)), 1.0)
        quaternion = matrix_to_quaternion(result)
        np.testing.assert_allclose(quaternion_to_matrix(quaternion), result, atol=1e-9)

    def test_opening_axis_flip_is_equivalent_approach(self):
        normal = grasp_to_tool_rotation(np.eye(3), False)
        flipped = grasp_to_tool_rotation(np.eye(3), True)
        np.testing.assert_allclose(normal[:, 2], flipped[:, 2])
        np.testing.assert_allclose(normal[:, 1], -flipped[:, 1])
        np.testing.assert_allclose(normal[:, 0], -flipped[:, 0])

    def test_plane_distance(self):
        self.assertAlmostEqual(point_plane_distance([0.0, 0.0, 0.05], [0, 0, 1, 0]), 0.05)

    def test_roi(self):
        mask = roi_mask((100, 200), [0.25, 0.20, 0.75, 0.80])
        self.assertEqual(mask.shape, (100, 200))
        self.assertEqual(int(np.count_nonzero(mask)), 100 * 60)

    def test_ros_color_and_depth_decode(self):
        color_array = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        color_message = SimpleNamespace(
            encoding="bgr8",
            is_bigendian=0,
            step=6,
            width=2,
            height=1,
            data=color_array.tobytes(),
        )
        decoded = decode_color(color_message)
        np.testing.assert_array_equal(decoded, color_array[:, :, ::-1])

        depth_array = np.array([[500, 750]], dtype=np.uint16)
        depth_message = SimpleNamespace(
            encoding="16UC1",
            is_bigendian=0,
            step=4,
            width=2,
            height=1,
            data=depth_array.tobytes(),
        )
        np.testing.assert_allclose(decode_depth_metres(depth_message), [[0.5, 0.75]])

    def test_rgb_table_plane_fallback(self):
        base_from_camera = np.eye(4)
        base_from_camera[:3, :3] = np.diag([1.0, -1.0, -1.0])
        base_from_camera[2, 3] = 1.0
        intrinsics = [100.0, 100.0, 50.0, 40.0]
        point = pixel_on_horizontal_plane(
            [50.0, 40.0], intrinsics, base_from_camera, 0.0
        )
        np.testing.assert_allclose(point, [0.0, 0.0, 0.0])
        candidate = build_topdown_candidate(
            {"category": "upright_rivet", "center": [50.0, 40.0]},
            {
                "center_height": 0.025,
                "gripper_width": 0.018,
                "opening_axis_mode": "fixed_base",
                "opening_axis_base_xy": [0.0, 1.0],
            },
            intrinsics,
            base_from_camera,
            0.0,
        )
        np.testing.assert_allclose(candidate["center"], [0.0, 0.0, 0.025])
        np.testing.assert_allclose(candidate["approach"], [0.0, 0.0, -1.0])
        self.assertAlmostEqual(float(np.linalg.det(candidate["grasp_rotation"])), 1.0)

    def test_detection_stabilizer_hysteresis_and_dropout(self):
        stabilizer = DetectionStabilizer(
            confirmation_frames=2,
            category_switch_frames=3,
            maximum_missed_frames=1,
        )
        base = {
            "category": "metal_disc",
            "category_zh": "disc",
            "center": [100.0, 100.0],
            "bbox": [80, 90, 120, 110],
            "pickable": True,
            "priority": 4,
        }
        self.assertEqual(stabilizer.update([base]), [])
        stable = stabilizer.update([dict(base, center=[104.0, 98.0])])
        self.assertEqual(len(stable), 1)
        self.assertEqual(stable[0]["category"], "metal_disc")
        wrong = dict(base, category="black_connector", category_zh="black", priority=6)
        stable = stabilizer.update([wrong])
        self.assertEqual(stable[0]["category"], "metal_disc")
        self.assertEqual(len(stabilizer.update([])), 1)
        self.assertEqual(stabilizer.update([]), [])

    def test_nearest_frozen_target(self):
        objects = [
            {"category": "upright_rivet", "pickable": True, "center": [10, 10]},
            {"category": "upright_rivet", "pickable": True, "center": [50, 50]},
            {"category": "yellow_pliers", "pickable": True, "center": [12, 12]},
        ]
        item, distance = nearest_target(objects, "upright_rivet", [13, 14])
        self.assertEqual(item["center"], [10, 10])
        self.assertAlmostEqual(distance, 5.0)

    def test_hexagonal_opening_uses_perspective_ellipse_center(self):
        image = np.full((200, 300, 3), 245, dtype=np.uint8)
        vertices = np.array(
            [[230, 100], [220, 118], [198, 118], [188, 101], [199, 82], [220, 82]],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(image, vertices, (20, 20, 20))
        cv2.rectangle(image, (80, 125), (150, 137), (25, 25, 25), thickness=-1)
        result = detect_hexagonal_opening(
            image,
            [40, 40, 270, 170],
            {
                "gray_thresholds": [35, 45, 55],
                "min_polygon_area_px": 300,
                "max_polygon_area_px": 2000,
            },
        )
        expected = polygon_centroid(vertices)
        np.testing.assert_allclose(result["center"], expected, atol=1.5)
        self.assertEqual(len(result["vertices"]), 6)
        self.assertEqual(
            result["center_method"], "median_outer_aperture_ellipse_center"
        )
        self.assertGreaterEqual(result["ellipse_center_inlier_count"], 1)

    def test_hexagonal_opening_rejects_non_hexagonal_scene(self):
        image = np.full((120, 180, 3), 240, dtype=np.uint8)
        cv2.rectangle(image, (50, 40), (130, 70), (15, 15, 15), thickness=-1)
        with self.assertRaises(RuntimeError):
            detect_hexagonal_opening(image, [10, 10, 170, 110])

    def test_circle_fit_recovers_planar_center(self):
        angles = np.linspace(0.0, 2.0 * np.pi, 36, endpoint=False)
        expected = np.array([0.426, -0.088])
        radius = 0.014
        points = expected + radius * np.column_stack(
            (np.cos(angles), np.sin(angles))
        )
        center, fitted_radius, rms = fit_circle_center_xy(points)
        np.testing.assert_allclose(center, expected, atol=1.0e-9)
        self.assertAlmostEqual(fitted_radius, radius, places=9)
        self.assertLess(rms, 1.0e-9)

    def test_circular_disk_uses_outer_rim(self):
        image = np.full((240, 360, 3), (30, 145, 55), dtype=np.uint8)
        cv2.rectangle(image, (60, 35), (310, 210), (245, 245, 245), thickness=-1)
        expected = np.array([210.0, 125.0])
        cv2.circle(image, tuple(expected.astype(int)), 48, (185, 185, 185), thickness=-1)
        cv2.circle(image, tuple(expected.astype(int)), 48, (105, 105, 105), thickness=2)
        cv2.circle(image, tuple(expected.astype(int)), 10, (245, 245, 245), thickness=-1)
        for angle in np.linspace(0.0, 2.0 * np.pi, 5, endpoint=False):
            point = expected + 31.0 * np.array([np.cos(angle), np.sin(angle)])
            cv2.circle(image, tuple(np.round(point).astype(int)), 6, (90, 90, 90), thickness=-1)
        result = detect_circular_disk(image, [60, 35, 310, 210])
        np.testing.assert_allclose(result["center"], expected, atol=2.0)
        self.assertGreaterEqual(len(result["boundary_pixel_sets"]), 3)


if __name__ == "__main__":
    unittest.main()
