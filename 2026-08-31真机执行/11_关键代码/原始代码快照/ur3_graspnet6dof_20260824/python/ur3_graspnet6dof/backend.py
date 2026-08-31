import sys
from pathlib import Path

import numpy as np

from .config import resolve_project_path


def calibrated_workspace_masks(
    cloud_organized,
    valid_mask,
    camera_to_planning,
    workspace_bounds,
    support_plane_z,
    foreground_min_height,
    foreground_max_height,
):
    """Build class-agnostic scene/foreground masks in the planning frame."""
    cloud = np.asarray(cloud_organized, dtype=np.float64)
    transform = np.asarray(camera_to_planning, dtype=np.float64).reshape(4, 4)
    planning = cloud.dot(transform[:3, :3].T) + transform[:3, 3]
    scene = np.asarray(valid_mask, dtype=bool).copy()
    for index, axis in enumerate(("x", "y", "z")):
        lower, upper = [float(value) for value in workspace_bounds[axis]]
        scene &= planning[..., index] >= lower
        scene &= planning[..., index] <= upper
    height = planning[..., 2] - float(support_plane_z)
    foreground = scene.copy()
    foreground &= height >= float(foreground_min_height)
    foreground &= height <= float(foreground_max_height)

    # p_planning = R p_camera + t, therefore planning z-z0=0 is:
    # R[2,:] p_camera + t_z-z0=0. R is orthonormal, so it is normalized.
    plane_camera = np.r_[
        transform[2, :3], transform[2, 3] - float(support_plane_z)
    ]
    return scene, foreground, planning, height, plane_camera


class GraspNetBackend:
    def __init__(self, config):
        self.config = config
        paths = config["paths"]
        self.baseline_root = resolve_project_path(config, paths["graspnet_baseline"])
        self.api_root = resolve_project_path(config, paths["graspnet_api"])
        self.checkpoint_path = resolve_project_path(config, paths["checkpoint"])
        self._configure_imports()
        self._load_modules()
        self.device = self.torch.device("cuda:0" if self.torch.cuda.is_available() else "cpu")
        if self.device.type != "cuda":
            raise RuntimeError("GraspNet real-time inference requires CUDA in this project")
        self.net = self._load_network()

    def _configure_imports(self):
        required = [
            self.baseline_root,
            self.baseline_root / "models",
            self.baseline_root / "dataset",
            self.baseline_root / "utils",
            self.api_root,
        ]
        missing = [str(path) for path in required if not path.exists()]
        if missing:
            raise RuntimeError("GraspNet source paths are missing: {}".format(", ".join(missing)))
        for path in reversed(required):
            value = str(path)
            if value not in sys.path:
                sys.path.insert(0, value)
        if not self.checkpoint_path.is_file():
            raise RuntimeError("checkpoint not found: {}".format(self.checkpoint_path))

    def _load_modules(self):
        import open3d as o3d
        import torch
        from collision_detector import ModelFreeCollisionDetector
        from graspnet import GraspNet, pred_decode
        from graspnetAPI import GraspGroup

        self.o3d = o3d
        self.torch = torch
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self.GraspNet = GraspNet
        self.pred_decode = pred_decode
        self.GraspGroup = GraspGroup

    def _load_network(self):
        network = self.config["network"]
        net = self.GraspNet(
            input_feature_dim=0,
            num_view=int(network["num_view"]),
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        net.to(self.device)
        checkpoint = self.torch.load(str(self.checkpoint_path), map_location=self.device)
        if "model_state_dict" not in checkpoint:
            raise RuntimeError("checkpoint has no model_state_dict")
        net.load_state_dict(checkpoint["model_state_dict"])
        net.eval()
        self.checkpoint_epoch = int(checkpoint.get("epoch", -1))
        return net

    @staticmethod
    def point_cloud_from_depth(depth_m, intrinsic):
        height, width = depth_m.shape
        rows, cols = np.indices((height, width), dtype=np.float32)
        fx, fy = float(intrinsic[0, 0]), float(intrinsic[1, 1])
        cx, cy = float(intrinsic[0, 2]), float(intrinsic[1, 2])
        z = depth_m.astype(np.float32)
        x = (cols - cx) * z / fx
        y = (rows - cy) * z / fy
        return np.stack((x, y, z), axis=-1)

    def _fit_plane(self, cloud_points):
        network = self.config["network"]
        if not network.get("fit_support_plane", True) or len(cloud_points) < 1000:
            return None, 0
        cloud = self.o3d.geometry.PointCloud()
        cloud.points = self.o3d.utility.Vector3dVector(cloud_points.astype(np.float64))
        plane, inliers = cloud.segment_plane(
            distance_threshold=float(network["plane_distance_threshold"]),
            ransac_n=3,
            num_iterations=int(network["plane_num_iterations"]),
        )
        plane = np.asarray(plane, dtype=np.float64)
        if not np.all(np.isfinite(plane)) or np.linalg.norm(plane[:3]) < 1e-8:
            return None, 0
        return plane, len(inliers)

    def infer(
        self,
        color_rgb,
        depth_m,
        intrinsic,
        valid_mask,
        camera_to_planning=None,
        target_pixel=None,
    ):
        network = self.config["network"]
        cloud_organized = self.point_cloud_from_depth(depth_m, intrinsic)
        mask = np.asarray(valid_mask, dtype=bool) & np.isfinite(depth_m) & (depth_m > 0)
        plane = None
        plane_inliers = 0
        scene_mask = mask
        foreground_mask = mask
        height_organized = None
        if camera_to_planning is not None:
            selector = self.config["selector"]
            scene_mask, foreground_mask, _, height_organized, plane = (
                calibrated_workspace_masks(
                    cloud_organized,
                    mask,
                    camera_to_planning,
                    selector["workspace_bounds"],
                    selector["support_plane_z"],
                    network["foreground_min_height"],
                    network["foreground_max_height"],
                )
            )
            plane_inliers = int(
                np.count_nonzero(
                    scene_mask
                    & (np.abs(height_organized) <= float(network["plane_distance_threshold"]))
                )
            )

        if target_pixel is not None:
            target_u, target_v, target_radius = [float(value) for value in target_pixel]
            rows, cols = np.indices(mask.shape)
            target_mask = (
                (cols - target_u) ** 2 + (rows - target_v) ** 2
                <= target_radius ** 2
            )
            foreground_mask &= target_mask

        scene_cloud = cloud_organized[scene_mask].astype(np.float32)
        foreground_cloud = cloud_organized[foreground_mask].astype(np.float32)
        foreground_color = color_rgb[foreground_mask].astype(np.float32) / 255.0
        if len(scene_cloud) < 500:
            raise RuntimeError(
                "only {} valid RGB-D points in the calibrated workspace".format(len(scene_cloud))
            )
        minimum_foreground = int(network.get("min_foreground_points", 500))
        if len(foreground_cloud) < minimum_foreground:
            raise RuntimeError(
                "only {} foreground points above the support plane (need {}); "
                "check depth coverage or place a larger workpiece".format(
                    len(foreground_cloud), minimum_foreground
                )
            )

        num_point = int(network["num_point"])
        if len(foreground_cloud) >= num_point:
            indices = np.random.choice(len(foreground_cloud), num_point, replace=False)
        else:
            base = np.arange(len(foreground_cloud))
            extra = np.random.choice(
                len(foreground_cloud), num_point - len(foreground_cloud), replace=True
            )
            indices = np.concatenate((base, extra))
        sampled = foreground_cloud[indices]

        end_points = {
            "point_clouds": self.torch.from_numpy(sampled[np.newaxis]).to(self.device),
            "cloud_colors": foreground_color[indices],
        }
        with self.torch.no_grad():
            predictions = self.pred_decode(self.net(end_points))
        grasp_array = predictions[0].detach().cpu().numpy()
        grasps = self.GraspGroup(grasp_array)

        collision_thresh = float(network["collision_thresh"])
        if collision_thresh > 0.0 and len(grasps) > 0:
            detector = self.ModelFreeCollisionDetector(
                scene_cloud,
                voxel_size=float(network["collision_voxel_size"]),
            )
            collision_mask = detector.detect(
                grasps,
                approach_dist=float(network["collision_approach_dist"]),
                collision_thresh=collision_thresh,
            )
            grasps = grasps[~collision_mask]

        if len(grasps) > 0:
            grasps.nms()
            grasps.sort_by_score()
            if camera_to_planning is not None:
                selector = self.config["selector"]
                transform = np.asarray(camera_to_planning, dtype=np.float64)
                keep = []
                required_down = np.cos(
                    np.deg2rad(float(selector["max_approach_tilt_deg"]))
                )
                bounds = selector["workspace_bounds"]
                for grasp in grasps:
                    center = transform[:3, :3].dot(grasp.translation) + transform[:3, 3]
                    approach = transform[:3, :3].dot(grasp.rotation_matrix[:, 0])
                    height = center[2] - float(selector["support_plane_z"])
                    in_bounds = all(
                        float(bounds[axis][0]) <= center[index] <= float(bounds[axis][1])
                        for index, axis in enumerate(("x", "y", "z"))
                    )
                    in_target = True
                    if target_pixel is not None:
                        camera_center = np.asarray(grasp.translation, dtype=np.float64)
                        if camera_center[2] <= 0.0:
                            in_target = False
                        else:
                            u = (
                                float(intrinsic[0, 0]) * camera_center[0] / camera_center[2]
                                + float(intrinsic[0, 2])
                            )
                            v = (
                                float(intrinsic[1, 1]) * camera_center[1] / camera_center[2]
                                + float(intrinsic[1, 2])
                            )
                            in_target = (
                                (u - float(target_pixel[0])) ** 2
                                + (v - float(target_pixel[1])) ** 2
                                <= float(target_pixel[2]) ** 2
                            )
                    keep.append(
                        in_bounds
                        and in_target
                        and float(selector["min_height_above_plane"])
                        <= height
                        <= float(selector["max_height_above_plane"])
                        and float(np.dot(approach, [0.0, 0.0, -1.0])) >= required_down
                        and float(selector["min_gripper_width"])
                        <= float(grasp.width)
                        <= float(selector["max_gripper_width"])
                        and float(grasp.score) >= float(selector["min_score"])
                    )
                grasps = grasps[np.asarray(keep, dtype=bool)]
            grasps = grasps[: int(network["publish_top_n"])]

        candidates = []
        for index in range(len(grasps)):
            grasp = grasps[index]
            rotation = np.asarray(grasp.rotation_matrix, dtype=np.float64).reshape(3, 3)
            translation = np.asarray(grasp.translation, dtype=np.float64).reshape(3)
            if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
                continue
            candidates.append(
                {
                    "id": index,
                    "score": float(grasp.score),
                    "width": float(grasp.width),
                    "height": float(grasp.height),
                    "depth": float(grasp.depth),
                    "translation": translation.tolist(),
                    "rotation": rotation.tolist(),
                }
            )

        if plane is None:
            plane, plane_inliers = self._fit_plane(scene_cloud)
        diagnostics = {
            "valid_point_count": int(np.count_nonzero(mask)),
            "workspace_point_count": int(len(scene_cloud)),
            "foreground_point_count": int(len(foreground_cloud)),
            "sampled_point_count": int(num_point),
            "candidate_count": int(len(candidates)),
            "support_plane_camera": plane.tolist() if plane is not None else None,
            "support_plane_inlier_count": int(plane_inliers),
            "support_plane_source": (
                "calibrated_tf_and_config"
                if camera_to_planning is not None
                else "ransac"
            ),
            "target_pixel": (
                [float(value) for value in target_pixel]
                if target_pixel is not None
                else None
            ),
        }
        return candidates, diagnostics
