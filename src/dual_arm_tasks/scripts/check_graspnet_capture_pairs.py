#!/usr/bin/env python3

import argparse
import os
import sys

import cv2
import numpy as np
import scipy.io as scio


def parse_args():
    parser = argparse.ArgumentParser(
        description="Check whether GraspNet-style RGB/depth/meta/mask files share one geometry space."
    )
    parser.add_argument(
        "--camera-dir",
        default="/home/gzu/gzu_ws/datasets/kinect_graspnet_capture/scenes/scene_0000/kinect",
    )
    parser.add_argument("--max-details", type=int, default=5)
    return parser.parse_args()


def stems(directory, suffix):
    if not os.path.isdir(directory):
        return set()
    result = set()
    for name in os.listdir(directory):
        stem, ext = os.path.splitext(name)
        if ext.lower() == suffix and stem.isdigit():
            result.add(stem)
    return result


def image_summary(path):
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if image is None:
        return None
    height, width = image.shape[:2]
    return {
        "width": width,
        "height": height,
        "shape": image.shape,
        "dtype": str(image.dtype),
        "min": int(np.min(image)) if image.size else 0,
        "max": int(np.max(image)) if image.size else 0,
        "nonzero": int(np.count_nonzero(image)),
    }


def mat_scalar(data, key, default=""):
    if key not in data:
        return default
    value = np.asarray(data[key]).squeeze()
    if value.shape == ():
        return value.item()
    if value.size == 1:
        return value.ravel()[0]
    return value


def mat_string(data, key):
    value = mat_scalar(data, key, "")
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def check_sample(camera_dir, stem):
    paths = {
        "rgb": os.path.join(camera_dir, "rgb", stem + ".png"),
        "depth": os.path.join(camera_dir, "depth", stem + ".png"),
        "workspace_mask": os.path.join(camera_dir, "workspace_mask", stem + ".png"),
        "meta": os.path.join(camera_dir, "meta", stem + ".mat"),
    }
    rgb = image_summary(paths["rgb"])
    depth = image_summary(paths["depth"])
    mask = image_summary(paths["workspace_mask"])
    meta = scio.loadmat(paths["meta"])
    intrinsic = np.asarray(meta.get("intrinsic_matrix", []))

    problems = []
    if rgb is None:
        problems.append("rgb unreadable")
    if depth is None:
        problems.append("depth unreadable")
    if mask is None:
        problems.append("workspace_mask unreadable")
    if intrinsic.shape != (3, 3):
        problems.append("intrinsic_matrix shape {}".format(intrinsic.shape))

    sizes = []
    for name, summary in (("rgb", rgb), ("depth", depth), ("workspace_mask", mask)):
        if summary is not None:
            sizes.append((name, summary["width"], summary["height"]))
    if len({(width, height) for _, width, height in sizes}) > 1:
        problems.append("image sizes differ: {}".format(sizes))

    saved_width = int(mat_scalar(meta, "saved_width", depth["width"] if depth else 0))
    saved_height = int(mat_scalar(meta, "saved_height", depth["height"] if depth else 0))
    if depth is not None and (saved_width, saved_height) != (depth["width"], depth["height"]):
        problems.append(
            "meta saved size {}x{} != depth {}x{}".format(
                saved_width,
                saved_height,
                depth["width"],
                depth["height"],
            )
        )

    return {
        "stem": stem,
        "rgb": rgb,
        "depth": depth,
        "workspace_mask": mask,
        "frame_id": mat_string(meta, "frame_id"),
        "geometry_space": mat_string(meta, "geometry_space"),
        "color_frame_id": mat_string(meta, "color_frame_id"),
        "depth_frame_id": mat_string(meta, "depth_frame_id"),
        "camera_info_frame_id": mat_string(meta, "camera_info_frame_id"),
        "k_scaled": int(mat_scalar(meta, "k_scaled_from_camera_info", 0)),
        "intrinsic": intrinsic,
        "problems": problems,
    }


def main():
    args = parse_args()
    camera_dir = args.camera_dir
    groups = {
        "rgb": stems(os.path.join(camera_dir, "rgb"), ".png"),
        "depth": stems(os.path.join(camera_dir, "depth"), ".png"),
        "workspace_mask": stems(os.path.join(camera_dir, "workspace_mask"), ".png"),
        "meta": stems(os.path.join(camera_dir, "meta"), ".mat"),
    }
    all_ids = set().union(*groups.values()) if groups else set()
    complete = set.intersection(*groups.values()) if groups else set()

    print("camera_dir:", camera_dir)
    for name, ids in groups.items():
        print("{}: {}".format(name, len(ids)))
    print("complete pairs:", len(complete))
    print("all ids:", len(all_ids))

    bad = False
    for name, ids in groups.items():
        missing = sorted(all_ids - ids)
        if missing:
            bad = True
            preview = ", ".join(missing[:30])
            if len(missing) > 30:
                preview += ", ..."
            print("missing {}: {}".format(name, preview))

    if complete:
        print("first complete ids:", ", ".join(sorted(complete)[:20]))
        print("last complete ids:", ", ".join(sorted(complete)[-20:]))

    details = []
    for stem in sorted(complete)[: max(0, args.max_details)]:
        details.append(check_sample(camera_dir, stem))

    for detail in details:
        print("\n[{}]".format(detail["stem"]))
        for name in ("rgb", "depth", "workspace_mask"):
            summary = detail[name]
            if summary is None:
                print("{}: unreadable".format(name))
            else:
                print(
                    "{}: {}x{} {} range=[{},{}] nonzero={}".format(
                        name,
                        summary["width"],
                        summary["height"],
                        summary["dtype"],
                        summary["min"],
                        summary["max"],
                        summary["nonzero"],
                    )
                )
        print("geometry_space:", detail["geometry_space"])
        print("frame_id:", detail["frame_id"])
        print("color_frame_id:", detail["color_frame_id"])
        print("depth_frame_id:", detail["depth_frame_id"])
        print("camera_info_frame_id:", detail["camera_info_frame_id"])
        print("k_scaled_from_camera_info:", detail["k_scaled"])
        if detail["intrinsic"].shape == (3, 3):
            print("intrinsic_matrix:")
            print(detail["intrinsic"])
        if detail["problems"]:
            bad = True
            print("problems:", "; ".join(detail["problems"]))

    if not complete and any(groups.values()):
        bad = True
        print("\nNo complete RGB/depth/meta/workspace_mask groups were found.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
