#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT / "python"))

import numpy as np
import scipy.io as scio
from PIL import Image

from ur3_graspnet6dof.backend import GraspNetBackend
from ur3_graspnet6dof.config import load_config
from ur3_graspnet6dof.ros_image import roi_mask


def parse_args():
    parser = argparse.ArgumentParser(description="Offline GraspNet RGB-D smoke test")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "right_arm_green_table.yaml"),
    )
    parser.add_argument("--input-dir", required=True)
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "runtime" / "offline_result.json"),
    )
    parser.add_argument(
        "--full-image",
        action="store_true",
        help="Use the whole input image instead of the configured live camera ROI.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    input_dir = Path(args.input_dir).expanduser().resolve()
    color = np.asarray(Image.open(input_dir / "color.png").convert("RGB"), dtype=np.uint8)
    depth_raw = np.asarray(Image.open(input_dir / "depth.png"))
    meta = scio.loadmat(str(input_dir / "meta.mat"))
    intrinsic = np.asarray(meta["intrinsic_matrix"], dtype=np.float64)
    factor_depth = float(np.asarray(meta["factor_depth"]).squeeze())
    depth_m = depth_raw.astype(np.float32) / factor_depth
    if color.shape[:2] != depth_m.shape:
        raise RuntimeError("color/depth dimensions differ")
    if args.full_image:
        valid = np.ones(depth_m.shape, dtype=bool)
    else:
        valid = roi_mask(depth_m.shape, config["camera"]["roi_normalized"])
    valid &= np.isfinite(depth_m)
    valid &= depth_m >= float(config["camera"]["min_depth_m"])
    valid &= depth_m <= float(config["camera"]["max_depth_m"])

    backend = GraspNetBackend(config)
    candidates, diagnostics = backend.infer(color, depth_m, intrinsic, valid)
    result = {"diagnostics": diagnostics, "candidates": candidates}
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("candidates:", len(candidates))
    print("diagnostics:", diagnostics)
    print("output:", output)
    if not candidates:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

