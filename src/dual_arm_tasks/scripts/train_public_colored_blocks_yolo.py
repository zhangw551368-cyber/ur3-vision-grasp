#!/usr/bin/env python3

import argparse
import os
import shutil
import sys


def parse_args():
    parser = argparse.ArgumentParser(
        description="Train a YOLO colored-block detector and copy best.pt to the grasp pipeline path."
    )
    parser.add_argument("--data", required=True, help="YOLO dataset data.yaml.")
    parser.add_argument("--model", default="yolov8n.pt", help="Base model, e.g. yolov8n.pt.")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    parser.add_argument(
        "--project", default="/home/gzu/gzu_ws/models/yolo", help="Ultralytics project dir."
    )
    parser.add_argument("--name", default="colored_blocks_red")
    parser.add_argument(
        "--output",
        default="/home/gzu/gzu_ws/src/ultralytics_ros/models/red_block.pt",
        help="Where to copy trained best.pt.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not os.path.exists(args.data):
        raise RuntimeError("data.yaml does not exist: {}".format(args.data))

    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise RuntimeError(
            "Cannot import ultralytics. Run with /home/gzu/anaconda3/bin/python3: {}".format(
                exc
            )
        )

    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=args.project,
        name=args.name,
    )

    save_dir = getattr(results, "save_dir", None)
    if save_dir is None:
        save_dir = os.path.join(args.project, args.name)
    best = os.path.join(str(save_dir), "weights", "best.pt")
    if not os.path.exists(best):
        raise RuntimeError("Training finished but best.pt was not found: {}".format(best))

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    shutil.copy2(best, args.output)
    print("Copied trained YOLO model:")
    print("  from: {}".format(best))
    print("  to:   {}".format(args.output))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
