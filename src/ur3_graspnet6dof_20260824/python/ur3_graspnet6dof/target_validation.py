"""Pure helpers for checking whether a frozen image target has moved."""

import numpy as np


def nearest_target(objects, category, frozen_pixel):
    candidates = [
        item
        for item in objects
        if item.get("pickable", False) and item.get("category") == category
    ]
    if not candidates:
        return None, None
    frozen = np.asarray(frozen_pixel, dtype=float)
    nearest = min(
        candidates,
        key=lambda item: float(
            np.linalg.norm(np.asarray(item["center"], dtype=float) - frozen)
        ),
    )
    distance = float(
        np.linalg.norm(np.asarray(nearest["center"], dtype=float) - frozen)
    )
    return nearest, distance
