"""Small temporal stabilizer for frame-by-frame image classifications."""

import copy

import numpy as np


class DetectionStabilizer:
    def __init__(
        self,
        maximum_distance=55.0,
        smoothing=0.45,
        confirmation_frames=2,
        category_switch_frames=4,
        maximum_missed_frames=2,
    ):
        self.maximum_distance = float(maximum_distance)
        self.smoothing = float(smoothing)
        self.confirmation_frames = int(confirmation_frames)
        self.category_switch_frames = int(category_switch_frames)
        self.maximum_missed_frames = int(maximum_missed_frames)
        self.tracks = []
        self.next_track_id = 1

    @staticmethod
    def _distance(first, second):
        return float(
            np.linalg.norm(
                np.asarray(first["center"], dtype=float)
                - np.asarray(second["center"], dtype=float)
            )
        )

    def _new_track(self, detection):
        track = {
            "id": self.next_track_id,
            "detection": copy.deepcopy(detection),
            "hits": 1,
            "misses": 0,
            "pending_category": None,
            "pending_count": 0,
        }
        self.next_track_id += 1
        return track

    def _update_track(self, track, current):
        stable = track["detection"]
        alpha = self.smoothing
        stable["center"] = (
            (1.0 - alpha) * np.asarray(stable["center"], dtype=float)
            + alpha * np.asarray(current["center"], dtype=float)
        ).tolist()
        stable["bbox"] = [
            int(round(value))
            for value in (
                (1.0 - alpha) * np.asarray(stable["bbox"], dtype=float)
                + alpha * np.asarray(current["bbox"], dtype=float)
            )
        ]
        if current["category"] == stable["category"]:
            track["pending_category"] = None
            track["pending_count"] = 0
        else:
            if track["pending_category"] == current["category"]:
                track["pending_count"] += 1
            else:
                track["pending_category"] = current["category"]
                track["pending_count"] = 1
            if track["pending_count"] >= self.category_switch_frames:
                for key in ("category", "category_zh", "priority"):
                    if key in current:
                        stable[key] = copy.deepcopy(current[key])
                track["pending_category"] = None
                track["pending_count"] = 0
        for key in ("major_axis_image", "exclusion_reason"):
            if key in current:
                stable[key] = copy.deepcopy(current[key])
        track["hits"] += 1
        track["misses"] = 0

    def update(self, detections):
        pairs = []
        for track_index, track in enumerate(self.tracks):
            old = track["detection"]
            for detection_index, current in enumerate(detections):
                if bool(old.get("pickable")) != bool(current.get("pickable")):
                    continue
                # Excluded regions have distinct semantic roles and must never
                # inherit one another's category merely because boxes overlap.
                if not old.get("pickable") and old["category"] != current["category"]:
                    continue
                distance = self._distance(old, current)
                if distance <= self.maximum_distance:
                    pairs.append((distance, track_index, detection_index))
        matched_tracks = set()
        matched_detections = set()
        for _, track_index, detection_index in sorted(pairs):
            if track_index in matched_tracks or detection_index in matched_detections:
                continue
            self._update_track(self.tracks[track_index], detections[detection_index])
            matched_tracks.add(track_index)
            matched_detections.add(detection_index)
        retained = []
        for index, track in enumerate(self.tracks):
            if index not in matched_tracks:
                track["misses"] += 1
            if track["misses"] <= self.maximum_missed_frames:
                retained.append(track)
        self.tracks = retained
        for index, detection in enumerate(detections):
            if index not in matched_detections:
                self.tracks.append(self._new_track(detection))
        output = []
        for track in self.tracks:
            if track["hits"] < self.confirmation_frames:
                continue
            item = copy.deepcopy(track["detection"])
            item["track_id"] = track["id"]
            item["temporally_stabilized"] = True
            output.append(item)
        output.sort(key=lambda item: (not item["pickable"], item["priority"]))
        return output
