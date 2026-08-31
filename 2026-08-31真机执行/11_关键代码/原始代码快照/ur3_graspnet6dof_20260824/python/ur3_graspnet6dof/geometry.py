import math

import numpy as np


def normalize(vector):
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("cannot normalize a zero or non-finite vector")
    return vector / norm


def quaternion_to_matrix(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def normalize_quaternion(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    return q / norm


def matrix_to_quaternion(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
            w = (matrix[2, 1] - matrix[1, 2]) / s
        elif index == 1:
            s = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
            w = (matrix[0, 2] - matrix[2, 0]) / s
        else:
            s = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s
            w = (matrix[1, 0] - matrix[0, 1]) / s
    return normalize_quaternion([x, y, z, w])


def transform_matrix(translation, quaternion):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = quaternion_to_matrix(quaternion)
    result[:3, 3] = np.asarray(translation, dtype=np.float64)
    return result


def transform_point(matrix, point):
    point = np.asarray(point, dtype=np.float64)
    return matrix[:3, :3].dot(point) + matrix[:3, 3]


def grasp_to_tool_rotation(grasp_rotation, opening_axis_flip=False):
    """Map GraspNet axes (+X approach, +Y opening) to UR tool axes.

    The Robotiq model used here extends along tool +Z and opens along tool +Y.
    Consequently tool Z = grasp X, tool Y = grasp Y and tool X = -grasp Z.
    Reversing tool X/Y is the physically equivalent 180-degree wrist variant.
    """
    grasp_rotation = np.asarray(grasp_rotation, dtype=np.float64).reshape(3, 3)
    approach = normalize(grasp_rotation[:, 0])
    opening = normalize(grasp_rotation[:, 1])
    if opening_axis_flip:
        opening = -opening
    tool_x = normalize(np.cross(opening, approach))
    tool_y = normalize(opening)
    tool_z = normalize(approach)
    result = np.column_stack((tool_x, tool_y, tool_z))
    if np.linalg.det(result) < 0.999:
        raise ValueError("grasp-to-tool rotation is not right handed")
    return result


def plane_point(plane):
    plane = np.asarray(plane, dtype=np.float64)
    normal = plane[:3]
    denom = float(normal.dot(normal))
    if denom < 1e-12:
        raise ValueError("invalid plane")
    return -plane[3] * normal / denom


def point_plane_distance(point, plane):
    plane = np.asarray(plane, dtype=np.float64)
    return abs(float(np.dot(plane[:3], point) + plane[3])) / float(np.linalg.norm(plane[:3]))

