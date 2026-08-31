import sys

import numpy as np


def _native_dtype(dtype, is_bigendian):
    dtype = np.dtype(dtype)
    if dtype.itemsize == 1:
        return dtype
    message_order = ">" if is_bigendian else "<"
    native_order = ">" if sys.byteorder == "big" else "<"
    if message_order == native_order:
        return dtype.newbyteorder("=")
    return dtype.newbyteorder(message_order)


def _rows_from_message(message, dtype, values_per_pixel):
    dtype = _native_dtype(dtype, bool(message.is_bigendian))
    row_values = int(message.step) // dtype.itemsize
    flat = np.frombuffer(message.data, dtype=dtype)
    expected = int(message.height) * row_values
    if flat.size < expected:
        raise ValueError("image data is shorter than height*step")
    rows = flat[:expected].reshape(int(message.height), row_values)
    width_values = int(message.width) * values_per_pixel
    return rows[:, :width_values]


def decode_color(message):
    encoding = message.encoding.lower()
    if encoding in ("rgb8", "bgr8"):
        rows = _rows_from_message(message, np.uint8, 3)
        image = rows.reshape(message.height, message.width, 3).copy()
        if encoding == "bgr8":
            image = image[:, :, ::-1].copy()
        return image
    if encoding in ("rgba8", "bgra8"):
        rows = _rows_from_message(message, np.uint8, 4)
        image = rows.reshape(message.height, message.width, 4)[:, :, :3].copy()
        if encoding == "bgra8":
            image = image[:, :, ::-1].copy()
        return image
    if encoding in ("mono8", "8uc1"):
        rows = _rows_from_message(message, np.uint8, 1)
        gray = rows.reshape(message.height, message.width).copy()
        return np.repeat(gray[:, :, None], 3, axis=2)
    raise ValueError("unsupported color encoding: {}".format(message.encoding))


def decode_depth_metres(message):
    encoding = message.encoding.lower()
    if encoding in ("16uc1", "mono16"):
        rows = _rows_from_message(message, np.uint16, 1)
        return rows.reshape(message.height, message.width).astype(np.float32) * 0.001
    if encoding in ("32fc1",):
        rows = _rows_from_message(message, np.float32, 1)
        return rows.reshape(message.height, message.width).astype(np.float32)
    if encoding in ("64fc1",):
        rows = _rows_from_message(message, np.float64, 1)
        return rows.reshape(message.height, message.width).astype(np.float32)
    raise ValueError("unsupported depth encoding: {}".format(message.encoding))


def intrinsic_matrix(camera_info):
    matrix = np.asarray(camera_info.K, dtype=np.float64).reshape(3, 3)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_info contains invalid focal lengths")
    return matrix


def roi_mask(shape, normalized_bounds):
    height, width = shape[:2]
    if len(normalized_bounds) != 4:
        raise ValueError("roi_normalized must contain four values")
    x0, y0, x1, y1 = [float(value) for value in normalized_bounds]
    if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
        raise ValueError("roi_normalized values must satisfy 0<=min<max<=1")
    ix0, ix1 = int(round(x0 * width)), int(round(x1 * width))
    iy0, iy1 = int(round(y0 * height)), int(round(y1 * height))
    result = np.zeros((height, width), dtype=bool)
    result[iy0:iy1, ix0:ix1] = True
    return result

