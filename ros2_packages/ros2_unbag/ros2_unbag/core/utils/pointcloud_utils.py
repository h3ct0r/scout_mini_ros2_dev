import struct

import numpy as np
from pypcd4 import PointCloud
from pypcd4.pointcloud2 import build_dtype_from_msg
from sensor_msgs.msg import PointCloud2


def quaternion_matrix(quaternion):
    """
    Compute a 4×4 transformation matrix from a quaternion [x, y, z, w].

    Args:
        quaternion: Sequence of 4 floats [x, y, z, w].

    Returns:
        numpy.ndarray: 4x4 transformation matrix.
    """
    x, y, z, w = quaternion
    N = x * x + y * y + z * z + w * w
    if N < np.finfo(float).eps:
        return np.eye(4)
    s = 2.0 / N
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s

    M = np.eye(4)
    M[0, 0] = 1 - (yy + zz)
    M[0, 1] = xy - wz
    M[0, 2] = xz + wy
    M[1, 0] = xy + wz
    M[1, 1] = 1 - (xx + zz)
    M[1, 2] = yz - wx
    M[2, 0] = xz - wy
    M[2, 1] = yz + wx
    M[2, 2] = 1 - (xx + yy)
    return M


def apply_pointcloud_transform(msg, translation, rotation):
    """
    Apply a rigid-body transform to all points in a PointCloud2 message.

    Args:
        msg: PointCloud2 message instance.
        translation: Iterable of 3 floats [x, y, z].
        rotation: Iterable of 4 floats [x, y, z, w] quaternion.

    Returns:
        PointCloud2: Transformed PointCloud2 message.

    Raises:
        ValueError: If message fields are missing or inputs are malformed.
    """
    translation = np.asarray(translation, dtype=float).reshape(-1)
    rotation = np.asarray(rotation, dtype=float).reshape(-1)
    if translation.size != 3:
        raise ValueError("Translation must have exactly three elements [x, y, z]")
    if rotation.size != 4:
        raise ValueError("Rotation must have exactly four elements [x, y, z, w]")

    transform_matrix = quaternion_matrix(rotation)
    transform_matrix[0:3, 3] = translation

    offsets = {}
    for field in msg.fields:
        if field.name in ('x', 'y', 'z'):
            offsets[field.name] = field.offset

    if not all(k in offsets for k in ('x', 'y', 'z')):
        raise ValueError("PointCloud2 message does not contain x, y, z fields")

    x_off = offsets['x']
    y_off = offsets['y']
    z_off = offsets['z']

    data = bytearray(msg.data)  # mutable copy

    for i in range(0, len(data), msg.point_step):
        x = struct.unpack_from('f', data, i + x_off)[0]
        y = struct.unpack_from('f', data, i + y_off)[0]
        z = struct.unpack_from('f', data, i + z_off)[0]

        point = np.array([x, y, z, 1.0])
        transformed = transform_matrix @ point

        struct.pack_into('f', data, i + x_off, transformed[0])
        struct.pack_into('f', data, i + y_off, transformed[1])
        struct.pack_into('f', data, i + z_off, transformed[2])

    transformed_msg = PointCloud2()
    transformed_msg.header = msg.header
    transformed_msg.height = msg.height
    transformed_msg.width = msg.width
    transformed_msg.fields = msg.fields
    transformed_msg.is_bigendian = msg.is_bigendian
    transformed_msg.point_step = msg.point_step
    transformed_msg.row_step = msg.row_step
    transformed_msg.is_dense = msg.is_dense
    transformed_msg.data = bytes(data)

    return transformed_msg


def convert_pointcloud2_to_pypcd(msg):
    """Convert a PointCloud2 message to a Pypcd PointCloud object.
    Args:
        msg (sensor_msgs.msg.PointCloud2): PointCloud2 message instance.
    Returns:
        pypcd4.PointCloud: Pypcd PointCloud object.
    """

    # Build dtype from message fields
    dtype_fields = build_dtype_from_msg(msg)
    dtype = np.dtype(dtype_fields)

    # Get field names and types
    field_names = tuple(f.name for f in msg.fields)
    np_types = tuple(dtype[name].type for name in field_names)
    structured_array = np.frombuffer(msg.data, dtype=dtype)
    points_np = np.vstack([structured_array[name] for name in field_names]).T

    # Build point cloud
    pc = PointCloud.from_points(points_np, field_names, np_types)

    return pc
