"""Frame/quaternion conventions used across the bridge (all explicit, all tested).

  * MuJoCo: quaternions wxyz; body_xmat row-major 3x3.
  * LIBERO observations (`<obj>_quat`, `robot0_eef_quat`): xyzw.
  * Intrinsic protos (intrinsic_proto.Pose): position {x,y,z}, orientation {x,y,z,w}.
  * SDF poses: x y z roll pitch yaw (extrinsic XYZ / intrinsic ZYX).
  * robosuite OSC_POSE rotation action: axis-angle delta applied in the world frame
    (goal_R = R(delta) @ current_R).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


def quat_wxyz_to_mat(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()


def quat_xyzw_to_mat(q) -> np.ndarray:
    return R.from_quat(np.asarray(q, dtype=float)).as_matrix()


def mat_to_quat_wxyz(rot) -> np.ndarray:
    x, y, z, w = R.from_matrix(rot).as_quat()
    return np.array([w, x, y, z])


def mat_to_quat_xyzw(rot) -> np.ndarray:
    return R.from_matrix(rot).as_quat()


def make_T(pos, rot) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = pos
    return T


def inv_T(T) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def rot_error_deg(Ra, Rb) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def rotvec_between(R_from, R_to) -> np.ndarray:
    """Axis-angle (world frame) delta d such that R(d) @ R_from == R_to."""
    return R.from_matrix(R_to @ R_from.T).as_rotvec()


# ---------------------------------------------------------------- Intrinsic protos
def pose_to_proto(pos, rot, pose_pb2):
    p = pose_pb2.Pose()
    p.position.x, p.position.y, p.position.z = [float(v) for v in pos]
    x, y, z, w = R.from_matrix(rot).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = float(x), float(y), float(z), float(w)
    return p


def proto_to_pose(p):
    pos = np.array([p.position.x, p.position.y, p.position.z])
    q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    n = np.linalg.norm(q)
    if n < 1e-9:  # a default/empty quaternion in a proto means identity
        rot = np.eye(3)
    else:
        rot = R.from_quat(q / n).as_matrix()
    return pos, rot


# ---------------------------------------------------------------- SDF
def rot_to_rpy(rot) -> np.ndarray:
    return R.from_matrix(rot).as_euler("xyz")


def rpy_to_rot(rpy) -> np.ndarray:
    return R.from_euler("xyz", rpy).as_matrix()
