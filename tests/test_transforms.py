import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.model import transforms as tf


def test_quat_roundtrips():
    rng = np.random.default_rng(0)
    for _ in range(100):
        rot = R.random(random_state=rng).as_matrix()
        assert np.allclose(tf.quat_wxyz_to_mat(tf.mat_to_quat_wxyz(rot)), rot, atol=1e-9)
        assert np.allclose(tf.quat_xyzw_to_mat(tf.mat_to_quat_xyzw(rot)), rot, atol=1e-9)
        # wxyz vs xyzw are the same rotation only if reordered
        w = tf.mat_to_quat_wxyz(rot)
        assert np.allclose(tf.quat_xyzw_to_mat([w[1], w[2], w[3], w[0]]), rot)


def test_rpy_matches_sdf_convention():
    # SDF/URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)
    rpy = np.array([0.3, -0.4, 1.1])
    Rx = R.from_euler("x", rpy[0]).as_matrix()
    Ry = R.from_euler("y", rpy[1]).as_matrix()
    Rz = R.from_euler("z", rpy[2]).as_matrix()
    assert np.allclose(tf.rpy_to_rot(rpy), Rz @ Ry @ Rx)
    assert np.allclose(tf.rpy_to_rot(tf.rot_to_rpy(Rz @ Ry @ Rx)), Rz @ Ry @ Rx)


def test_rotvec_between_is_world_frame_premultiplied():
    rng = np.random.default_rng(1)
    Ra = R.random(random_state=rng).as_matrix()
    Rb = R.random(random_state=rng).as_matrix()
    d = tf.rotvec_between(Ra, Rb)
    assert np.allclose(R.from_rotvec(d).as_matrix() @ Ra, Rb)


def test_inv_T():
    T = tf.make_T([1, 2, 3], R.from_euler("xyz", [0.1, 0.2, 0.3]).as_matrix())
    assert np.allclose(tf.inv_T(T) @ T, np.eye(4))


def test_pose_proto_roundtrip():
    from intrinsic.math.proto import pose_pb2

    rot = R.from_euler("xyz", [0.5, -0.2, 0.9]).as_matrix()
    p = tf.pose_to_proto([0.1, 0.2, 0.3], rot, pose_pb2)
    pos2, rot2 = tf.proto_to_pose(p)
    assert np.allclose(pos2, [0.1, 0.2, 0.3]) and np.allclose(rot2, rot)
    assert abs(p.orientation.w) <= 1.0
