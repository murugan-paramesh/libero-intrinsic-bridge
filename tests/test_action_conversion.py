import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.env.executor import POS_SCALE, ROT_SCALE, CONTROL_DT
from libero_intrinsic.model import transforms as tf
from libero_intrinsic.intrinsic.client import Trajectory


def test_osc_scaling_constants_match_robosuite_osc_pose_json():
    import json, os, robosuite
    cfg = json.load(open(os.path.join(os.path.dirname(robosuite.__file__), "controllers", "config", "osc_pose.json")))
    assert cfg["output_max"] == [POS_SCALE] * 3 + [ROT_SCALE] * 3
    assert cfg["control_delta"] is True and cfg["uncouple_pos_ori"] is True
    assert abs(CONTROL_DT - 1 / 20) < 1e-12


def test_delta_rotation_action_is_world_frame_premultiply():
    # robosuite: goal_ori = R(axisangle(a[3:6]*ROT_SCALE)) @ ee_ori
    Rc = R.from_euler("xyz", [0.2, -0.1, 0.4]).as_matrix()
    Rt = R.from_euler("xyz", [0.25, -0.05, 0.5]).as_matrix()
    d = tf.rotvec_between(Rc, Rt)
    a = np.clip(d / ROT_SCALE, -1, 1)
    assert np.allclose(R.from_rotvec(a * ROT_SCALE).as_matrix() @ Rc, Rt, atol=1e-9)


def test_trajectory_sampling():
    t = np.array([0.0, 1.0, 2.0])
    q = np.array([[0.0] * 7, [1.0] * 7, [3.0] * 7])
    tr = Trajectory("id", t, q, np.zeros_like(q), np.zeros_like(q), 0.1)
    assert np.allclose(tr.sample(-1), q[0]) and np.allclose(tr.sample(5), q[2])
    assert np.allclose(tr.sample(0.5), [0.5] * 7) and np.allclose(tr.sample(1.5), [2.0] * 7)
    assert tr.duration == 2.0
