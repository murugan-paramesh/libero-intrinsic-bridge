"""Thin, explicit wrapper around the LIBERO benchmark environment.

Facts verified from the pinned sources (see docs/contract.md):
  * env.step(action) with action in R^7: a[0:3] = delta position (scaled to +-0.05 m per
    20 Hz step), a[3:6] = delta rotation axis-angle in the WORLD frame (scaled to +-0.5 rad),
    a[6] = gripper (-1 open, +1 close; only the sign is used). Controller: robosuite OSC_POSE.
  * `done` returned by step() is the official goal check (BDDLBaseDomain.step overrides it
    with self._check_success()), not a timeout.
  * robot0_eef_pos is the world position of site gripper0_grip_site; robot0_eef_quat is the
    orientation of body robot0_right_hand in xyzw order. We instead read the grip-site frame
    directly from MuJoCo (site_xpos / site_xmat) so position and orientation refer to the
    same frame (the TCP used everywhere in this project).
  * Object poses come from MuJoCo body_xpos / body_xquat (wxyz).
"""
from __future__ import annotations

import dataclasses
import os
from typing import Dict, List, Optional

import numpy as np

from libero_intrinsic.env.init_states import load_init_states

TCP_SITE = "gripper0_grip_site"
ARM_JOINTS = [f"robot0_joint{i}" for i in range(1, 8)]
FINGER_JOINTS = ["gripper0_finger_joint1", "gripper0_finger_joint2"]
ROBOT_BASE_BODY = "robot0_base"
HAND_BODY = "robot0_right_hand"
SUITE_NAME = "libero_10"


@dataclasses.dataclass
class TaskInfo:
    index: int
    name: str
    language: str
    bddl_path: str
    init_states_path: str
    problem_folder: str


def language_from_task_name(name: str) -> str:
    """Re-implementation of libero.libero.benchmark.grab_language_from_filename
    (the upstream module imports torch at import time, which we do not install)."""
    x = name if name.endswith(".bddl") else name + ".bddl"
    if x[0].isupper():
        off = 8 if "SCENE10" in x else 7
        language = " ".join(x[x.find("SCENE") + off:].split("_"))
    else:
        language = " ".join(x.split("_"))
    return language[: language.find(".bddl")]


def list_tasks() -> List[TaskInfo]:
    """Enumerate LIBERO-10 in the official order.

    Mirrors libero.libero.benchmark.LIBERO_10 with task_order_index=0 (identity order):
    task names come from libero_suite_task_map.libero_task_map["libero_10"], the bddl file
    is <name>.bddl and the init file <name>.pruned_init in problem folder "libero_10".
    """
    import importlib.util

    from libero.libero import get_libero_path

    # Load the task map module by path: importing libero.libero.benchmark would import torch.
    map_path = os.path.join(get_libero_path("benchmark_root"), "benchmark", "libero_suite_task_map.py")
    spec = importlib.util.spec_from_file_location("libero_suite_task_map", map_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    names = mod.libero_task_map[SUITE_NAME]
    out = []
    for i, name in enumerate(names):
        out.append(
            TaskInfo(
                index=i,
                name=name,
                language=language_from_task_name(name),
                bddl_path=os.path.join(get_libero_path("bddl_files"), SUITE_NAME, name + ".bddl"),
                init_states_path=os.path.join(get_libero_path("init_states"), SUITE_NAME, name + ".pruned_init"),
                problem_folder=SUITE_NAME,
            )
        )
    return out


@dataclasses.dataclass
class RobotState:
    q: np.ndarray  # 7 arm joint positions (rad), order robot0_joint1..7
    qd: np.ndarray
    finger_q: np.ndarray  # 2 finger joint positions
    tcp_pos: np.ndarray  # world position of gripper0_grip_site
    tcp_rot: np.ndarray  # 3x3 world rotation of gripper0_grip_site
    base_pos: np.ndarray
    base_rot: np.ndarray


class LiberoEnv:
    """Owns one LIBERO OffScreenRenderEnv for one task."""

    def __init__(self, task: TaskInfo, camera_hw=(256, 256), seed: int = 0, horizon: int = 100000,
                 use_camera_obs: bool = False):
        from libero.libero.envs import OffScreenRenderEnv

        self.task = task
        self.camera_hw = camera_hw
        # horizon is set large so that robosuite's own timeout never raises
        # "executing action in terminated episode"; our own episode budget is enforced by
        # the runner. use_camera_obs=False removes per-step image rendering (26 ms vs 170 ms per
        # step on this CPU); it changes observations only, not physics or control. Video frames
        # are rendered on demand with sim.render(). Every other setting is the LIBERO default
        # (OSC_POSE, 20 Hz, Panda).
        self.env = OffScreenRenderEnv(
            bddl_file_name=task.bddl_path,
            camera_heights=camera_hw[0],
            camera_widths=camera_hw[1],
            horizon=horizon,
            use_camera_obs=use_camera_obs,
        )
        self.env.seed(seed)
        self.sim = None
        self.init_states = load_init_states(task.init_states_path)
        self._last_obs = None
        self.step_count = 0

    # ------------------------------------------------------------------ lifecycle
    def reset_to(self, init_state_index: int, settle_steps: int = 5):
        """Official reset: env.reset() then set_init_state(init_states[i]) then a few
        zero actions (the same recipe as libero/lifelong/metric.py)."""
        self.env.reset()
        self.sim = self.env.env.sim
        obs = self.env.set_init_state(self.init_states[init_state_index])
        for _ in range(settle_steps):
            obs, _, _, _ = self.env.step(np.zeros(7))
        self._last_obs = obs
        self.step_count = 0
        return obs

    def close(self):
        self.env.close()

    # ------------------------------------------------------------------ stepping
    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=np.float64).reshape(7)
        obs, reward, done, info = self.env.step(action)
        self._last_obs = obs
        self.step_count += 1
        return obs, reward, bool(done), info

    def check_success(self) -> bool:
        return bool(self.env.check_success())

    # ------------------------------------------------------------------ state access
    @property
    def model(self):
        return self.sim.model

    @property
    def data(self):
        return self.sim.data

    def robot_state(self) -> RobotState:
        d, m = self.sim.data, self.sim.model
        q = np.array([d.get_joint_qpos(j) for j in ARM_JOINTS], dtype=np.float64)
        qd = np.array([d.get_joint_qvel(j) for j in ARM_JOINTS], dtype=np.float64)
        fq = np.array([d.get_joint_qpos(j) for j in FINGER_JOINTS], dtype=np.float64)
        return RobotState(
            q=q,
            qd=qd,
            finger_q=fq,
            tcp_pos=np.array(d.get_site_xpos(TCP_SITE)),
            tcp_rot=np.array(d.get_site_xmat(TCP_SITE)).reshape(3, 3),
            base_pos=np.array(d.get_body_xpos(ROBOT_BASE_BODY)),
            base_rot=np.array(d.get_body_xmat(ROBOT_BASE_BODY)).reshape(3, 3),
        )

    def joint_limits(self) -> np.ndarray:
        m = self.sim.model
        return np.array([m.jnt_range[m.joint_name2id(j)] for j in ARM_JOINTS])

    def body_pose(self, body: str):
        """World pose (pos, 3x3 rot) of a MuJoCo body."""
        d = self.sim.data
        return np.array(d.get_body_xpos(body)), np.array(d.get_body_xmat(body)).reshape(3, 3)

    def body_quat_wxyz(self, body: str) -> np.ndarray:
        return np.array(self.sim.data.get_body_xquat(body))

    def joint_qpos(self, joint: str) -> float:
        return float(self.sim.data.get_joint_qpos(joint))

    def object_names(self) -> List[str]:
        return list(self.env.env.objects_dict.keys())

    def fixture_names(self) -> List[str]:
        return list(self.env.env.fixtures_dict.keys())

    def object_root_body(self, obj_name: str) -> str:
        e = self.env.env
        o = e.objects_dict.get(obj_name) or e.fixtures_dict.get(obj_name)
        return o.root_body

    def object_site_pose(self, site: str):
        d = self.sim.data
        return np.array(d.get_site_xpos(site)), np.array(d.get_site_xmat(site)).reshape(3, 3)

    def site_size(self, site: str) -> np.ndarray:
        m = self.sim.model
        return np.array(m.site_size[m.site_name2id(site)])

    # ------------------------------------------------------------------ contacts
    def contacts(self):
        """List of (geom1_name, geom2_name, dist) for all active contacts."""
        d, m = self.sim.data, self.sim.model
        out = []
        for i in range(d.ncon):
            c = d.contact[i]
            out.append((m.geom_id2name(c.geom1), m.geom_id2name(c.geom2), float(c.dist)))
        return out

    def gripper_contacts_with(self, body_root: str) -> int:
        """Count contacts between finger-pad geoms and any geom of body subtree `body_root`."""
        d, m = self.sim.data, self.sim.model
        root_id = m.body_name2id(body_root)

        def in_subtree(bid):
            while bid > 0:
                if bid == root_id:
                    return True
                bid = m.body_parentid[bid]
            return bid == root_id

        pads = {m.geom_name2id(n) for n in ["gripper0_finger1_pad_collision", "gripper0_finger2_pad_collision"]}
        n = 0
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if (g1 in pads and in_subtree(m.geom_bodyid[g2])) or (g2 in pads and in_subtree(m.geom_bodyid[g1])):
                n += 1
        return n

    # ------------------------------------------------------------------ rendering
    def render_frame(self, camera: str = "agentview") -> np.ndarray:
        """Render an upright RGB frame on demand (robosuite's offscreen buffer is vertically flipped)."""
        img = self.sim.render(width=self.camera_hw[1], height=self.camera_hw[0], camera_name=camera)
        return np.ascontiguousarray(img[::-1])
