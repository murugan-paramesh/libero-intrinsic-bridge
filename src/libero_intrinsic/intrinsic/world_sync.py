"""Keep the Intrinsic world in step with the LIBERO/MuJoCo scene.

The SDF world (model/scene_to_sdf.py) contains one static object per MuJoCo body that has
collision geometry, named by the MuJoCo body name, plus the kinematic robot "panda" with frame
"tcp". Before every planning request we push the current world pose of every such body into
Intrinsic (ObjectWorldService.UpdateTransform) and the current joint vector
(UpdateObjectJoints). Grasped objects are re-parented to the tcp frame (ReparentObject) so the
planner treats them as part of the moving robot; on release they are re-parented to root at
their observed pose. Nothing is ever removed from the collision world; intentional contacts are
declared per request through CollisionSettings rules (see grasp collision rules in skills).
"""
from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from libero_intrinsic.env.libero_env import LiberoEnv
from libero_intrinsic.intrinsic.client import IntrinsicClient, collision_settings
from libero_intrinsic.model import transforms as tf
from libero_intrinsic.model.scene_to_sdf import SdfWorldSpec


@dataclasses.dataclass
class SyncReport:
    n_objects: int
    max_pose_error_m: float   # |Intrinsic root_t_obj - MuJoCo| after sync (read back)
    max_rot_error_deg: float


class WorldSync:
    def __init__(self, env: LiberoEnv, client: IntrinsicClient, spec: SdfWorldSpec):
        self.env = env
        self.client = client
        self.spec = spec
        self.bodies: List[str] = [b.body for b in spec.exported_bodies]
        self.attached: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}  # body -> tcp_t_obj at grasp
        client.ensure_tcp_frame(spec.tcp_link, spec.tcp_link_t_tcp_pos, spec.tcp_link_t_tcp_rot)
        # set link_t_tcp explicitly (see scene_to_sdf.py for why the SDF value is not trusted)
        client.set_tcp_frame_offset(spec.tcp_link, spec.tcp_link_t_tcp_pos, spec.tcp_link_t_tcp_rot)
        self._intrinsic_objects = set(client.list_objects())
        missing = [b for b in self.bodies if b not in self._intrinsic_objects]
        if missing:
            raise RuntimeError(f"objects missing in Intrinsic world {client.world_id}: {missing}")

    # ------------------------------------------------------------------ pose push
    def sync(self, verify: bool = False) -> SyncReport:
        rs = self.env.robot_state()
        self.client.update_robot_joints(rs.q)
        errs, rerrs = [], []
        for body in self.bodies:
            if body in self.attached:
                continue  # its pose is defined by the robot FK while attached
            pos, rot = self.env.body_pose(body)
            self.client.set_object_world_pose(body, pos, rot)
            if verify:
                p2, r2 = self.client.get_transform("root", body)
                errs.append(np.linalg.norm(p2 - pos))
                rerrs.append(tf.rot_error_deg(r2, rot))
        return SyncReport(len(self.bodies), float(max(errs)) if errs else 0.0, float(max(rerrs)) if rerrs else 0.0)

    # ------------------------------------------------------------------ attachment
    def attach(self, body: str):
        """Declare `body` grasped: re-parent to the tcp frame keeping its world pose."""
        pos, rot = self.env.body_pose(body)
        self.client.set_object_world_pose(body, pos, rot)
        self.client.reparent_to_frame(body, self.client.robot, self.client.tcp_frame)
        rs = self.env.robot_state()
        tcp_T = tf.make_T(rs.tcp_pos, rs.tcp_rot)
        obj_T = tf.make_T(pos, rot)
        rel = tf.inv_T(tcp_T) @ obj_T
        self.attached[body] = (rel[:3, 3].copy(), rel[:3, :3].copy())

    def detach(self, body: str):
        if body not in self.attached:
            return
        self.client.reparent_to_root(body)
        pos, rot = self.env.body_pose(body)
        self.client.set_object_world_pose(body, pos, rot)
        del self.attached[body]

    def attached_bodies(self) -> List[str]:
        return list(self.attached.keys())

    # ------------------------------------------------------------------ collision settings
    def grasp_collision_settings(self, target_body: str, extra_pairs: Sequence[Tuple[str, str]] = (),
                                 margin: Optional[float] = None):
        """Rules for the approach/grasp phase: contact between the robot (gripper) and the
        object about to be grasped is intentional; everything else is checked. While an
        object is attached, contact between it and the robot is intentional too (the fingers
        squeeze it), and contact between it and its support surface at release is intentional."""
        pairs = [(self.client.robot, target_body)] + [(self.client.robot, b) for b in self.attached] + list(extra_pairs)
        return collision_settings(pairs, minimum_margin=margin, resolver=self.client.oref)

    def transport_collision_settings(self, support_bodies: Sequence[str] = (), margin: Optional[float] = None):
        pairs = [(self.client.robot, b) for b in self.attached]
        for b in self.attached:
            for s in support_bodies:
                pairs.append((b, s))
        return collision_settings(pairs, minimum_margin=margin, resolver=self.client.oref)

    def free_collision_settings(self, margin: Optional[float] = None):
        return collision_settings([(self.client.robot, b) for b in self.attached], minimum_margin=margin, resolver=self.client.oref)
