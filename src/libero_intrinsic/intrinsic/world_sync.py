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
        self.fingers: Dict[str, str] = dict(spec.finger_objects or {})  # finger body -> object
        client.joint_limits = env.joint_limits()
        client.ensure_tcp_frame(spec.tcp_link, spec.tcp_link_t_tcp_pos, spec.tcp_link_t_tcp_rot)
        # set link_t_tcp explicitly (see scene_to_sdf.py for why the SDF value is not trusted)
        client.set_tcp_frame_offset(spec.tcp_link, spec.tcp_link_t_tcp_pos, spec.tcp_link_t_tcp_rot)
        self._intrinsic_objects = set(client.list_objects())
        missing = [b for b in self.bodies + list(self.fingers.values()) if b not in self._intrinsic_objects]
        if missing:
            raise RuntimeError(f"objects missing in Intrinsic world {client.world_id}: {missing}")
        # fingers ride on the flange frame; their offset follows the finger joints (see sync)
        for obj in self.fingers.values():
            client.reparent_to_frame(obj, client.robot, client.tcp_frame)
        self._sync_fingers()

    def _sync_fingers(self):
        rs = self.env.robot_state()
        tcp_inv = tf.inv_T(tf.make_T(rs.tcp_pos, rs.tcp_rot))
        for fbody, obj in self.fingers.items():
            rel = tcp_inv @ tf.make_T(*self.env.body_pose(fbody))
            self.client.set_frame_relative_pose(self.client.robot, self.client.tcp_frame, obj, rel[:3, 3], rel[:3, :3])

    def resting_pairs(self):
        """Every movable body vs the body it currently rests on: physically resting objects
        penetrate their support by a fraction of a millimetre in MuJoCo, which the exact
        collision checker would otherwise report as an invalid world state."""
        from libero_intrinsic.skills import geometry as geo
        pairs = []
        boxes = {b: geo.object_box(self.env, b) for b in self.bodies}
        for b, box in boxes.items():
            if b in self.attached:
                continue
            for s, sb in boxes.items():
                if s == b:
                    continue
                if -0.006 <= box.bottom_z - sb.top_z <= 0.01 and np.all(np.abs(sb.center_world[:2] - box.center_world[:2]) <= sb.half_extents_world[:2] + box.half_extents_world[:2]):
                    pairs.append((b, s))
        return pairs

    def _base_pairs(self):
        """Contacts that are always intentional: fingers vs the robot (they are part of it),
        fingers vs each other, fingers/robot vs an attached (grasped) object, and every
        resting object vs its support."""
        r = self.client.robot
        f = list(self.fingers.values())
        pairs = [(r, x) for x in f] + [(f[0], f[1])] if len(f) == 2 else [(r, x) for x in f]
        for b in self.attached:
            pairs += [(r, b)] + [(x, b) for x in f]
        return pairs + self.resting_pairs()

    # ------------------------------------------------------------------ pose push
    def sync(self, verify: bool = False) -> SyncReport:
        rs = self.env.robot_state()
        self.client.update_robot_joints(rs.q)
        self._sync_fingers()
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

    def refresh_attachment(self, body: str) -> float:
        """Re-measure tcp_t_obj for an attached object (the object may have slipped or rotated in
        the grasp) and update the Intrinsic world accordingly. Returns the position drift (m)."""
        if body not in self.attached:
            return 0.0
        rs = self.env.robot_state()
        rel = tf.inv_T(tf.make_T(rs.tcp_pos, rs.tcp_rot)) @ tf.make_T(*self.env.body_pose(body))
        drift = float(np.linalg.norm(rel[:3, 3] - self.attached[body][0]))
        self.attached[body] = (rel[:3, 3].copy(), rel[:3, :3].copy())
        self.client.set_frame_relative_pose(self.client.robot, self.client.tcp_frame, body, rel[:3, 3], rel[:3, :3])
        return drift

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
                                 margin: Optional[float] = None, support_bodies: Sequence[str] = ()):
        """Rules for the approach/grasp phase: contact between the robot (gripper) and the
        object about to be grasped is intentional; everything else is checked. While an
        object is attached, contact between it and the robot is intentional too (the fingers
        squeeze it), and contact between it and its support surface at release is intentional."""
        f = list(self.fingers.values())
        pairs = self._base_pairs() + [(self.client.robot, target_body)] + [(x, target_body) for x in f] + list(extra_pairs)
        for sb in support_bodies:  # fingers may touch the surface the target rests on / the container walls
            pairs += [(x, sb) for x in f]
        return collision_settings(pairs, minimum_margin=margin, resolver=self.client.oref)

    def transport_collision_settings(self, support_bodies: Sequence[str] = (), margin: Optional[float] = None):
        pairs = self._base_pairs()
        for b in self.attached:
            for s in support_bodies:
                pairs.append((b, s))
        return collision_settings(pairs, minimum_margin=margin, resolver=self.client.oref)

    def free_collision_settings(self, margin: Optional[float] = None):
        return collision_settings(self._base_pairs(), minimum_margin=margin, resolver=self.client.oref)
