"""Object-relative geometric reasoning: collision-box extents, grasp candidates, place targets.

All quantities are computed from the *current* scene (MuJoCo body/geom state of the object),
never from fixed world coordinates. Uses only the object's collision primitives (the same boxes
LIBERO uses for contact), so it is replaceable by a perception module that outputs the same
object pose + a stored object model.
"""
from __future__ import annotations

import dataclasses
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.model import transforms as tf

GEOM_BOX, GEOM_MESH = 6, 7
GRIPPER_MAX_OPENING = 0.08     # PandaGripper: finger joints 0..0.04 each
GRIPPER_PAD_HALF_HEIGHT = 0.008


def body_collision_points_local(model, body: str) -> np.ndarray:
    """Corner points of all collision geoms of `body` (and its child bodies without joints)
    expressed in the body frame."""
    m = model
    root = m.body_name2id(body)
    pts = []
    for gi in range(m.ngeom):
        bid = int(m.geom_bodyid[gi])
        if bid != root:
            continue
        if int(m.geom_contype[gi]) == 0 and int(m.geom_conaffinity[gi]) == 0:
            continue
        pos = np.array(m.geom_pos[gi])
        rot = tf.quat_wxyz_to_mat(m.geom_quat[gi])
        size = np.array(m.geom_size[gi])
        t = int(m.geom_type[gi])
        if t == GEOM_BOX:
            corners = np.array([[sx, sy, sz] for sx in (-size[0], size[0]) for sy in (-size[1], size[1]) for sz in (-size[2], size[2])])
        elif t == GEOM_MESH:
            mid = int(m.geom_dataid[gi])
            va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            corners = np.asarray(m.mesh_vert[va:va + vn]).reshape(-1, 3)
        else:  # sphere/cylinder/capsule: bounding box
            r = size[0]
            h = size[1] if t in (3, 5) else r
            corners = np.array([[sx, sy, sz] for sx in (-r, r) for sy in (-r, r) for sz in (-h, h)])
        pts.append(corners @ rot.T + pos)
    if not pts:
        raise ValueError(f"body {body} has no collision geometry")
    return np.vstack(pts)


@dataclasses.dataclass
class ObjectBox:
    body: str
    center_world: np.ndarray      # AABB center in world
    half_extents_world: np.ndarray
    pos: np.ndarray               # body world pose
    rot: np.ndarray
    local_min: np.ndarray
    local_max: np.ndarray

    @property
    def top_z(self):
        return self.center_world[2] + self.half_extents_world[2]

    @property
    def bottom_z(self):
        return self.center_world[2] - self.half_extents_world[2]


def object_box(env, body: str) -> ObjectBox:
    pts_l = body_collision_points_local(env.model, body)
    pos, rot = env.body_pose(body)
    pts_w = pts_l @ rot.T + pos
    lo, hi = pts_w.min(0), pts_w.max(0)
    return ObjectBox(body, (lo + hi) / 2, (hi - lo) / 2, pos, rot, pts_l.min(0), pts_l.max(0))


def extent_along(env, body: str, axis_world: np.ndarray) -> float:
    pts_l = body_collision_points_local(env.model, body)
    pos, rot = env.body_pose(body)
    proj = (pts_l @ rot.T) @ axis_world
    return float(proj.max() - proj.min())


@dataclasses.dataclass
class GraspCandidate:
    pos: np.ndarray      # desired tcp position (world)
    rot: np.ndarray      # desired tcp rotation (world); tcp z points along approach direction
    approach: np.ndarray # unit approach direction (world), e.g. -z for top-down
    width: float         # object extent along the closing axis
    yaw: float
    score: float
    label: str


def top_down_rotation(yaw: float) -> np.ndarray:
    """tcp frame with z pointing down (-world z) and the closing axis (tcp x) rotated by yaw
    about world z. yaw=0 -> closing axis = world y (the LIBERO home orientation)."""
    base = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # columns: x=(0,1,0) y=(1,0,0) z=(0,0,-1)
    return R.from_euler("z", yaw).as_matrix() @ base


def top_down_grasps(env, body: str, height_fraction: float = 0.5, yaws: Sequence[float] = (),
                    min_clearance: float = 0.012, z_offset: float = 0.0, max_depth: float = 0.045) -> List[GraspCandidate]:
    """Candidates: tcp above the object's AABB center, at a fraction of its height, for a set of
    yaw angles; keep those whose extent along the closing axis fits in the gripper.
    max_depth: the finger pads are ~4.5 cm long, so the grasp height is at most that far below
    the top of the object (deeper grasps would push the hand into the object)."""
    box = object_box(env, body)
    if not yaws:
        yaws = np.arange(0, np.pi, np.pi / 8)
    out = []
    z = box.bottom_z + height_fraction * (box.top_z - box.bottom_z)
    z = max(z, box.top_z - max_depth) + z_offset
    for yaw in yaws:
        rot = top_down_rotation(yaw)
        closing = rot[:, 0]
        w = extent_along(env, body, closing)
        if w > GRIPPER_MAX_OPENING - min_clearance:
            continue
        # score: narrower objects are more robust; prefer yaw near 0 (home orientation)
        score = (GRIPPER_MAX_OPENING - w) - 0.01 * abs(np.arctan2(np.sin(yaw), np.cos(yaw)))
        out.append(GraspCandidate(np.array([box.center_world[0], box.center_world[1], z]), rot,
                                  np.array([0, 0, -1.0]), w, float(yaw), float(score), f"top_down_yaw{np.degrees(yaw):.0f}"))
    out.sort(key=lambda g: -g.score)
    return out


def site_box_world(env, site: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(center, rot, half-size) of a LIBERO region site."""
    pos, rot = env.object_site_pose(site)
    return pos, rot, env.site_size(site)
