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


def body_collision_geoms_world(env, body: str):
    """Per-geom corner point sets in world coordinates (list of (N,3) arrays)."""
    m = env.model
    root = m.body_name2id(body)
    pos, rot = env.body_pose(body)
    out = []
    for gi in range(m.ngeom):
        if int(m.geom_bodyid[gi]) != root:
            continue
        if int(m.geom_contype[gi]) == 0 and int(m.geom_conaffinity[gi]) == 0:
            continue
        gpos = np.array(m.geom_pos[gi]); grot = tf.quat_wxyz_to_mat(m.geom_quat[gi]); size = np.array(m.geom_size[gi])
        t = int(m.geom_type[gi])
        if t == GEOM_BOX:
            corners = np.array([[sx, sy, sz] for sx in (-size[0], size[0]) for sy in (-size[1], size[1]) for sz in (-size[2], size[2])])
        elif t == GEOM_MESH:
            mid = int(m.geom_dataid[gi]); va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            corners = np.asarray(m.mesh_vert[va:va + vn]).reshape(-1, 3)
        else:
            r = size[0]; h = size[1] if t in (3, 5) else r
            corners = np.array([[sx, sy, sz] for sx in (-r, r) for sy in (-r, r) for sz in (-h, h)])
        out.append((corners @ grot.T + gpos) @ rot.T + pos)
    return out


def extent_along(env, body: str, axis_world: np.ndarray, z_band=None) -> float:
    """Object extent along a world axis. With z_band=(zlo, zhi) only geoms overlapping the band
    are considered (approximates the horizontal cross-section at the grasp height)."""
    geoms = body_collision_geoms_world(env, body)
    if z_band is not None:
        geoms = [g for g in geoms if g[:, 2].max() >= z_band[0] and g[:, 2].min() <= z_band[1]] or geoms
    pts = np.vstack(geoms)
    proj = pts @ axis_world
    return float(proj.max() - proj.min())


def cross_section_center(env, body: str, z_band) -> np.ndarray:
    geoms = body_collision_geoms_world(env, body)
    sel = [g for g in geoms if g[:, 2].max() >= z_band[0] and g[:, 2].min() <= z_band[1]] or geoms
    pts = np.vstack(sel)
    lo, hi = pts.min(0), pts.max(0)
    return (lo + hi) / 2


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
    band = (z - 0.012, z + 0.012)
    center = cross_section_center(env, body, band)
    for yaw in yaws:
        rot = top_down_rotation(yaw)
        closing = rot[:, 0]
        w = extent_along(env, body, closing, z_band=band)
        if w > GRIPPER_MAX_OPENING - min_clearance:
            continue
        # score: narrower objects are more robust; prefer yaw near 0 (home orientation)
        score = (GRIPPER_MAX_OPENING - w) - 0.01 * abs(np.arctan2(np.sin(yaw), np.cos(yaw)))
        out.append(GraspCandidate(np.array([center[0], center[1], z]), rot,
                                  np.array([0, 0, -1.0]), w, float(yaw), float(score), f"top_down_yaw{np.degrees(yaw):.0f}"))
    out.sort(key=lambda g: -g.score)
    return out


def site_box_world(env, site: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(center, rot, half-size) of a LIBERO region site."""
    pos, rot = env.object_site_pose(site)
    return pos, rot, env.site_size(site)


def rim_grasps(env, body: str, depth: float = 0.025, n_dirs: int = 12, wall_offset: float = 0.006) -> List[GraspCandidate]:
    """Pinch grasps of a thin wall (mugs, bowls): tcp on the rim at `depth` below the top, closing
    axis radial, so one finger goes inside the cavity and one outside. Candidates are ordered by
    how well the wall direction is clear of protrusions (handles) - the outer radius along the
    direction close to the median radius wins."""
    box = object_box(env, body)
    z = box.top_z - depth
    band = (box.top_z - 2 * depth, box.top_z)
    center = cross_section_center(env, body, band)
    geoms = body_collision_geoms_world(env, body)
    pts = np.vstack([g for g in geoms if g[:, 2].max() >= band[0]] or geoms)
    out = []
    radii = []
    dirs = [np.array([np.cos(a), np.sin(a), 0.0]) for a in np.linspace(0, 2 * np.pi, n_dirs, endpoint=False)]
    for d in dirs:
        radii.append(float(((pts - center) @ d).max()))
    med = float(np.median(radii))
    for d, r in zip(dirs, radii):
        if r > med + 0.015:   # protrusion (handle) in this direction
            continue
        yaw = np.arctan2(d[1], d[0]) - np.pi / 2  # closing axis (tcp x) along d  (yaw=0 -> x = world y)
        rot = top_down_rotation(yaw)
        pos = np.array([center[0] + (r - wall_offset) * d[0], center[1] + (r - wall_offset) * d[1], z])
        score = -abs(r - med) - 0.002 * abs(np.arctan2(np.sin(yaw), np.cos(yaw)))
        out.append(GraspCandidate(pos, rot, np.array([0, 0, -1.0]), 2 * wall_offset, float(yaw), float(score), f"rim_yaw{np.degrees(yaw):.0f}"))
    out.sort(key=lambda g: -g.score)
    return out


# ----------------------------------------------------------------------------- generic grasp sampler
def object_point_cloud(env, body: str, spacing: float = 0.008) -> np.ndarray:
    """Volumetric lattice points of every collision geom of `body`, in world coordinates.
    Boxes get a regular lattice (spacing ~ `spacing`), meshes their vertices."""
    m = env.model
    root = m.body_name2id(body)
    pos, rot = env.body_pose(body)
    pts = []
    for gi in range(m.ngeom):
        if int(m.geom_bodyid[gi]) != root or (int(m.geom_contype[gi]) == 0 and int(m.geom_conaffinity[gi]) == 0):
            continue
        gpos = np.array(m.geom_pos[gi]); grot = tf.quat_wxyz_to_mat(m.geom_quat[gi]); size = np.array(m.geom_size[gi])
        t = int(m.geom_type[gi])
        if t == GEOM_BOX:
            axes = [np.linspace(-s, s, int(min(max(np.ceil(2 * s / spacing) + 1, 2), 14))) for s in size]
            local = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        elif t == GEOM_MESH:
            mid = int(m.geom_dataid[gi]); va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
            local = np.asarray(m.mesh_vert[va:va + vn]).reshape(-1, 3)
        else:
            r = size[0]; h = size[1] if t in (3, 5) else r
            axes = [np.linspace(-r, r, 5), np.linspace(-r, r, 5), np.linspace(-h, h, 5)]
            local = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
        pts.append((local @ grot.T + gpos) @ rot.T + pos)
    return np.vstack(pts)


# PandaGripper geometry in the tcp frame (x = closing axis, y = finger width, z = toward fingertips)
PAD_Z = (-0.016, 0.007)        # finger pad extent along tcp z
PAD_HALF_Y = 0.012             # half width of the pad contact face (+ margin)
FINGER_HALF_Y = 0.016          # finger body half width
FINGER_Z = (-0.05, -0.016)     # finger bodies above the pads
HAND_Z = (-0.20, -0.05)        # hand/palm zone
HAND_HALF_X, HAND_HALF_Y = 0.105, 0.035
MAX_WIDTH = 0.072              # usable opening (0.08 minus pad thickness/margin)


def evaluate_grasp(pts_world: np.ndarray, tcp_pos: np.ndarray, rot: np.ndarray):
    """Return (ok, width, n_contact_points, reason) for a top-down pinch at tcp pose."""
    local = (pts_world - tcp_pos) @ rot           # object points in the tcp frame
    x, y, z = local[:, 0], local[:, 1], local[:, 2]
    in_pad_band = (np.abs(y) < PAD_HALF_Y) & (z > PAD_Z[0]) & (z < PAD_Z[1])
    occ = x[in_pad_band & (np.abs(x) < 0.05)]
    if occ.size == 0:
        return False, 0.0, 0, "no_material_between_pads"
    lo, hi = float(occ.min()), float(occ.max())
    if not (lo < 0.0 < hi):
        return False, hi - lo, 0, "tcp_outside_feature"
    width = hi - lo
    if width > MAX_WIDTH:
        return False, width, 0, "too_wide"
    # material where the finger bodies would be (outside the pinched interval), or in the palm zone
    finger_zone = (np.abs(y) < FINGER_HALF_Y) & (z > FINGER_Z[0]) & (z < FINGER_Z[1]) & (np.abs(x) < 0.05) & ((x < lo - 0.004) | (x > hi + 0.004))
    if finger_zone.any():
        return False, width, 0, "finger_body_blocked"
    palm_zone = (np.abs(y) < HAND_HALF_Y) & (np.abs(x) < HAND_HALF_X) & (z > HAND_Z[0]) & (z < HAND_Z[1])
    if palm_zone.any():
        return False, width, 0, "palm_blocked"
    return True, width, int(in_pad_band.sum()), "ok"


def grasp_candidates(env, body: str, yaws: Sequence[float] = (), heights: Sequence[float] = (),
                     extra_points: Sequence[np.ndarray] = ()) -> List[GraspCandidate]:
    """Generic top-down pinch grasps for a box-decomposed object: candidate tcp positions are the
    centers of the object's geoms (thin features such as handles, rims, walls) plus the cross-section
    centers at several heights; every (position, yaw) is checked with evaluate_grasp against the
    object's own geometry. Returned sorted by score (width margin, contact support, yaw near home)."""
    pts = object_point_cloud(env, body)
    box = object_box(env, body)
    m = env.model
    root = m.body_name2id(body)
    pos, rot = env.body_pose(body)
    cand_pos = []
    for gi in range(m.ngeom):
        if int(m.geom_bodyid[gi]) == root and (int(m.geom_contype[gi]) or int(m.geom_conaffinity[gi])):
            gp = rot @ np.array(m.geom_pos[gi]) + pos
            cand_pos.append(gp)
    if not heights:
        heights = (0.3, 0.5, 0.7, 0.85)
    for h in heights:
        z = box.bottom_z + h * (box.top_z - box.bottom_z)
        cand_pos.append(np.array([*cross_section_center(env, body, (z - 0.01, z + 0.01))[:2], z]))
    cand_pos += list(extra_points)
    if not yaws:
        yaws = np.arange(0, np.pi, np.pi / 8)
    out = []
    seen = set()
    for p in cand_pos:
        p = np.array(p, dtype=float)
        p[2] = min(p[2], box.top_z - 0.012)  # pads must be below the top
        p[2] = max(p[2], box.bottom_z + 0.008)  # and above the support surface
        for yaw in yaws:
            R_ = top_down_rotation(yaw)
            ok, w, n, why = evaluate_grasp(pts, p, R_)
            if not ok:
                continue
            key = (round(p[0], 3), round(p[1], 3), round(p[2], 3), round(float(yaw), 2))
            if key in seen:
                continue
            seen.add(key)
            score = (MAX_WIDTH - w) + 0.0005 * min(n, 60) - 0.01 * abs(np.arctan2(np.sin(yaw), np.cos(yaw))) + 0.3 * (p[2] - box.bottom_z)
            out.append(GraspCandidate(p, R_, np.array([0, 0, -1.0]), w, float(yaw), float(score), f"pinch_z{p[2]:.3f}_yaw{np.degrees(yaw):.0f}"))
    out.sort(key=lambda g: -g.score)
    return out
