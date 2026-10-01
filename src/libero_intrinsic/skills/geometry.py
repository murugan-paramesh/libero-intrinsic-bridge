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
# measured from the robosuite PandaGripper collision meshes in the tcp frame (see docs/report.md)
PAD_Z = (-0.016, 0.010)        # finger pad / finger tip extent along tcp z (+3 mm margin)
PAD_HALF_Y = 0.012             # half width of the pad contact face (+ margin)
FINGER_HALF_Y = 0.016          # finger body half width (+ margin)
FINGER_Z = (-0.050, -0.016)    # finger bodies above the pads
HAND_Z = (-0.130, -0.026)      # hand/palm body: z in [-0.123, -0.031] measured, +5 mm margin
HAND_HALF_X, HAND_HALF_Y = 0.109, 0.037
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


def tilted_rotation(yaw: float, tilt_axis: str, tilt: float) -> np.ndarray:
    """Top-down grasp frame additionally rotated by `tilt` about its own x (closing) or y axis,
    so the approach direction (tcp z) leans away from a neighbouring obstacle."""
    R0 = top_down_rotation(yaw)
    ax = R0[:, 0] if tilt_axis == "x" else R0[:, 1]
    return R.from_rotvec(ax * tilt).as_matrix() @ R0


def grasp_candidates(env, body: str, yaws: Sequence[float] = (), heights: Sequence[float] = (),
                     extra_points: Sequence[np.ndarray] = (), tilts: Sequence[float] = ()) -> List[GraspCandidate]:
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
            variants = [("", top_down_rotation(yaw), 0.0)]
            for t in tilts:
                for axn in ("x", "y"):
                    for sgn in (1, -1):
                        variants.append((f"_tilt{axn}{sgn*np.degrees(t):+.0f}", tilted_rotation(yaw, axn, sgn * t), abs(t)))
            for suffix, R_, tilt_mag in variants:
                ok, w, n, why = evaluate_grasp(pts, p, R_)
                if not ok:
                    continue
                key = (round(p[0], 3), round(p[1], 3), round(p[2], 3), round(float(yaw), 2), suffix)
                if key in seen:
                    continue
                seen.add(key)
                score = (MAX_WIDTH - w) + 0.0005 * min(n, 60) - 0.01 * abs(np.arctan2(np.sin(yaw), np.cos(yaw))) + 0.3 * (p[2] - box.bottom_z) - 0.02 * tilt_mag
                out.append(GraspCandidate(p, R_, R_[:, 2].copy(), w, float(yaw), float(score), f"pinch_z{p[2]:.3f}_yaw{np.degrees(yaw):.0f}{suffix}"))
    out.sort(key=lambda g: -g.score)
    return out


def hand_clearance(env, tcp_pos, tcp_rot, opening: float, width: float, other_bodies: Sequence[str],
                   radius: float = 0.3, spacing: float = 0.012) -> Tuple[bool, str]:
    """Hand/finger zones of the tcp frame at (tcp_pos, tcp_rot) must not contain points of any
    of `other_bodies` (fingers opened to `opening`, holding an object of `width`)."""
    half_open = opening / 2.0
    for b in other_bodies:
        p = env.body_pose(b)[0]
        if np.linalg.norm(p[:2] - tcp_pos[:2]) > radius:
            continue
        pts = object_point_cloud(env, b, spacing=spacing)
        local = (pts - tcp_pos) @ tcp_rot
        x, y, z = local[:, 0], local[:, 1], local[:, 2]
        finger = (np.abs(x) >= width / 2 - 0.002) & (np.abs(x) <= half_open + 0.027) & (np.abs(y) < FINGER_HALF_Y) & (z > FINGER_Z[0]) & (z < PAD_Z[1])
        palm = (np.abs(x) < HAND_HALF_X) & (np.abs(y) < HAND_HALF_Y) & (z > HAND_Z[0]) & (z < HAND_Z[1])
        if finger.any() or palm.any():
            return False, b
    return True, ""


def gripper_corners_tcp(opening: float) -> np.ndarray:
    """Corner points (tcp frame) of the hand body and of both finger bodies at `opening`."""
    hand = np.array([[sx, sy, sz] for sx in (-HAND_HALF_X, HAND_HALF_X) for sy in (-HAND_HALF_Y, HAND_HALF_Y) for sz in (HAND_Z[0], HAND_Z[1])])
    fingers = []
    for side in (-1, 1):
        x0, x1 = side * opening / 2, side * (opening / 2 + 0.027)
        fingers += [[sx, sy, sz] for sx in (x0, x1) for sy in (-FINGER_HALF_Y, FINGER_HALF_Y) for sz in (FINGER_Z[0], PAD_Z[1])]
    return np.vstack([hand, np.array(fingers)])


def gripper_above_plane(tcp_pos, tcp_rot, opening: float, plane_z: float, margin: float = 0.004) -> bool:
    """Exact check that no hand/finger corner is below a horizontal support plane (table top)."""
    pts = gripper_corners_tcp(opening) @ tcp_rot.T + tcp_pos
    return bool(pts[:, 2].min() >= plane_z + margin)


def hand_lowest_offset(tcp_rot: np.ndarray) -> float:
    """Height of the lowest hand-body corner relative to the tcp for a given tcp rotation
    (negative = above the tcp for a top-down grasp)."""
    corners = np.array([[sx, sy, sz] for sx in (-HAND_HALF_X, HAND_HALF_X) for sy in (-HAND_HALF_Y, HAND_HALF_Y) for sz in (HAND_Z[0], HAND_Z[1])])
    world = corners @ tcp_rot.T
    return float(world[:, 2].min())


def scene_clearance(env, cand: GraspCandidate, opening: float, other_bodies: Sequence[str], radius: float = 0.25) -> Tuple[bool, str]:
    """Check a grasp candidate against neighbouring objects: with the fingers opened to
    `opening` (total), no point of another object may lie in the finger or hand zones of the
    tcp frame. Returns (ok, blocking_body)."""
    half_open = opening / 2.0
    for b in other_bodies:
        p = env.body_pose(b)[0]
        if np.linalg.norm(p[:2] - cand.pos[:2]) > radius:
            continue
        pts = object_point_cloud(env, b, spacing=0.012)
        local = (pts - cand.pos) @ cand.rot
        x, y, z = local[:, 0], local[:, 1], local[:, 2]
        finger = (np.abs(x) >= cand.width / 2 - 0.002) & (np.abs(x) <= half_open + 0.027) & (np.abs(y) < FINGER_HALF_Y) & (z > FINGER_Z[0]) & (z < PAD_Z[1])
        palm = (np.abs(x) < HAND_HALF_X) & (np.abs(y) < HAND_HALF_Y) & (z > HAND_Z[0]) & (z < HAND_Z[1])
        if finger.any() or palm.any():
            return False, b
    return True, ""


def side_rotation(approach_yaw: float, roll: float = 0.0) -> np.ndarray:
    """tcp frame for a horizontal approach: z (approach) = (cos, sin, 0) pointing at the object,
    y = world up (so the closing axis x is horizontal and tangential; the 6.4 cm-thick hand is
    horizontal, which fits low openings such as a microwave). `roll` rotates about the approach."""
    z = np.array([np.cos(approach_yaw), np.sin(approach_yaw), 0.0])
    y = np.array([0.0, 0.0, 1.0])
    x = np.cross(y, z)
    R0 = np.stack([x, y, z], axis=1)
    return R.from_rotvec(z * roll).as_matrix() @ R0


def region_opening_normal(env, region: str, owner: str):
    """Horizontal unit vector (world) pointing INTO a roofed container through its open side:
    of the four side directions of the region box, the one whose outward extension (5 cm beyond
    the box face, at the box height) contains the fewest owner collision points."""
    c, rot, half = site_box_world(env, region)
    pts = object_point_cloud(env, owner, spacing=0.01)
    local = (pts - c) @ rot
    best, best_n = None, None
    for axis in (0, 1):
        for sign in (-1.0, 1.0):
            other = 1 - axis
            sel = (sign * local[:, axis] > half[axis]) & (sign * local[:, axis] < half[axis] + 0.05) & \
                  (np.abs(local[:, other]) < half[other]) & (np.abs(local[:, 2]) < half[2])
            n = int(sel.sum())
            if best_n is None or n < best_n:
                best_n = n
                d = np.zeros(3)
                d[axis] = -sign                   # into the container = opposite to the open side
                best = rot @ d
    best[2] = 0.0
    return best / (np.linalg.norm(best) + 1e-9)


def side_grasp_candidates(env, body: str, n_yaw: int = 16, rolls=(0.0, np.pi / 2),
                          pitches=(0.0, np.radians(20), np.radians(35)), approach_dir=None,
                          approach_tol=np.radians(25)) -> List[GraspCandidate]:
    """Horizontal and oblique (pitched-down) approach pinch grasps (e.g. a mug handle or a thin
    rim from the side). The tcp is positioned on every geom centre and on the object's outer
    surface points at several heights; the candidate is kept if the object's own geometry admits
    the pinch. A positive pitch tilts the approach below the horizontal (the wrist rises and
    moves back: Intrinsic IK probe, task 9: the level wrist collides with a neighbouring mug,
    the 30 deg pitched hand has collision-free solutions at the microwave mouth)."""
    pts = object_point_cloud(env, body)
    box = object_box(env, body)
    m = env.model
    root = m.body_name2id(body)
    pos, rot = env.body_pose(body)
    centres = [rot @ np.array(m.geom_pos[gi]) + pos for gi in range(m.ngeom)
               if int(m.geom_bodyid[gi]) == root and (int(m.geom_contype[gi]) or int(m.geom_conaffinity[gi]))]
    out, seen = [], set()
    yaws = list(np.linspace(0, 2 * np.pi, n_yaw, endpoint=False))
    if approach_dir is not None:
        # the object will be inserted along `approach_dir` (a roofed container's opening normal):
        # only grasps whose approach already points that way admit the insertion
        a0 = float(np.arctan2(approach_dir[1], approach_dir[0]))
        yaws = [a0 + d for d in (0.0, -approach_tol, approach_tol)]
    for yaw in yaws:
        for roll in rolls:
            for pitch in pitches:
                R0 = side_rotation(yaw, roll)
                # pitch about the closing axis (tcp x): the approach z tilts below the horizontal
                R_ = R.from_rotvec(R0[:, 0] * pitch).as_matrix() @ R0 if pitch else R0
                if R_[2, 2] > 0:          # keep the approach pointing down, never up
                    R_ = R.from_rotvec(-R0[:, 0] * pitch).as_matrix() @ R0
                # grasp heights: every geom centre plus low grasps (2.5 / 4.5 cm above the bottom):
                # for insertion under a roof the hand must stay low (placement-aware ranking decides)
                heights = [np.array([c[0], c[1], z]) for c in centres for z in (c[2], box.bottom_z + 0.025, box.bottom_z + 0.045)]
                for c in heights:
                    p = np.array(c, dtype=float)
                    p[2] = min(max(p[2], box.bottom_z + 0.02), box.top_z - 0.01)
                    ok, w, n, why = evaluate_grasp(pts, p, R_)
                    if not ok:
                        continue
                    key = (round(p[0], 3), round(p[1], 3), round(p[2], 3), round(float(yaw), 2), round(roll, 2), round(float(pitch), 2))
                    if key in seen:
                        continue
                    seen.add(key)
                    # prefer high grasps (object hangs less), narrow features, more contact, level approaches
                    score = (MAX_WIDTH - w) + 0.0005 * min(n, 60) + 0.3 * (p[2] - box.bottom_z) - 0.01 * pitch
                    out.append(GraspCandidate(p, R_, R_[:, 2].copy(), w, float(yaw), float(score),
                                              f"side_z{p[2]:.3f}_yaw{np.degrees(yaw):.0f}_roll{np.degrees(roll):.0f}"
                                              + (f"_pitch{np.degrees(pitch):.0f}" if pitch else "")))
    out.sort(key=lambda g: -g.score)
    return out


def is_roofed_region(env, region: str, owner: str) -> bool:
    """A container region with the owner's geometry above it (microwave): objects must enter
    from the side, not from above."""
    c, rot, half = site_box_world(env, region)
    for g in body_collision_geoms_world(env, owner):
        above = g[:, 2].min() > c[2] + half[2] - 0.02
        over = np.all(np.abs(g[:, :2].mean(0) - c[:2]) < half[:2] + 0.05)
        if above and over:
            return True
    return False
