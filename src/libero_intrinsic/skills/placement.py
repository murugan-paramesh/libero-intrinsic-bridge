"""Placement target selection inside a region: keeps object origins inside the LIBERO region box
(the `In` predicate tests the origin) while avoiding footprints of objects already there.
Slots for several objects going to the same region are assigned along the region's longer
horizontal axis so that all of them fit."""
from __future__ import annotations

from typing import Callable, List, Sequence, Tuple

import numpy as np

from scipy.spatial import cKDTree

from libero_intrinsic.skills import geometry as geo

REGION_USE = 0.8       # fraction of the region half-size the origin may use
MARGIN = 0.012         # clearance between footprints (m)


def footprint_radius(env, body: str) -> float:
    b = geo.object_box(env, body)
    return float(np.max(b.half_extents_world[:2]))


def objects_inside_region(env, region: str, candidates: Sequence[str]) -> List[str]:
    c, rot, half = geo.site_box_world(env, region)
    out = []
    for b in candidates:
        p = env.body_pose(b)[0]
        local = rot.T @ (p - c)
        if np.all(np.abs(local) <= half + 0.02):
            out.append(b)
    return out


def xy_points_local(env, body: str, spacing: float = 0.01) -> np.ndarray:
    """Object collision point cloud projected to xy, relative to the object origin (world orientation)."""
    pts = geo.object_point_cloud(env, body, spacing=spacing)
    return (pts - env.body_pose(body)[0])[:, :2]


def extent_along_xy(env, body: str, axis_xy: np.ndarray) -> float:
    proj = xy_points_local(env, body) @ axis_xy
    return float(proj.max() - proj.min())


def region_slots(env, region: str, bodies: Sequence[str], closing_axis_world=None) -> List[np.ndarray]:
    """Preferred xy (site-frame offsets) for `bodies` placed into `region`, spread along the
    site axis most perpendicular to the gripper closing axis (the closed finger outside the
    object must not be pushed into a container wall) with spacing 2*r_max + MARGIN, clipped so
    that the object footprint stays inside the region box (used as the container interior)."""
    c, rot, half = geo.site_box_world(env, region)
    k = len(bodies)
    if k <= 1:
        return [np.zeros(2)]
    container = is_container_region(env, region)
    axes_w = [rot[:, i][:2] / (np.linalg.norm(rot[:, i][:2]) + 1e-9) for i in range(2)]
    exts = [max(extent_along_xy(env, b, a) for b in bodies) for a in axes_w]  # object widths along each site axis
    if container and closing_axis_world is not None:
        local = rot.T @ np.asarray(closing_axis_world)
        axis = 0 if abs(local[0]) < abs(local[1]) else 1   # perpendicular to closing (finger vs wall)
    else:
        axis = 0 if exts[0] <= exts[1] else 1                # narrowest packing on an open surface
    ext = exts[axis]
    spacing = ext + MARGIN
    offs = [(i - (k - 1) / 2) * spacing for i in range(k)]
    lim = min(REGION_USE * half[axis], half[axis] - ext / 2 - 0.004) if container else REGION_USE * half[axis]
    offs = [float(np.clip(o, -lim, lim)) for o in offs]
    slots = []
    for o in offs:
        v = np.zeros(2)
        v[axis] = o
        slots.append(v)
    return slots


def is_container_region(env, region: str) -> bool:
    """Deep region boxes (basket, drawer, microwave, caddy compartments) are containers; flat
    ones (stove burner, table target zones) are open surfaces."""
    return geo.site_box_world(env, region)[2][2] > 0.03


def region_owner(env, region: str) -> str:
    m = env.model
    return m.body_id2name(m.site_bodyid[m.site_name2id(region)])


def free_spot(env, region: str, body: str, preferred_local_xy: np.ndarray, all_bodies: Sequence[str],
              exclude: Sequence[str] = ()):
    """World xy for `body`'s origin and the minimum bottom height at release.

    The valid grid spot inside the region nearest the preferred slot is chosen; if every spot
    overlaps an object already in the region (a small container holding several objects), the
    preferred slot is used and the release height is raised above the overlapped objects, so
    the object is dropped onto/against them instead of being planned into them."""
    c, rot, half = geo.site_box_world(env, region)
    body_pts = xy_points_local(env, body)
    others = [b for b in objects_inside_region(env, region, all_bodies) if b != body and b not in exclude]
    trees = [(cKDTree(geo.object_point_cloud(env, b, spacing=0.01)[:, :2]), geo.object_box(env, b).top_z) for b in others]
    container = is_container_region(env, region)
    if container:
        ext = [extent_along_xy(env, body, rot[:, i][:2] / (np.linalg.norm(rot[:, i][:2]) + 1e-9)) for i in range(2)]
        lim = [min(REGION_USE * half[i], half[i] - ext[i] / 2 - 0.004) for i in range(2)]
    else:
        lim = [REGION_USE * half[i] for i in range(2)]

    def clear(xy):
        shifted = body_pts + xy
        return all(t.query(shifted, k=1)[0].min() >= MARGIN for t, _ in trees)

    ranked = []
    for gx in np.linspace(-lim[0], lim[0], 9):
        for gy in np.linspace(-lim[1], lim[1], 9):
            w = c + rot @ np.array([gx, gy, 0.0])
            if not clear(w[:2]):
                continue
            d = np.linalg.norm(np.array([gx, gy]) - preferred_local_xy)
            ranked.append((d, w[:2].copy()))
    ranked.sort(key=lambda t: t[0])
    best = ranked[0][1] if ranked else None
    min_bottom_z = -np.inf
    if best is None:
        pl = np.array([np.clip(preferred_local_xy[0], -lim[0], lim[0]), np.clip(preferred_local_xy[1], -lim[1], lim[1]), 0.0])
        best = (c + rot @ pl)[:2]
        shifted = body_pts + best
        for t, otop in trees:
            if t.query(shifted, k=1)[0].min() < MARGIN:
                min_bottom_z = max(min_bottom_z, otop + 0.01)
    return best, min_bottom_z


def free_spots(env, region: str, body: str, preferred_local_xy: np.ndarray, all_bodies: Sequence[str],
               exclude: Sequence[str] = (), n: int = 12):
    """Ranked list of (xy, min_bottom_z) candidate spots (nearest to the preferred slot first);
    the place skill picks the first one whose release pose keeps the hand clear of obstacles."""
    c, rot, half = geo.site_box_world(env, region)
    body_pts = xy_points_local(env, body)
    others = [b for b in objects_inside_region(env, region, all_bodies) if b != body and b not in exclude]
    trees = [(cKDTree(geo.object_point_cloud(env, b, spacing=0.01)[:, :2]), geo.object_box(env, b).top_z) for b in others]
    if is_container_region(env, region):
        ext = [extent_along_xy(env, body, rot[:, i][:2] / (np.linalg.norm(rot[:, i][:2]) + 1e-9)) for i in range(2)]
        lim = [min(REGION_USE * half[i], half[i] - ext[i] / 2 - 0.004) for i in range(2)]
    else:
        lim = [REGION_USE * half[i] for i in range(2)]
    ranked = []
    for gx in np.linspace(-lim[0], lim[0], 9):
        for gy in np.linspace(-lim[1], lim[1], 9):
            w = (c + rot @ np.array([gx, gy, 0.0]))[:2]
            shifted = body_pts + w
            dmin = min((t.query(shifted, k=1)[0].min() for t, _ in trees), default=np.inf)
            d = np.linalg.norm(np.array([gx, gy]) - preferred_local_xy)
            if dmin >= MARGIN:
                ranked.append((d, w, -np.inf))
            else:  # overlapping: would have to be released above the overlapped objects
                top = max(otop for t, otop in trees if t.query(shifted, k=1)[0].min() < MARGIN)
                ranked.append((d + 1.0, w, top + 0.01))
    ranked.sort(key=lambda t: t[0])
    return [(w, mb) for _, w, mb in ranked[:n]]


def fit_rotation(env, region: str, body: str, angles=(0.0, np.pi / 2, -np.pi / 2, np.pi)) -> float:
    """Yaw (about world z) to apply to the carried object so that its footprint fits the
    container region best (smallest overhang beyond the region box). Returns 0 when the
    current orientation already fits."""
    c, rot, half = geo.site_box_world(env, region)
    pts = xy_points_local(env, body)
    ax = [rot[:, i][:2] / (np.linalg.norm(rot[:, i][:2]) + 1e-9) for i in range(2)]
    best, best_cost = 0.0, np.inf
    for a in angles:
        R2 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        q = pts @ R2.T
        over = 0.0
        for i in range(2):
            proj = q @ ax[i]
            ext = proj.max() - proj.min()
            over += max(0.0, ext - 2 * half[i] + 0.006)
        cost = over + 0.002 * abs(a)
        if cost < best_cost - 1e-9:
            best, best_cost = a, cost
    return float(best)
