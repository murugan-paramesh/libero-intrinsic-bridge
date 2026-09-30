"""Placement target selection inside a region: keeps object origins inside the LIBERO region box
(the `In` predicate tests the origin) while avoiding footprints of objects already there.
Slots for several objects going to the same region are assigned along the region's longer
horizontal axis so that all of them fit."""
from __future__ import annotations

from typing import Callable, List, Sequence, Tuple

import numpy as np

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


def region_slots(env, region: str, bodies: Sequence[str], closing_axis_world=None) -> List[np.ndarray]:
    """Preferred xy (site-frame offsets) for `bodies` placed into `region`, spread along the
    site axis most perpendicular to the gripper closing axis (the closed finger outside the
    object must not be pushed into a container wall) with spacing 2*r_max + MARGIN, clipped so
    that the object footprint stays inside the region box (used as the container interior)."""
    c, rot, half = geo.site_box_world(env, region)
    k = len(bodies)
    if k <= 1:
        return [np.zeros(2)]
    rmax = max(footprint_radius(env, b) for b in bodies)
    if closing_axis_world is not None:
        local = rot.T @ np.asarray(closing_axis_world)
        axis = 0 if abs(local[0]) < abs(local[1]) else 1   # perpendicular to closing
    else:
        axis = 0 if half[0] >= half[1] else 1
    spacing = 2 * rmax + MARGIN
    offs = [(i - (k - 1) / 2) * spacing for i in range(k)]
    lim = min(REGION_USE * half[axis], half[axis] - rmax - 0.004)
    offs = [float(np.clip(o, -lim, lim)) for o in offs]
    slots = []
    for o in offs:
        v = np.zeros(2)
        v[axis] = o
        slots.append(v)
    return slots


def free_spot(env, region: str, body: str, preferred_local_xy: np.ndarray, all_bodies: Sequence[str],
              exclude: Sequence[str] = ()):
    """World xy for `body`'s origin and the minimum bottom height at release.

    The valid grid spot inside the region nearest the preferred slot is chosen; if every spot
    overlaps an object already in the region (a small container holding several objects), the
    preferred slot is used and the release height is raised above the overlapped objects, so
    the object is dropped onto/against them instead of being planned into them."""
    c, rot, half = geo.site_box_world(env, region)
    r_body = footprint_radius(env, body)
    others = [b for b in objects_inside_region(env, region, all_bodies) if b != body and b not in exclude]
    obstacles = [(env.body_pose(b)[0][:2], footprint_radius(env, b), geo.object_box(env, b).top_z) for b in others]
    lim = [min(REGION_USE * half[i], half[i] - r_body - 0.004) for i in range(2)]
    best, best_d = None, np.inf
    for gx in np.linspace(-lim[0], lim[0], 9):
        for gy in np.linspace(-lim[1], lim[1], 9):
            w = c + rot @ np.array([gx, gy, 0.0])
            if not all(np.linalg.norm(w[:2] - op) >= r_body + orad + MARGIN for op, orad, _ in obstacles):
                continue
            d = np.linalg.norm(np.array([gx, gy]) - preferred_local_xy)
            if d < best_d:
                best, best_d = w[:2].copy(), d
    min_bottom_z = -np.inf
    if best is None:
        pl = np.array([np.clip(preferred_local_xy[0], -lim[0], lim[0]), np.clip(preferred_local_xy[1], -lim[1], lim[1]), 0.0])
        best = (c + rot @ pl)[:2]
        for op, orad, otop in obstacles:
            if np.linalg.norm(best - op) < r_body + orad + MARGIN:
                min_bottom_z = max(min_bottom_z, otop + 0.01)
    return best, min_bottom_z
