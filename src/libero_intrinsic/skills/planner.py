"""Map BDDL goal atoms to a skill sequence (classical task planning by predicate type).

  In(obj, region_site)     -> Pick(obj), Place(obj -> inside region site, support = region owner)
  On(obj, body)            -> Pick(obj), Place(obj -> on top of body)
  On(obj, region_site)     -> Pick(obj), Place(obj -> region site center, support = site owner/table)
  Turnon(stove)            -> TurnKnob(stove button)
  Close(drawer_region)     -> Push(drawer front)
  Close(microwave)         -> Push(door along its hinge arc)
Ordering: manipulation atoms first, then articulation atoms that close something (you must put
the object in before closing), knob turns before placing on the stove (LIBERO-10 task 2 wording:
"turn on the stove and put the moka pot on it").
"""
from __future__ import annotations

from typing import List

import numpy as np

from libero_intrinsic.skills import geometry as geo
from libero_intrinsic.skills.articulation import PushSkill, TurnKnobSkill, joint_world_axis_and_anchor
from libero_intrinsic.skills.manipulation import PickParams, PickSkill, PlaceParams, PlaceSkill
from libero_intrinsic.skills.task_spec import TaskSpec

# object-relative grasp parameters per object category (documented approximations)
PICK_PARAMS = {
    "default": PickParams(),
    "alphabet_soup": PickParams(height_fraction=0.55),
    "tomato_sauce": PickParams(height_fraction=0.55),
    "cream_cheese": PickParams(height_fraction=0.5),
    "butter": PickParams(height_fraction=0.5),
    "moka_pot": PickParams(height_fraction=0.6, max_depth=0.04),
    "akita_black_bowl": PickParams(height_fraction=0.9, max_depth=0.025),
    "porcelain_mug": PickParams(height_fraction=0.8, max_depth=0.03),
    "white_yellow_mug": PickParams(height_fraction=0.8, max_depth=0.03),
    "chocolate_pudding": PickParams(height_fraction=0.5),
    "black_book": PickParams(height_fraction=0.5),
}


def root_body(env, obj: str) -> str:
    return env.object_root_body(obj)


def category(obj: str) -> str:
    return obj.rsplit("_", 1)[0]


def region_owner_body(env, region: str, spec: TaskSpec) -> str:
    """The MuJoCo body a region site is attached to."""
    m = env.model
    sid = m.site_name2id(region)
    return m.body_id2name(m.site_bodyid[sid])


def has_collision_geoms(env, body: str) -> bool:
    m = env.model
    bid = m.body_name2id(body)
    return any(int(m.geom_bodyid[g]) == bid and (int(m.geom_contype[g]) or int(m.geom_conaffinity[g])) for g in range(m.ngeom))


def support_body_for(env, owner: str) -> str:
    """Collision body that physically supports a region site: the site's body if it has collision
    geometry, else a body whose name extends it (e.g. living_room_table -> living_room_table_col),
    else the plain kitchen/study 'table' body."""
    if has_collision_geoms(env, owner):
        return owner
    m = env.model
    names = [m.body_id2name(i) for i in range(m.nbody)]
    for n in names:
        if n != owner and n.startswith(owner) and has_collision_geoms(env, n):
            return n
    for n in ("table", "study_table", "kitchen_table", "living_room_table_col"):
        if n in names and has_collision_geoms(env, n):
            return n
    raise RuntimeError(f"no support body for {owner}")


def build_skill_sequence(spec: TaskSpec, env, ctx) -> List:
    skills = []
    manip, artic = [], []
    for atom in spec.goal:
        (artic if atom.predicate in ("turnon", "close", "open", "turnoff") else manip).append(atom)
    # knob before placing on the stove (task 2); drawer/microwave closing after placing (tasks 3, 9)
    knobs = [a for a in artic if a.predicate == "turnon"]
    closes = [a for a in artic if a.predicate != "turnon"]
    support_bodies = set()

    for atom in knobs:
        stove = atom.args[0]
        body = root_body(env, stove)
        knob_body = body.replace("_main", "_button") if body.endswith("_main") else f"{stove}_button"
        joint = f"{stove}_button"
        skills.append(TurnKnobSkill(knob_body, joint, target_qpos=0.5))

    for atom in manip:
        obj = atom.args[0]
        body = root_body(env, obj)
        params = PICK_PARAMS.get(category(obj), PICK_PARAMS["default"])
        skills.append(PickSkill(body, params))
        if atom.predicate == "in":
            region = atom.args[1]
            owner = support_body_for(env, region_owner_body(env, region, spec))
            support = [owner]

            def target_fn(region=region, owner=owner):
                c, rot, half = geo.site_box_world(env, region)
                ob = geo.object_box(env, owner)
                # release above the container floor (bottom of the region box, but never below the owner's bottom)
                floor = max(c[2] - half[2], ob.bottom_z + 0.005)
                return np.array([c[0], c[1], floor]), floor
            skills.append(PlaceSkill(body, target_fn, support))
        elif atom.predicate == "on":
            tgt = atom.args[1]
            try:
                env.model.site_name2id(tgt)
                is_site = True
            except Exception:
                is_site = False
            if is_site:
                owner = support_body_for(env, region_owner_body(env, tgt, spec))
                support = [owner]

                def target_fn(tgt=tgt, owner=owner):
                    c, rot, half = geo.site_box_world(env, tgt)
                    top = c[2] + half[2]
                    return np.array([c[0], c[1], top]), top
            else:
                owner = root_body(env, tgt)
                support = [owner]

                def target_fn(owner=owner):
                    ob = geo.object_box(env, owner)
                    return np.array([ob.pos[0], ob.pos[1], ob.top_z]), ob.top_z
            skills.append(PlaceSkill(body, target_fn, support))
        support_bodies.update(support)

    for atom in closes:
        target = atom.args[0]
        if atom.predicate == "close" and "cabinet" in target and "_region" in target:
            # e.g. white_cabinet_1_bottom_region -> drawer body white_cabinet_1_cabinet_bottom, joint white_cabinet_1_bottom_level
            cab = target.split("_region")[0].rsplit("_", 1)[0]  # white_cabinet_1
            level = target.split("_region")[0].rsplit("_", 1)[1]  # bottom
            drawer_body = f"{cab}_cabinet_{level}"
            joint = f"{cab}_{level}_level"
            skills.append(PushSkill(drawer_body, *drawer_close_geometry(env, drawer_body, joint), label="close_drawer",
                                    extra_contact_bodies=[f"{cab}_base"]))
        elif atom.predicate == "close" and "microwave" in target:
            door_body = f"{target}_microdoorroot"
            joint = f"{target}_microjoint"
            skills.append(PushSkill(door_body, *microwave_close_geometry(env, door_body, joint), label="close_microwave",
                                    extra_contact_bodies=[f"{target}_main"]))
        else:
            raise NotImplementedError(f"no skill for goal atom {atom}")
    ctx.params["support_bodies"] = sorted(support_bodies)
    return skills


def drawer_close_geometry(env, drawer_body: str, joint: str):
    def contact_fn():
        axis, _ = joint_world_axis_and_anchor(env, joint)  # slide axis (world); open = negative qpos
        q = env.joint_qpos(joint)
        box = geo.object_box(env, drawer_body)
        # front face: the extreme of the drawer along -axis (it slid out that way)
        pts = geo.body_collision_points_local(env.model, drawer_body) @ box.rot.T + box.pos
        proj = pts @ axis
        front = pts[np.argmin(proj)]
        center_line = box.center_world - axis * (proj.max() - proj.min()) / 2
        contact = np.array([center_line[0], center_line[1], box.bottom_z + 0.6 * (box.top_z - box.bottom_z)])
        contact = contact - axis * (0.02)  # fingertips 2 cm outside the front face
        rot = geo.top_down_rotation(np.arctan2(axis[1], axis[0]))  # closing axis along push direction
        pre = contact + np.array([0, 0, 0.10])
        path = [contact, contact + axis * (abs(q) + 0.03)]
        return pre, rot, path

    def check_fn():
        return env.joint_qpos(joint) > 0.0
    return contact_fn, check_fn


def microwave_close_geometry(env, door_body: str, joint: str):
    def contact_fn():
        axis, anchor = joint_world_axis_and_anchor(env, joint)  # vertical hinge
        q = env.joint_qpos(joint)  # open: <= -1.3, closed: > -0.005
        box = geo.object_box(env, door_body)
        pts = geo.body_collision_points_local(env.model, door_body) @ box.rot.T + box.pos
        # door outer edge: farthest point from the hinge axis (in the horizontal plane)
        rel = pts - anchor
        rel[:, 2] = 0
        edge = pts[np.argmax(np.linalg.norm(rel, axis=1))]
        r_edge = 0.85 * np.linalg.norm((edge - anchor)[:2])
        ang0 = np.arctan2((edge - anchor)[1], (edge - anchor)[0])
        z = box.bottom_z + 0.5 * (box.top_z - box.bottom_z)
        # the door rotates by -q about the axis to close; push the outside face along the arc
        sign = 1.0 if q < 0 else -1.0
        n_seg = max(int(np.ceil(abs(q) / 0.35)), 2)
        path = []
        for k in range(1, n_seg + 1):
            a = ang0 + sign * abs(q) * k / n_seg * (1 if axis[2] > 0 else -1)
            path.append(anchor + np.array([r_edge * np.cos(a), r_edge * np.sin(a), z - anchor[2]]))
        # contact point: just outside the door face at the starting angle, offset opposite the motion
        a_pre = ang0 - sign * 0.12 * (1 if axis[2] > 0 else -1)
        contact = anchor + np.array([r_edge * np.cos(a_pre), r_edge * np.sin(a_pre), z - anchor[2]])
        rot = geo.top_down_rotation(0.0)
        pre = contact + np.array([0, 0, 0.12])
        return pre, rot, [contact] + path

    def check_fn():
        return env.joint_qpos(joint) > -0.005
    return contact_fn, check_fn
