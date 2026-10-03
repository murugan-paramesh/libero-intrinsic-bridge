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

import re

from typing import List

import numpy as np
from scipy.spatial.transform import Rotation as R

from libero_intrinsic.skills import geometry as geo
from libero_intrinsic.skills import placement
from libero_intrinsic.intrinsic.client import IntrinsicRequestError

# side-grasp staging: stage when the best pre-grasp IK margin is below this (experiment: 9.0 = always)
STAGE_MARGIN_THRESHOLD = 9.0
from libero_intrinsic.skills.base import Skill, SkillResult
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


def is_site(env, name: str) -> bool:
    try:
        env.model.site_name2id(name)
        return True
    except Exception:
        return False


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
    ctx.params["home_q"] = np.array(env.robot_state().q, dtype=float)   # episode start posture (IK seed diversity)
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

    # slots for several objects going into the same region
    all_movable = [root_body(env, o) for o in env.object_names()]
    region_members = {}
    for atom in manip:
        if atom.predicate == "in" or (atom.predicate == "on" and is_site(env, atom.args[1])):
            region_members.setdefault(atom.args[1], []).append(root_body(env, atom.args[0]))
    slot_index = {}
    for region, bodies in region_members.items():
        for i, b in enumerate(bodies):
            slot_index[(region, b)] = (i, bodies)

    import dataclasses as _dc
    for atom in manip:
        obj = atom.args[0]
        body = root_body(env, obj)
        params = PICK_PARAMS.get(category(obj), PICK_PARAMS["default"])
        if atom.predicate == "in" and geo.is_roofed_region(env, atom.args[1], support_body_for(env, region_owner_body(env, atom.args[1], spec))):
            owner_b = support_body_for(env, region_owner_body(env, atom.args[1], spec))
            # must enter a front-loading container: side grasps whose approach is the opening normal
            params = _dc.replace(params, side_grasp=True, approach_dir=geo.region_opening_normal(env, atom.args[1], owner_b))
            ctx.record(event="opening_normal", region=atom.args[1], normal=[float(v) for v in params.approach_dir])
        pick = PickSkill(body, params)
        if params.side_grasp:
            # a roofed-container pick may be blocked by a neighbouring non-goal object (the hand
            # must come in level along the opening normal): allow one obstacle relocation
            table_b = next(n for n in ("table", "kitchen_table", "study_table", "living_room_table_col")
                           if n in [env.model.body_id2name(i) for i in range(env.model.nbody)])
            goal_b = [root_body(env, a.args[0]) for a in manip]
            region_b = atom.args[1]
            keep = lambda body=body, region_b=region_b: [env.body_pose(body)[0], geo.site_box_world(env, region_b)[0]]
            skills.append(PickWithRelocation(env, pick, goal_b, table_b, all_movable, keep, PICK_PARAMS["default"]))
        else:
            skills.append(pick)
        if atom.predicate == "in":
            region = atom.args[1]
            owner = support_body_for(env, region_owner_body(env, region, spec))
            support = [owner]
            si, members = slot_index[(region, body)]

            def target_fn(closing_axis, region=region, owner=owner, body=body, si=si, members=members):
                c, rot, half = geo.site_box_world(env, region)
                ob = geo.object_box(env, owner)
                # release above the container floor (bottom of the region box, but never below the owner's bottom)
                floor = max(c[2] - half[2], ob.bottom_z + 0.005)
                slot = placement.region_slots(env, region, members, closing_axis)[si]
                spots = placement.free_spots(env, region, body, slot, all_movable, exclude=[owner])
                return [(np.array([xy[0], xy[1], max(floor, mb)]), max(floor, mb)) for xy, mb in spots]
            skills.append(PlaceSkill(body, target_fn, support, region=region))
            pick.place_skill = skills[-1]
        elif atom.predicate == "on":
            tgt = atom.args[1]
            if is_site(env, tgt):
                owner = support_body_for(env, region_owner_body(env, tgt, spec))
                support = [owner]
                si, members = slot_index[(tgt, body)]

                def target_fn(closing_axis, tgt=tgt, owner=owner, body=body, si=si, members=members):
                    c, rot, half = geo.site_box_world(env, tgt)
                    top = max(c[2] + half[2], geo.object_box(env, owner).top_z)
                    slot = placement.region_slots(env, tgt, members, closing_axis)[si]
                    spots = placement.free_spots(env, tgt, body, slot, all_movable, exclude=[owner])
                    return [(np.array([xy[0], xy[1], max(top, mb)]), max(top, mb)) for xy, mb in spots]
                region_for_place = tgt
            else:
                owner = root_body(env, tgt)
                support = [owner]
                region_for_place = None

                def target_fn(closing_axis, owner=owner):
                    ob = geo.object_box(env, owner)
                    return np.array([ob.pos[0], ob.pos[1], ob.top_z]), ob.top_z
            skills.append(PlaceSkill(body, target_fn, support, region=region_for_place))
            pick.place_skill = skills[-1]
        support_bodies.update(support)

    for atom in closes:
        target = atom.args[0]
        if atom.predicate == "close" and "cabinet" in target and "_region" in target:
            # e.g. white_cabinet_1_bottom_region -> drawer body white_cabinet_1_cabinet_bottom, joint white_cabinet_1_bottom_level
            cab = target.split("_region")[0].rsplit("_", 1)[0]  # white_cabinet_1
            level = target.split("_region")[0].rsplit("_", 1)[1]  # bottom
            drawer_body = f"{cab}_cabinet_{level}"
            joint = f"{cab}_{level}_level"
            ctx.params["push_extra"] = [f"{cab}_base"]
            # two-stage closing (contact-constrained push along the prismatic axis): stage A pushes
            # from above as far as the geometry allows (partial progress is a success), stage B
            # finishes with a horizontal hand chosen by Intrinsic IK feasibility of the whole travel
            # C5: stage A = pitched push on the handle bar (closes the whole travel on the development
            # states when the bowl lies level); the earlier top-down stage A is kept in
            # drawer_push_topdown_geometry for reference
            stage_a = PushSkill(drawer_body, *drawer_push_handle_geometry(env, drawer_body, joint), label="close_drawer_a",
                                extra_contact_bodies=[f"{cab}_base"], partial_ok=True,
                                progress_fn=lambda joint=joint: float(env.joint_qpos(joint)), time_scale=2.0)
            # C5: before anything else, pull the drawer to its open limit (best effort)
            skills.insert(0, BestEffortPush(drawer_body, *drawer_open_geometry(env, drawer_body, joint), label="open_drawer_fully",
                                            extra_contact_bodies=[f"{cab}_base"], partial_ok=True,
                                            progress_fn=lambda joint=joint: -float(env.joint_qpos(joint)), time_scale=1.5))
            geom_b, check_b = drawer_close_geometry(env, drawer_body, joint)
            stage_b = PushSkill(drawer_body, geom_b, check_b, label="close_drawer_b", extra_contact_bodies=[f"{cab}_base"])
            goal_bodies = [root_body(env, a.args[0]) for a in manip]
            table = next(n for n in ("table", "kitchen_table", "study_table", "living_room_table_col")
                         if n in [env.model.body_id2name(i) for i in range(env.model.nbody)])
            skills.append(DrawerCloseComposite(env, stage_a, stage_b, check_b, drawer_body, goal_bodies, table, all_movable,
                                               PICK_PARAMS["default"]))
        elif atom.predicate == "close" and "microwave" in target:
            door_body = f"{target}_microdoorroot"
            joint = f"{target}_microjoint"
            skills.append(PushSkill(door_body, *microwave_close_geometry(env, door_body, joint), label="close_microwave",
                                    extra_contact_bodies=[f"{target}_main"],
                                    progress_fn=lambda joint=joint: float(env.joint_qpos(joint)),   # door angle (closing increases it)
                                    time_scale=1.5))
        else:
            raise NotImplementedError(f"no skill for goal atom {atom}")
    ctx.params["support_bodies"] = sorted(support_bodies)
    return skills


def is_movable_body(env, body: str) -> bool:
    """A body that can be relocated by manipulation: it has its own free joint (fixtures such as
    the wine rack or the cabinet have none and must never be relocation candidates)."""
    m = env.model
    try:
        bid = m.body_name2id(body)
    except Exception:
        return False
    for j in range(m.njnt):
        if int(m.jnt_bodyid[j]) == bid and int(m.jnt_type[j]) == 0:   # mjJNT_FREE
            return True
    return False


def free_table_spot(env, ctx, obstacle, table_body, all_movable, keep_away_xy, keep_away_r=0.25, record=True):
    """Grid search on the table top: >= 8 cm from every other object's footprint, inside the table
    minus 8 cm, 0.35-0.70 m from the robot base, >= keep_away_r from each point in keep_away_xy
    (the manipulation corridor), >= 15 cm from the obstacle's current spot; nearest such spot."""
    from scipy.spatial import cKDTree
    tb = geo.object_box(env, table_body)
    base = env.robot_state().base_pos[:2]
    obj_xy = env.body_pose(obstacle)[0][:2]
    trees = [cKDTree(geo.object_point_cloud(env, b, spacing=0.01)[:, :2]) for b in all_movable if b != obstacle]
    fixed = [b for b in ctx.sync.bodies if b not in all_movable and b != table_body]
    trees += [cKDTree(geo.object_point_cloud(env, b, spacing=0.02)[:, :2]) for b in fixed]
    r_obj = float(np.max(geo.object_box(env, obstacle).half_extents_world[:2]))
    best, best_d = None, np.inf
    for gx in np.arange(tb.center_world[0] - tb.half_extents_world[0] + 0.08, tb.center_world[0] + tb.half_extents_world[0] - 0.08, 0.05):
        for gy in np.arange(tb.center_world[1] - tb.half_extents_world[1] + 0.08, tb.center_world[1] + tb.half_extents_world[1] - 0.08, 0.05):
            xy = np.array([gx, gy])
            reach = np.linalg.norm(xy - base)
            if not (0.35 <= reach <= 0.70) or np.linalg.norm(xy - obj_xy) < 0.15:
                continue
            if any(np.linalg.norm(xy - np.asarray(k)[:2]) < keep_away_r for k in keep_away_xy):
                continue
            if any(t.query(xy)[0] < r_obj + 0.08 for t in trees):
                continue
            d = np.linalg.norm(xy - obj_xy)
            if d < best_d:
                best, best_d = xy, d
    if record:
        ctx.record(event="relocation_spot", obstacle=obstacle, spot=None if best is None else [float(v) for v in best])
    return best


class PickWithRelocation(Skill):
    """Bounded task-level replanning for a blocked pick: when the pick fails because every grasp
    candidate is blocked by ONE movable, non-goal object (the geometric clearance filter and the
    IK collision reports name it), that object is relocated by real manipulation (pick, place on
    a free table spot away from the goal object and its target region) and the pick is retried
    once. Everything is counted in the episode budget."""
    name = "pick"
    max_attempts = 1

    def __init__(self, env, pick, goal_bodies, table_body, all_movable, keep_away_xy_fn, pick_params):
        self.env, self.pick, self.goal_bodies, self.table_body, self.all_movable = env, pick, set(goal_bodies), table_body, all_movable
        self.keep_away_xy_fn, self.pick_params = keep_away_xy_fn, pick_params
        self.body = pick.body

    def preconditions(self, ctx):
        return self.pick.preconditions(ctx)

    def attempt(self, ctx, i):
        # Assess first (geometry only, no motion): if no grasp admits the later insertion and the
        # candidates blocked at the pick are blocked by one movable non-goal object, relocate it
        # BEFORE picking (a pickable grasp that cannot be inserted is not accepted).
        ctx.sync.sync()
        self.pick.assess_only = True
        try:
            self.pick.attempt(ctx, 0)
        finally:
            self.pick.assess_only = False
        a = self.pick.last_assessment
        blockers = [b for b, n in sorted(a.get("blockers", {}).items(), key=lambda kv: -kv[1])
                    if b in self.all_movable and b not in self.goal_bodies and is_movable_body(self.env, b)]
        if a.get("n_compatible", 1) > 0 or not blockers:
            r = self.pick.run(ctx)
            if r.ok or not blockers:
                return r
        obstacle = blockers[0]
        ctx.record(event="obstacle_relocation", obstacle=obstacle, blockers=blockers, stage="pick", assessment=a)
        spot = free_table_spot(self.env, ctx, obstacle, self.table_body, self.all_movable, self.keep_away_xy_fn())
        if spot is None:
            return SkillResult(self.name, False, "no_relocation_spot:" + r.reason)
        rp = PickSkill(obstacle, self.pick_params).run(ctx)
        if not rp.ok:
            return SkillResult(self.name, False, "relocate_pick:" + rp.reason)
        top = geo.object_box(self.env, self.table_body).top_z
        rpl = PlaceSkill(obstacle, lambda closing_axis, spot=spot, top=top: (np.array([spot[0], spot[1], top]), top), [self.table_body]).run(ctx)
        if not rpl.ok:
            return SkillResult(self.name, False, "relocate_place:" + rpl.reason)
        self.pick._tried = []
        self._stage_for_side_grasp(ctx)
        r2 = self.pick.run(ctx)
        return SkillResult(self.name, r2.ok, "" if r2.ok else "pick_after_relocation:" + r2.reason, details=r2.details)

    def _stage_for_side_grasp(self, ctx):
        """Temporary placement + regrasp (physically executed): a level side grasp along a
        container's opening normal needs a comfortable arm posture (observed on task 9: on some
        states every pre-grasp configuration lies at the joint-2 limit and the controller reaches
        the pose 3-5 cm off; on the others the same grasp succeeds). If Intrinsic IK finds no
        pre-grasp configuration with a joint-limit margin, the object is first picked top-down and
        placed on the opening-normal line at a reach-comfortable distance, then side-picked."""
        p = self.pick.p
        if not p.side_grasp or p.approach_dir is None:
            return
        env, client, sync = ctx.env, ctx.client, ctx.sync
        body = self.pick.body
        cands = geo.side_grasp_candidates(env, body, approach_dir=p.approach_dir)
        if not cands:
            return
        rs = env.robot_state()
        lim = env.joint_limits()
        best_margin = -1.0
        for c in cands[:6]:
            pre = c.pos - c.approach * p.pregrasp_clearance
            try:
                sols, _, _ = client.ik(pre, c.rot, rs.q, max_solutions=8, collision_settings=sync.free_collision_settings())
            except IntrinsicRequestError:
                continue
            for q in sols:
                best_margin = max(best_margin, float(min(np.min(np.asarray(q) - lim[:, 0]), np.min(lim[:, 1] - np.asarray(q)))))
        ctx.record(event="side_grasp_posture_check", best_margin_rad=float(best_margin), n_candidates=len(cands))
        if best_margin >= STAGE_MARGIN_THRESHOLD:
            return
        # staging spot: on the opening-normal line through the region centre, in front of the opening
        region_c = self.keep_away_xy_fn()[1]
        n = np.asarray(p.approach_dir[:2]); n = n / (np.linalg.norm(n) + 1e-9)
        base = env.robot_state().base_pos[:2]
        obj_xy = env.body_pose(body)[0][:2]
        spot = None
        for d in np.arange(0.28, 0.46, 0.02):
            xy = np.asarray(region_c[:2]) - n * d
            if 0.60 <= np.linalg.norm(xy - base) <= 0.70 and np.linalg.norm(xy - obj_xy) > 0.03:
                blocked = False
                for b in self.all_movable:
                    if b != body and np.linalg.norm(env.body_pose(b)[0][:2] - xy) < 0.12:
                        blocked = True
                if not blocked:
                    spot = xy
                    break
        ctx.record(event="side_grasp_staging", spot=None if spot is None else [float(v) for v in spot], margin=float(best_margin))
        if spot is None:
            return
        top = geo.object_box(env, self.table_body).top_z
        rp = PickSkill(body, self.pick_params).run(ctx)
        if not rp.ok:
            ctx.record(event="side_grasp_staging_failed", stage="pick", reason=rp.reason[:100])
            return
        rpl = PlaceSkill(body, lambda closing_axis, spot=spot, top=top: (np.array([spot[0], spot[1], top]), top), [self.table_body]).run(ctx)
        ctx.record(event="side_grasp_staging_done", ok=bool(rpl.ok), reason=rpl.reason[:100], obj_xy=[float(v) for v in env.body_pose(body)[0][:2]])
        sync.sync()
        self.pick._tried = []


class DrawerCloseComposite(Skill):
    """Two-stage drawer closing with obstacle relocation (bounded task-level replanning).

    stage A (top-down push, partial progress allowed) -> stage B (horizontal push chosen by IK
    feasibility). If stage B finds no feasible contact because a MOVABLE, non-goal object blocks
    the arm (the wine bottle next to the cabinet: forearm collisions in the IK probes), that
    object is relocated by real manipulation (pick, place on a free table spot away from the
    push corridor) and stage B is retried once. Everything is counted in the episode budget."""
    name = "close_drawer"
    max_attempts = 1

    def __init__(self, env, stage_a, stage_b, check_fn, drawer_body, goal_bodies, table_body, all_movable, pick_params):
        self.env, self.a, self.b, self.check_fn = env, stage_a, stage_b, check_fn
        self.drawer_body, self.goal_bodies, self.table_body, self.all_movable, self.pick_params = drawer_body, set(goal_bodies), table_body, all_movable, pick_params
        self.body = drawer_body

    def attempt(self, ctx, i):
        ra = self.a.run(ctx)
        if not ra.ok:
            # stage A (top-down partial push) could not even start on some states (posture at the
            # reach limit); stage B's run-time contact selection is independent of it, so it is
            # still attempted (recorded)
            ctx.record(event="stage_a_failed_continue", reason=ra.reason[:120])
        if self.check_fn():
            return SkillResult(self.name, True)
        rb = self.b.run(ctx)
        if rb.ok:
            return SkillResult(self.name, True)
        blockers = [b for b in ctx.params.get("last_push_blockers", []) if b not in self.goal_bodies and b != self.drawer_body
                    and is_movable_body(self.env, b)]
        if not blockers:
            return SkillResult(self.name, False, "stage_b:" + rb.reason)
        obstacle = blockers[0]
        ctx.record(event="obstacle_relocation", obstacle=obstacle, blockers=blockers)
        spot = free_table_spot(self.env, ctx, obstacle, self.table_body, self.all_movable,
                               [geo.object_box(self.env, self.drawer_body).center_world])
        if spot is None:
            return SkillResult(self.name, False, "no_relocation_spot:" + rb.reason)
        pick = PickSkill(obstacle, self.pick_params)
        rp = pick.run(ctx)
        if not rp.ok:
            return SkillResult(self.name, False, "relocate_pick:" + rp.reason)
        top = geo.object_box(self.env, self.table_body).top_z
        place = PlaceSkill(obstacle, lambda closing_axis, spot=spot, top=top: (np.array([spot[0], spot[1], top]), top), [self.table_body])
        rpl = place.run(ctx)
        if not rpl.ok:
            return SkillResult(self.name, False, "relocate_place:" + rpl.reason)
        rb2 = self.b.run(ctx)
        return SkillResult(self.name, rb2.ok, "" if rb2.ok else "stage_b_after_relocation:" + rb2.reason)

    def _free_table_spot(self, ctx, obstacle):
        """Grid search on the table top: >= 8 cm from every other object's footprint, inside the
        table minus 8 cm, 0.35-0.70 m from the robot base, >= 25 cm from the drawer front, and
        >= 15 cm from the obstacle's current spot; nearest such spot to the obstacle."""
        from scipy.spatial import cKDTree
        env = self.env
        tb = geo.object_box(env, self.table_body)
        base = env.robot_state().base_pos[:2]
        obj_xy = env.body_pose(obstacle)[0][:2]
        drawer_xy = geo.object_box(env, self.drawer_body).center_world[:2]
        trees = [cKDTree(geo.object_point_cloud(env, b, spacing=0.01)[:, :2]) for b in self.all_movable if b != obstacle]
        fixed = [b for b in ctx.sync.bodies if b not in self.all_movable and b != self.table_body]
        trees += [cKDTree(geo.object_point_cloud(env, b, spacing=0.02)[:, :2]) for b in fixed]
        r_obj = float(np.max(geo.object_box(env, obstacle).half_extents_world[:2]))
        best, best_d = None, np.inf
        for gx in np.arange(tb.center_world[0] - tb.half_extents_world[0] + 0.08, tb.center_world[0] + tb.half_extents_world[0] - 0.08, 0.05):
            for gy in np.arange(tb.center_world[1] - tb.half_extents_world[1] + 0.08, tb.center_world[1] + tb.half_extents_world[1] - 0.08, 0.05):
                xy = np.array([gx, gy])
                reach = np.linalg.norm(xy - base)
                if not (0.35 <= reach <= 0.70) or np.linalg.norm(xy - drawer_xy) < 0.25 or np.linalg.norm(xy - obj_xy) < 0.15:
                    continue
                if any(t.query(xy)[0] < r_obj + 0.08 for t in trees):
                    continue
                d = np.linalg.norm(xy - obj_xy)
                if d < best_d:
                    best, best_d = xy, d
        ctx.record(event="relocation_spot", spot=None if best is None else [float(v) for v in best])
        return best


def drawer_push_topdown_geometry(env, drawer_body: str, joint: str):
    """Stage A of the drawer closing: top-down push with the closing axis along the push and the
    approach tilted 20 deg toward the push direction (IK probe: solvable and collision-free at the
    contact and mid-travel poses; the last ~3 cm are blocked by the wrist/forearm against the
    cabinet's upper drawer fronts, which stage B handles with a horizontal hand)."""
    def contact_fn(ctx=None):
        axis, _ = joint_world_axis_and_anchor(env, joint)
        q = env.joint_qpos(joint)
        box = geo.object_box(env, drawer_body)
        pts = geo.body_collision_points_local(env.model, drawer_body) @ box.rot.T + box.pos
        proj = pts @ axis
        center_line = box.center_world - axis * (proj.max() - proj.min()) / 2
        contact = np.array([center_line[0], center_line[1], box.bottom_z + 0.4 * (box.top_z - box.bottom_z)])
        contact = contact - axis * 0.015
        yaw = np.arctan2(axis[1], axis[0])
        rot = geo.top_down_rotation(yaw)
        tilt_axis = rot[:, 1]
        sign = 1.0 if np.dot(np.cross(tilt_axis, rot[:, 2]), axis) > 0 else -1.0
        rot = R.from_rotvec(tilt_axis * sign * np.radians(20)).as_matrix() @ rot
        pre = contact - rot[:, 2] * 0.10
        return pre, rot, [contact, contact + axis * (abs(q) + 0.03)]

    def check_fn():
        return env.joint_qpos(joint) > 0.0
    return contact_fn, check_fn


def drawer_open_geometry(env, drawer_body: str, joint: str):
    """Pull the drawer fully open (to its joint limit) by pushing the inner face of its inner
    front wall from INSIDE the empty cavity with the closed fingertips: hand vertical, closing
    axis across the drawer, palm above the side walls. LIBERO samples the initial opening of the
    drawer in [-0.16, -0.14] m; measured on the protocol states, the bowl lands level only when
    the cavity in front of the upper drawers' handle bars (world-fixed) is long enough, i.e. when
    the drawer is open to within ~5 mm of its limit (states at -0.16: level; -0.147: tilted on the
    inner wall, which then jams the closing). The extra 1-2 cm are gained here before the place."""
    m = env.model
    lim_lo = float(m.jnt_range[m.joint_name2id(joint)][0])
    region = joint.replace("_level", "_region")

    def contact_fn(ctx=None):
        axis, _ = joint_world_axis_and_anchor(env, joint)
        q = env.joint_qpos(joint)
        c_r, rot_r, half_r = geo.site_box_world(env, region)
        half_along = max(abs(float(np.dot(rot_r[:, i], axis))) * float(half_r[i]) for i in range(3))
        box = geo.object_box(env, drawer_body)
        contact = c_r - axis * half_along + axis * 0.010      # 1 cm behind the inner front wall's inner face
        contact[2] = box.top_z - 0.028                        # fingertips 2.8 cm below the wall top (palm above it)
        pre = contact + np.array([0.0, 0.0, 0.08])
        rot = geo.top_down_rotation(np.arctan2(axis[1], axis[0]))
        travel = max(float(q - lim_lo), 0.0) + 0.004
        return pre, rot, [contact, contact - axis * travel]

    def check_fn():
        return env.joint_qpos(joint) <= lim_lo + 0.003
    return contact_fn, check_fn


def drawer_push_handle_geometry(env, drawer_body: str, joint: str, tilt_deg: float = 50.0):
    """Closing push on the drawer's HANDLE BAR with the hand pitched `tilt_deg` below horizontal
    (fingers pointing forward-down along the drawer axis). Established with Intrinsic on the
    development states (scripts/t3_lab.py): the near-vertical stage-A hand is blocked for the last
    5 cm by robot0_link5 against the cabinet's top drawer front, the horizontal stage-B hand by
    link5/link6 against the wine rack and the table; at 50 deg the wrist stays back and low and
    the collision-checked IK and LINEAR plans exist for the whole travel to the closed position."""
    def contact_fn(ctx=None):
        axis, _ = joint_world_axis_and_anchor(env, joint)
        q = env.joint_qpos(joint)
        box = geo.object_box(env, drawer_body)
        pts = geo.body_collision_points_local(env.model, drawer_body) @ box.rot.T + box.pos
        proj = pts @ axis
        front = pts[proj < proj.min() + 0.006]                 # the handle bar's front face
        contact = front.mean(0) - axis * 0.015                 # fingertips 1.5 cm in front of the bar
        yaw = np.arctan2(axis[1], axis[0])
        rot = geo.side_rotation(yaw, 0.0)
        ta = rot[:, 0] if abs(rot[2, 0]) < 0.5 else rot[:, 1]
        sign = 1.0 if np.dot(np.cross(ta, rot[:, 2]), np.array([0.0, 0.0, -1.0])) > 0 else -1.0
        rot = R.from_rotvec(ta * sign * np.radians(tilt_deg)).as_matrix() @ rot
        pre = contact - rot[:, 2] * 0.08
        return pre, rot, [contact, contact + axis * (abs(q) + 0.02)]

    def check_fn():
        return env.joint_qpos(joint) > 0.0
    return contact_fn, check_fn


class BestEffortPush(PushSkill):
    """A push whose failure must not end the episode (the following skills can still succeed)."""

    def run(self, ctx):
        r = super().run(ctx)
        ctx.record(event="best_effort_push", label=self.label, ok=bool(r.ok), reason=r.reason[:120])
        return SkillResult(self.name, True, "" if r.ok else "best_effort:" + r.reason)


def drawer_close_geometry(env, drawer_body: str, joint: str):
    """Drawer closing as a contact-constrained push along the prismatic axis.

    Strategy family selected by Intrinsic IK probes (docs/methods.md): a HORIZONTAL hand (tcp
    z = push direction, wide hand dimension horizontal, 7.4 cm tall) whose body stays within
    the front panel's height band, so nothing of the gripper reaches above the panel when the
    drawer is flush with the cabinet (every top-down pose collides with the upper drawer fronts
    at the end of the travel). The lateral contact point along the panel is chosen at run time:
    candidates are ordered by prior and the first one whose pre-contact, contact, mid-travel
    and end-of-travel poses all have collision-free Intrinsic IK solutions is used (category 2
    orchestration around ComputeIk; no local IK)."""
    LATERAL = (-0.06, -0.045, -0.075, -0.03, -0.09, 0.0, 0.04)   # metres along the panel (prior order)
    # fraction of the panel height; observed with the rider-aware probe: at 0.35-0.5 the wrist
    # (link 6) touches the table at the end of the travel, so higher contacts come first
    HEIGHTS = (0.8, 0.65, 0.5, 0.4)

    def contact_fn(ctx=None):
        axis, _ = joint_world_axis_and_anchor(env, joint)  # slide axis (world); open = negative qpos
        q = env.joint_qpos(joint)
        box = geo.object_box(env, drawer_body)
        pts = geo.body_collision_points_local(env.model, drawer_body) @ box.rot.T + box.pos
        proj = pts @ axis
        center_line = box.center_world - axis * (proj.max() - proj.min()) / 2   # front face centre
        lateral = np.cross(np.array([0.0, 0.0, 1.0]), axis)
        yaw = np.arctan2(axis[1], axis[0])
        rot = geo.side_rotation(yaw, 0.0)          # approach = push direction, hand horizontal
        travel = abs(q) + 0.03

        def poses_for(xo, h):
            contact = np.array([center_line[0], center_line[1], box.bottom_z + h * (box.top_z - box.bottom_z)])
            contact = contact + lateral * xo - axis * 0.015       # fingertips 1.5 cm outside the front face
            return contact, {"pre": contact - axis * 0.08, "contact": contact, "mid": contact + axis * travel / 2,
                             "end": contact + axis * travel}
        chosen, tried = None, []
        if ctx is not None:
            rs = ctx.env.robot_state()
            # bodies riding inside the drawer (the bowl) travel with it: at the mid/end probe poses
            # they are not where the (stale) world has them, so they are excluded from the PROBE
            # only; the executed segments re-synchronise the world and check them at their true poses
            c_r, rot_r, half_r = geo.site_box_world(env, joint.replace("_level", "_region"))   # white_cabinet_1_bottom_level -> white_cabinet_1_bottom_region
            riders = [b for b in ctx.sync.bodies if b != drawer_body and np.all(np.abs(rot_r.T @ (env.body_pose(b)[0] - c_r)) <= half_r)]
            extra_pairs = [(ctx.client.robot, b) for b in ctx.params.get("push_extra", [])]
            extra_pairs += [(ctx.client.robot, b) for b in riders] + [(f, b) for f in ctx.sync.fingers.values() for b in riders]
            if riders:
                ctx.record(event="push_probe_riders_excluded", riders=riders)
            cs = ctx.sync.grasp_collision_settings(drawer_body, extra_pairs)
            # posture diversity: the numerical IK is seeded; the current (post-push) configuration,
            # the episode's home configuration and two fixed elbow postures are tried per pose
            seeds = [rs.q] + ([ctx.params["home_q"]] if "home_q" in ctx.params else []) + \
                    [np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.8]), np.array([0.0, 0.6, 0.0, -1.6, 0.0, 2.2, 0.8])]
            lim = ctx.env.joint_limits()
            # posture families (JointPositionLimits IK constraint): unrestricted, shoulder forward
            # (joint 2 >= 0.3 rad: the forearm rises faster and clears objects next to the cabinet)
            # (a third family, base turned toward the drawer side, was tested in the final pass and
            # rejected every pose for the same reasons: link 5 vs the wine rack, wrist vs table)
            postures = [("free", None), ("shoulder_fwd", (np.maximum(lim[:, 0] + 0.05, [-9, 0.3, -9, -9, -9, -9, -9]), lim[:, 1] - 0.05))]
            for pname, jl in postures:
                for xo in LATERAL:
                    for h in HEIGHTS:
                        contact, probe = poses_for(xo, h)
                        ok, why = True, ""
                        for name in ("end", "contact", "mid", "pre"):   # the end pose is the discriminating one
                            found, last = False, ""
                            for seed in seeds:
                                try:
                                    sols, _, _ = ctx.client.ik(probe[name], rot, seed, max_solutions=8, collision_settings=cs, joint_limits=jl)
                                    if sols:
                                        found = True
                                        break
                                    last = "none"
                                except IntrinsicRequestError as e:
                                    m = re.search(r"Left object: (\S+)[^\n]*\n\s*Right objects?: (\S+)", str(e))
                                    last = ("collision:" + m.group(1).replace("panda.", "") + "/" + m.group(2).split(".")[0]) if m else str(e.code)
                                    if not m:
                                        break   # no solution at all (solver timeout, 5 s): more seeds never rescued such a pose in any probe
                            if not found:
                                ok, why = False, name + ":" + last
                                break
                        tried.append({"posture": pname, "lateral": xo, "height": h, "ok": ok, "why": why})
                        if ok:
                            chosen = (xo, h)
                            ctx.params["push_joint_limits"] = jl
                            break
                    if chosen:
                        break
                if chosen:
                    break
            ctx.record(event="push_contact_selection", chosen=chosen, posture=(tried[-1]["posture"] if tried else None), tried=tried)
            # movable objects that blocked the probed poses (for obstacle relocation by the composite skill)
            from collections import Counter
            partners = Counter(t["why"].split("/")[-1] for t in tried if "collision:" in t["why"])
            ctx.params["last_push_blockers"] = [b for b, _ in partners.most_common() if b in ctx.sync.bodies]
        xo, h = chosen or (LATERAL[0], HEIGHTS[0])
        contact, probe = poses_for(xo, h)
        return probe["pre"], rot, [contact, contact + axis * travel]

    def check_fn():
        return env.joint_qpos(joint) > 0.0
    return contact_fn, check_fn


def microwave_close_geometry(env, door_body: str, joint: str):
    def contact_fn(ctx=None):
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
        # the door rotates by -q about the axis to close; push the outside face along the arc with
        # a horizontal hand whose approach axis is the local tangent (perpendicular to the door face)
        sign = 1.0 if q < 0 else -1.0
        s_dir = sign * (1 if axis[2] > 0 else -1)
        n_seg = max(int(np.ceil(abs(q) / 0.3)), 2)
        path = []
        for k in range(1, n_seg + 1):
            a = ang0 + s_dir * abs(q) * k / n_seg
            p = anchor + np.array([r_edge * np.cos(a), r_edge * np.sin(a), z - anchor[2]])
            tangent = s_dir * np.array([-np.sin(a), np.cos(a), 0.0])
            path.append((p, geo.side_rotation(np.arctan2(tangent[1], tangent[0]), 0.0)))
        a_pre = ang0 - s_dir * 0.10
        contact = anchor + np.array([r_edge * np.cos(a_pre), r_edge * np.sin(a_pre), z - anchor[2]])
        tangent0 = s_dir * np.array([-np.sin(a_pre), np.cos(a_pre), 0.0])
        rot = geo.side_rotation(np.arctan2(tangent0[1], tangent0[0]), 0.0)
        pre = contact - tangent0 * 0.08
        return pre, rot, [contact] + path

    def check_fn():
        return env.joint_qpos(joint) > -0.005
    return contact_fn, check_fn
