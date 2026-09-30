"""Task specification derived from the LIBERO env's parsed BDDL problem.

We do not hand-write per-task scripts: the goal atoms come from `env.env.parsed_problem`
["goal_state"] (the same structure LIBERO evaluates), and each atom maps to a skill sequence
via SKILL_FOR_PREDICATE. Only object-relative geometric parameters (grasp offsets, place
heights) are configurable per object category (configs/objects.yaml).
"""
from __future__ import annotations

import dataclasses
from typing import List, Optional

import numpy as np


@dataclasses.dataclass
class GoalAtom:
    predicate: str          # in, on, turnon, close, open ...
    args: List[str]
    subgoal_index: int


@dataclasses.dataclass
class TaskSpec:
    name: str
    language: str
    goal: List[GoalAtom]
    objects: List[str]
    fixtures: List[str]
    regions: dict           # region name -> {"target": body/site owner, "site": mujoco site name}


def region_site_name(env, region: str) -> Optional[str]:
    """LIBERO region names are MuJoCo site names ('<target>_<region>')."""
    m = env.model
    try:
        m.site_name2id(region)
        return region
    except Exception:
        return None


def task_spec_from_env(env, task_name: str, language: str) -> TaskSpec:
    parsed = env.env.env.parsed_problem
    goal = []
    for i, atom in enumerate(parsed["goal_state"]):
        goal.append(GoalAtom(predicate=atom[0].lower(), args=[a for a in atom[1:]], subgoal_index=i))
    regions = {}
    for name in parsed.get("regions", {}):
        s = region_site_name(env, name)
        regions[name] = {"site": s, "target": parsed["regions"][name].get("target")}
    return TaskSpec(name=task_name, language=language, goal=goal,
                    objects=list(parsed["obj_of_interest"]) if "obj_of_interest" in parsed else env.object_names(),
                    fixtures=env.fixture_names(), regions=regions)
