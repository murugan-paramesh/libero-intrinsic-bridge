"""Skill framework: every skill has preconditions, a bounded number of recovery attempts, a
step budget, explicit failure reasons, and writes structured records into the episode log."""
from __future__ import annotations

import dataclasses
import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from libero_intrinsic.env.executor import ExecutionConfig, ExecutionResult, TrajectoryExecutor
from libero_intrinsic.env.libero_env import LiberoEnv
from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicRequestError, Trajectory
from libero_intrinsic.intrinsic.world_sync import WorldSync


class BudgetExceeded(RuntimeError):
    pass


@dataclasses.dataclass
class SkillContext:
    env: LiberoEnv
    client: IntrinsicClient
    sync: WorldSync
    executor: TrajectoryExecutor
    step_budget: int
    log: List[dict] = dataclasses.field(default_factory=list)
    on_step: Optional[Callable[[], None]] = None
    params: Dict[str, Any] = dataclasses.field(default_factory=dict)

    def steps_used(self) -> int:
        return self.env.step_count

    def check_budget(self):
        if self.env.step_count >= self.step_budget:
            raise BudgetExceeded(f"step budget {self.step_budget} exhausted")

    def record(self, **kw):
        kw.setdefault("t_wall", time.time())
        kw.setdefault("env_step", self.env.step_count)
        self.log.append(kw)

    # ---------------------------------------------------------------- primitives
    def hold(self, gripper: float, steps: int):
        """Zero Cartesian motion, gripper command held for `steps` control steps."""
        a = np.zeros(7)
        a[6] = gripper
        for _ in range(steps):
            self.check_budget()
            self.env.step(a)
            if self.on_step:
                self.on_step()

    def close_gripper(self, steps: int = 12) -> np.ndarray:
        self.hold(+1.0, steps)
        return self.env.robot_state().finger_q

    def open_gripper(self, steps: int = 12) -> np.ndarray:
        self.hold(-1.0, steps)
        return self.env.robot_state().finger_q

    def execute(self, traj: Trajectory, cfg: ExecutionConfig, label: str) -> ExecutionResult:
        cfg.max_steps = min(cfg.max_steps, self.step_budget - self.env.step_count)
        if cfg.max_steps <= 0:
            raise BudgetExceeded("no steps left for execution")
        res = self.executor.execute(traj, cfg)
        self.record(event="execute", label=label, trajectory_id=traj.request_id, ok=res.ok, reason=res.reason,
                    steps=res.steps, pos_err_max=res.pos_err_max, pos_err_mean=res.pos_err_mean,
                    rot_err_max_deg=res.rot_err_max_deg, joint_err_max=res.joint_err_max,
                    joint_err_final=res.joint_err_final, final_pos_err=res.final_pos_err,
                    final_rot_err_deg=res.final_rot_err_deg, planned_duration_s=traj.duration,
                    planning_latency_s=traj.planning_latency_s, n_states=int(len(traj.t)), time_scale=res.time_scale)
        return res


@dataclasses.dataclass
class SkillResult:
    skill: str
    ok: bool
    reason: str = ""
    attempts: int = 1
    recoveries: int = 0
    steps: int = 0
    details: Dict[str, Any] = dataclasses.field(default_factory=dict)


class Skill:
    name = "skill"
    max_attempts = 1

    def preconditions(self, ctx: SkillContext) -> Optional[str]:
        return None

    def attempt(self, ctx: SkillContext, attempt_index: int) -> SkillResult:
        raise NotImplementedError

    def recover(self, ctx: SkillContext, last: SkillResult):
        """Bounded recovery between attempts (e.g. open gripper, retreat)."""

    def run(self, ctx: SkillContext) -> SkillResult:
        t0 = ctx.env.step_count
        pre = self.preconditions(ctx)
        if pre:
            r = SkillResult(self.name, False, f"precondition:{pre}", attempts=0)
            ctx.record(event="skill", skill=self.name, ok=False, reason=r.reason)
            return r
        last = None
        for i in range(self.max_attempts):
            ctx.check_budget()
            try:
                last = self.attempt(ctx, i)
            except IntrinsicRequestError as e:
                last = SkillResult(self.name, False, f"intrinsic:{e.rpc}:{e.code}:{e.details[:120]}")
            last.attempts = i + 1
            last.recoveries = i
            last.steps = ctx.env.step_count - t0
            ctx.record(event="skill_attempt", skill=self.name, attempt=i, ok=last.ok, reason=last.reason, details=last.details)
            if last.ok:
                break
            if i + 1 < self.max_attempts:
                try:
                    self.recover(ctx, last)
                except IntrinsicRequestError as e:
                    ctx.record(event="recover_failed", skill=self.name, reason=str(e))
        ctx.record(event="skill", skill=self.name, ok=last.ok, reason=last.reason, attempts=last.attempts, steps=last.steps)
        return last
