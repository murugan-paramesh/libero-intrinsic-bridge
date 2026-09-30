"""Execute an Intrinsic-planned joint trajectory in LIBERO through the benchmark's own
OSC_POSE controller, and measure how faithfully it was tracked.

The benchmark action interface is Cartesian (delta TCP pose, world frame, +-0.05 m / +-0.5 rad per
20 Hz step). A joint-space trajectory therefore has to be converted: for each control step we
take the planned joint configuration at the current trajectory time, map it to a TCP pose with
Intrinsic ComputeFk (so the reference itself comes from Intrinsic's kinematic model), and command
the saturated delta toward that pose. The executed motion follows the planned path in task
space; joint-space deviation (the OSC null-space is pulled toward the robot's init posture by
robosuite, not toward our path) is measured and reported, never hidden.

Safety monitors: joint limits, per-step tracking error, total timeout, unexpected contacts.
"""
from __future__ import annotations

import dataclasses
import time
from typing import Callable, List, Optional

import numpy as np

from libero_intrinsic.env.libero_env import LiberoEnv
from libero_intrinsic.intrinsic.client import IntrinsicClient, Trajectory
from libero_intrinsic.model import transforms as tf

POS_SCALE = 0.05   # m per unit action (osc_pose.json output_max)
ROT_SCALE = 0.5    # rad per unit action
CONTROL_DT = 1.0 / 20.0


@dataclasses.dataclass
class ExecutionConfig:
    time_scale: float = 1.0          # >1 slows execution (trajectory time * time_scale)
    max_tcp_step: float = 0.015      # m per 20 Hz step the OSC controller is asked to follow (auto time scaling)
    max_rot_step_deg: float = 6.0    # deg per step
    pos_gain: float = 1.5            # error gain per step before saturation (tuned: 2 mm mean tcp error)
    rot_gain: float = 1.5
    lookahead_s: float = 0.10        # target time offset (compensates OSC lag)
    settle_pos_tol: float = 0.006    # m: final convergence tolerance
    settle_rot_tol_deg: float = 3.0
    settle_max_steps: int = 20
    max_pos_err: float = 0.08        # m: abort if tracking error exceeds this
    max_steps: int = 600
    gripper: float = -1.0            # gripper command held during the motion (-1 open, +1 close)
    contact_monitor: Optional[Callable[[], Optional[str]]] = None  # returns a reason string to abort


@dataclasses.dataclass
class ExecutionResult:
    trajectory_id: str
    ok: bool
    reason: str
    steps: int
    pos_err_max: float
    pos_err_mean: float
    rot_err_max_deg: float
    rot_err_mean_deg: float
    joint_err_max: float    # max |q_executed - q_planned| at matched times (rad, per-joint max)
    joint_err_final: float
    final_pos_err: float
    final_rot_err_deg: float
    wall_time_s: float
    log: List[dict]
    executed_q: Optional[np.ndarray] = None   # (steps, 7) joint samples actually reached
    time_scale: float = 1.0                   # execution slow-down actually applied


class TrajectoryExecutor:
    def __init__(self, env: LiberoEnv, client: IntrinsicClient, on_step: Optional[Callable] = None):
        self.env = env
        self.client = client
        self.on_step = on_step  # called after every env.step (e.g. video frame capture)

    def _tcp_refs(self, traj: Trajectory, cfg: ExecutionConfig):
        """Sample the trajectory at control rate and compute Intrinsic FK for every sample.

        Intrinsic's trajectories respect the Panda velocity limits and can be far faster than the
        20 Hz OSC controller can follow (its output saturates at 5 cm / 0.5 rad per step). The
        execution time scale is therefore raised automatically until no two consecutive TCP
        samples are farther apart than max_tcp_step / max_rot_step_deg; the path is unchanged."""
        scale = cfg.time_scale
        for _ in range(8):
            T = traj.duration * scale
            n = max(int(np.ceil(T / CONTROL_DT)), 1)
            times = np.linspace(0.0, T, n + 1)
            qs = np.array([traj.sample(t / scale) for t in times])
            poses = [self.client.fk(q) for q in qs]
            dpos = max((np.linalg.norm(poses[i + 1][0] - poses[i][0]) for i in range(n)), default=0.0)
            drot = max((tf.rot_error_deg(poses[i + 1][1], poses[i][1]) for i in range(n)), default=0.0)
            f = max(dpos / cfg.max_tcp_step, drot / cfg.max_rot_step_deg, 1.0)
            if f <= 1.02:
                break
            scale *= f * 1.05
        self.last_time_scale = scale
        return times, qs, poses

    def execute(self, traj: Trajectory, cfg: ExecutionConfig = ExecutionConfig()) -> ExecutionResult:
        t_wall = time.time()
        times, q_ref, poses = self._tcp_refs(traj, cfg)
        n = len(times)
        log, pos_errs, rot_errs, joint_errs, executed_q = [], [], [], [], []
        reason, ok = "completed", True
        lim = self.env.joint_limits()
        steps = 0
        # main tracking loop: at step k, target sample index is k + lookahead
        look = int(round(cfg.lookahead_s / CONTROL_DT))
        for k in range(n):
            rs = self.env.robot_state()
            idx = min(k + look, n - 1)
            p_t, R_t = poses[idx]
            dp = p_t - rs.tcp_pos
            drot = tf.rotvec_between(rs.tcp_rot, R_t)
            a = np.zeros(7)
            a[0:3] = np.clip(cfg.pos_gain * dp / POS_SCALE, -1, 1)
            a[3:6] = np.clip(cfg.rot_gain * drot / ROT_SCALE, -1, 1)
            a[6] = cfg.gripper
            self.env.step(a)
            steps += 1
            if self.on_step:
                self.on_step()
            rs2 = self.env.robot_state()
            pe = float(np.linalg.norm(poses[k][0] - rs2.tcp_pos))
            re = tf.rot_error_deg(poses[k][1], rs2.tcp_rot)
            je = float(np.max(np.abs(rs2.q - q_ref[k])))
            pos_errs.append(pe); rot_errs.append(re); joint_errs.append(je); executed_q.append(rs2.q.copy())
            log.append({"k": k, "t": float(times[k]), "pos_err": pe, "rot_err_deg": re, "joint_err": je,
                        "action": a.round(4).tolist()})
            if np.any(rs2.q < lim[:, 0] - 1e-3) or np.any(rs2.q > lim[:, 1] + 1e-3):
                ok, reason = False, "joint_limit_violation"
                break
            if pe > cfg.max_pos_err:
                ok, reason = False, f"tracking_error_exceeded({pe:.3f}m)"
                break
            if cfg.contact_monitor is not None:
                r = cfg.contact_monitor()
                if r:
                    ok, reason = False, f"contact:{r}"
                    break
            if steps >= cfg.max_steps:
                ok, reason = False, "timeout"
                break
        # settle on the final pose
        if ok:
            p_t, R_t = poses[-1]
            for _ in range(cfg.settle_max_steps):
                rs = self.env.robot_state()
                dp = p_t - rs.tcp_pos
                drot = tf.rotvec_between(rs.tcp_rot, R_t)
                if np.linalg.norm(dp) < cfg.settle_pos_tol and np.degrees(np.linalg.norm(drot)) < cfg.settle_rot_tol_deg:
                    break
                a = np.zeros(7)
                a[0:3] = np.clip(dp / POS_SCALE, -1, 1)
                a[3:6] = np.clip(drot / ROT_SCALE, -1, 1)
                a[6] = cfg.gripper
                self.env.step(a)
                steps += 1
                if self.on_step:
                    self.on_step()
        rs = self.env.robot_state()
        fp = float(np.linalg.norm(poses[-1][0] - rs.tcp_pos))
        fr = tf.rot_error_deg(poses[-1][1], rs.tcp_rot)
        fj = float(np.max(np.abs(rs.q - q_ref[-1])))
        if ok and (fp > 3 * cfg.settle_pos_tol or fr > 3 * cfg.settle_rot_tol_deg):
            ok, reason = False, f"final_pose_error({fp:.3f}m,{fr:.1f}deg)"
        return ExecutionResult(
            trajectory_id=traj.request_id, ok=ok, reason=reason, steps=steps,
            pos_err_max=float(np.max(pos_errs)) if pos_errs else 0.0,
            pos_err_mean=float(np.mean(pos_errs)) if pos_errs else 0.0,
            rot_err_max_deg=float(np.max(rot_errs)) if rot_errs else 0.0,
            rot_err_mean_deg=float(np.mean(rot_errs)) if rot_errs else 0.0,
            joint_err_max=float(np.max(joint_errs)) if joint_errs else 0.0,
            joint_err_final=fj, final_pos_err=fp, final_rot_err_deg=fr,
            wall_time_s=time.time() - t_wall, log=log, executed_q=np.array(executed_q) if executed_q else None,
            time_scale=float(self.last_time_scale))
