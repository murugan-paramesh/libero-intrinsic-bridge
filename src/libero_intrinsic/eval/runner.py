"""Episode runner: LIBERO env + Intrinsic planner server + skill sequencing + evidence.

One EpisodeRecord (JSON) per episode with run id, software revisions, task/init identity,
controller configuration, Intrinsic endpoints, every RPC (ids + latency), every skill outcome,
planned-vs-executed tracking summaries, the official success result and the video path.
"""
from __future__ import annotations

import dataclasses
import json
import os
import subprocess
import time
import traceback
import uuid
from typing import Dict, List, Optional

import numpy as np

from libero_intrinsic.env.executor import TrajectoryExecutor
from libero_intrinsic.env.libero_env import LiberoEnv, TaskInfo, list_tasks
from libero_intrinsic.intrinsic.client import IntrinsicClient, IntrinsicServer, IntrinsicUnavailableError, RequestLog
from libero_intrinsic.intrinsic.world_sync import WorldSync
from libero_intrinsic.model.scene_to_sdf import SceneToSdf, SdfWorldSpec
from libero_intrinsic.skills.base import BudgetExceeded, SkillContext
from libero_intrinsic.skills.task_spec import task_spec_from_env
from libero_intrinsic.skills.planner import build_skill_sequence

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DEFAULT_BINARY = "/home/user/bazel_out/execroot/_main/bazel-out/k8-opt/bin/libero_bridge/libero_planner_server"


def git_rev(path: str) -> str:
    try:
        return subprocess.check_output(["git", "-C", path, "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


def software_revisions() -> Dict[str, str]:
    import mujoco, robosuite
    return {
        "libero_intrinsic_bridge": git_rev(ROOT),
        "intrinsic_core": git_rev(os.path.join(ROOT, "third_party", "intrinsic-core")),
        "LIBERO": git_rev(os.path.join(ROOT, "third_party", "LIBERO")),
        "robosuite": robosuite.__version__,
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
    }


class VideoRecorder:
    def __init__(self, env: LiberoEnv, path: str, fps: int = 10, camera: str = "agentview", every: int = 2):
        import imageio
        self.env, self.path, self.every, self.camera = env, path, every, camera
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.writer = imageio.get_writer(path, fps=fps, codec="libx264", quality=7, macro_block_size=1)
        self.n = 0

    def __call__(self):
        if self.n % self.every == 0:
            self.writer.append_data(self.env.render_frame(self.camera))
        self.n += 1

    def close(self):
        self.writer.close()


@dataclasses.dataclass
class EpisodeConfig:
    step_budget: int = 1200            # 20 Hz control steps (60 s simulated)
    camera_hw: tuple = (256, 256)
    video: bool = True
    seed: int = 0
    dump_protos: bool = False
    observation_mode: str = "privileged_state"   # object poses read from the simulator (documented assumption)
    controller: str = "OSC_POSE (robosuite 1.4.0 default osc_pose.json, 20 Hz)"


class TaskSession:
    """Everything that is shared across episodes of one task: env, SDF world, server, client."""

    def __init__(self, task: TaskInfo, out_dir: str, cfg: EpisodeConfig, binary: str = DEFAULT_BINARY,
                 server: Optional[IntrinsicServer] = None):
        self.task, self.out_dir, self.cfg = task, out_dir, cfg
        os.makedirs(out_dir, exist_ok=True)
        self.env = LiberoEnv(task, camera_hw=cfg.camera_hw, seed=cfg.seed)
        self.env.reset_to(0)
        self.world_dir = os.path.join(out_dir, "world")
        self.spec: SdfWorldSpec = SceneToSdf(self.env, self.world_dir).write(f"task{task.index}")
        self.world_id = f"libero10_task{task.index}"
        self.owns_server = server is None
        self.server = server or IntrinsicServer(binary, {self.world_id: self.spec.sdf_path}, os.path.join(out_dir, "server"))
        if self.owns_server:
            self.server.start()
        self.binary = binary

    def close(self):
        if self.owns_server:
            self.server.stop()
        self.env.close()

    def run_episode(self, init_index: int, run_id: Optional[str] = None) -> dict:
        run_id = run_id or f"t{self.task.index}_i{init_index}_{uuid.uuid4().hex[:8]}"
        ep_dir = os.path.join(self.out_dir, "episodes", run_id)
        os.makedirs(ep_dir, exist_ok=True)
        log = RequestLog(os.path.join(ep_dir, "intrinsic_requests.jsonl"), dump_protos=self.cfg.dump_protos)
        rec = {
            "run_id": run_id, "task_index": self.task.index, "task_name": self.task.name, "language": self.task.language,
            "init_state_index": int(init_index), "seed": self.cfg.seed, "revisions": software_revisions(),
            "observation_mode": self.cfg.observation_mode, "controller": self.cfg.controller,
            "step_budget": self.cfg.step_budget,
            "intrinsic": {"component": "MotionPlannerServiceInProcess + FakeWorldService (intrinsic-core, in-process gRPC)",
                          "binary": self.binary, "world_id": self.world_id, "sdf": self.spec.sdf_path,
                          "addresses": dataclasses.asdict(self.server.addresses)},
            "t_start": time.time(), "success": False, "error": None, "skills": [], "events": [],
        }
        video = None
        try:
            self.env.reset_to(init_index)
            client = IntrinsicClient(self.server.addresses, self.world_id, log)
            sync = WorldSync(self.env, client, self.spec)
            sr = sync.sync(verify=True)
            rec["world_sync"] = dataclasses.asdict(sr)
            video = VideoRecorder(self.env, os.path.join(ep_dir, "agentview.mp4")) if self.cfg.video else None
            executor = TrajectoryExecutor(self.env, client, on_step=video)
            ctx = SkillContext(self.env, client, sync, executor, self.cfg.step_budget, on_step=video)
            spec = task_spec_from_env(self.env, self.task.name, self.task.language)
            rec["goal"] = [dataclasses.asdict(g) for g in spec.goal]
            skills = build_skill_sequence(spec, self.env, ctx)
            rec["plan"] = [s.name + ":" + getattr(s, "body", "") for s in skills]
            for s in skills:
                r = s.run(ctx)
                rec["skills"].append(dataclasses.asdict(r))
                if not r.ok:
                    rec["failure_stage"] = f"{s.name}:{getattr(s, 'body', '')}"
                    rec["failure_reason"] = r.reason
                    break
                if self.env.check_success():
                    break
            rec["success"] = bool(self.env.check_success())
            rec["events"] = ctx.log
        except BudgetExceeded as e:
            rec["failure_stage"] = rec.get("failure_stage", "budget")
            rec["failure_reason"] = str(e)
            rec["success"] = bool(self.env.check_success())
        except IntrinsicUnavailableError as e:
            rec["error"] = f"infrastructure:intrinsic_unavailable:{e}"
        except Exception as e:  # infrastructure/other errors are recorded, never hidden
            rec["error"] = f"exception:{type(e).__name__}:{e}"
            rec["traceback"] = traceback.format_exc()
        finally:
            if video is not None:
                video.close()
                rec["video"] = video.path
        rec["steps"] = self.env.step_count
        rec["t_end"] = time.time()
        rec["wall_time_s"] = rec["t_end"] - rec["t_start"]
        rpcs = log.records
        rec["intrinsic"]["n_requests"] = len(rpcs)
        rec["intrinsic"]["requests_by_rpc"] = {k: sum(1 for r in rpcs if r["rpc"] == k) for k in sorted({r["rpc"] for r in rpcs})}
        plans = [r for r in rpcs if r["rpc"] == "PlanTrajectory"]
        rec["intrinsic"]["plan_latency_s"] = [r["latency_s"] for r in plans]
        rec["intrinsic"]["plan_failures"] = sum(1 for r in plans if r["status"] != "OK")
        rec["intrinsic"]["plan_timeouts"] = sum(1 for r in plans if r["status"] == "DEADLINE_EXCEEDED")
        with open(os.path.join(ep_dir, "episode.json"), "w") as f:
            json.dump(rec, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o))
        return rec
