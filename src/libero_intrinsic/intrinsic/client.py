"""gRPC client for the Intrinsic Core services served by intrinsic_stack/cc/libero_planner_server.

Every consequential robotics computation (FK, IK, collision checking, path planning, trajectory
parameterization) happens inside Intrinsic Core; this module only builds requests with the
official protos (generated from intrinsic-core by scripts/gen_intrinsic_protos.py), sends them
and records request/response evidence.

Request construction mirrors intrinsic_sdk/intrinsic/motion_planning/motion_planner_client.py and
intrinsic_sdk/intrinsic/world/python/object_world_client.py (we do not import those modules
because they pull in the cluster/asset dependency stack).
"""
from __future__ import annotations

import dataclasses
import json
import os
import subprocess
import time
import uuid
from typing import Dict, List, Optional, Sequence, Tuple

import grpc
import numpy as np
from google.protobuf import text_format

from intrinsic.icon.proto import joint_space_pb2
from intrinsic.math.proto import pose_pb2
from intrinsic.motion_planning.proto.v1 import (
    geometric_constraints_pb2,
    motion_planner_service_pb2,
    motion_planner_service_pb2_grpc,
    motion_specification_pb2,
    robot_specification_pb2,
)
from intrinsic.world.proto import (
    collision_settings_pb2,
    object_world_refs_pb2,
    object_world_service_pb2,
    object_world_service_pb2_grpc,
    object_world_updates_pb2,
)

from libero_intrinsic.model import transforms as tf

ROOT_OBJECT = "root"


class IntrinsicUnavailableError(RuntimeError):
    """Raised when the Intrinsic backend cannot be reached. There is no fallback planner."""


class IntrinsicRequestError(RuntimeError):
    """An Intrinsic RPC returned a non-OK status (e.g. IK infeasible, planning failed)."""

    def __init__(self, rpc: str, code, details: str, request_id: str):
        super().__init__(f"{rpc} failed [{code}] {details} (request {request_id})")
        self.rpc, self.code, self.details, self.request_id = rpc, code, details, request_id


# ----------------------------------------------------------------------------- server process
@dataclasses.dataclass
class ServerAddresses:
    world_service: str
    motion_planner_service: str
    credentials: str


class IntrinsicServer:
    """Launches intrinsic_stack/cc/libero_planner_server as a subprocess and waits for its
    address file. The binary hosts the real Intrinsic services in-process."""

    def __init__(self, binary: str, worlds: Dict[str, str], log_dir: str, timeout_s: float = 120.0,
                 extra_args: Sequence[str] = ()):
        self.binary = binary
        self.worlds = dict(worlds)
        self.log_dir = log_dir
        self.timeout_s = timeout_s
        self.extra_args = list(extra_args)
        self.proc: Optional[subprocess.Popen] = None
        self.addresses: Optional[ServerAddresses] = None

    def start(self) -> ServerAddresses:
        if not os.path.isfile(self.binary) or not os.access(self.binary, os.X_OK):
            raise IntrinsicUnavailableError(f"planner server binary not found/executable: {self.binary}")
        os.makedirs(self.log_dir, exist_ok=True)
        addr_file = os.path.join(self.log_dir, f"intrinsic_addresses_{os.getpid()}_{int(time.time())}.json")
        if os.path.exists(addr_file):
            os.remove(addr_file)
        worlds_arg = ",".join(f"{k}={v}" for k, v in self.worlds.items())
        cmd = [self.binary, f"--worlds={worlds_arg}", f"--address_file={addr_file}"] + self.extra_args
        self._stderr = open(os.path.join(self.log_dir, "intrinsic_server.stderr.log"), "ab")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=self._stderr)
        t0 = time.time()
        while time.time() - t0 < self.timeout_s:
            if self.proc.poll() is not None:
                raise IntrinsicUnavailableError(
                    f"planner server exited with code {self.proc.returncode}; see {self._stderr.name}")
            if os.path.exists(addr_file):
                try:
                    with open(addr_file) as f:
                        d = json.load(f)
                    self.addresses = ServerAddresses(**d)
                    return self.addresses
                except (json.JSONDecodeError, TypeError):
                    pass
            time.sleep(0.2)
        self.stop()
        raise IntrinsicUnavailableError(f"planner server did not publish addresses within {self.timeout_s}s")

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.proc = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *a):
        self.stop()


# ----------------------------------------------------------------------------- request log
class RequestLog:
    """Append-only JSONL log of every Intrinsic RPC (id, timings, status, summaries). Full
    request/response protos are stored as textproto files next to it when `dump_protos`."""

    def __init__(self, path: str, dump_protos: bool = False):
        self.path = path
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.dump_dir = os.path.join(os.path.dirname(path), "intrinsic_protos") if dump_protos else None
        if self.dump_dir:
            os.makedirs(self.dump_dir, exist_ok=True)
        self.records: List[dict] = []

    def record(self, rec: dict, request=None, response=None):
        self.records.append(rec)
        with open(self.path, "a") as f:
            f.write(json.dumps(rec) + "\n")
        if self.dump_dir is not None and request is not None:
            with open(os.path.join(self.dump_dir, f"{rec['id']}.request.textproto"), "w") as f:
                f.write(text_format.MessageToString(request))
            if response is not None:
                with open(os.path.join(self.dump_dir, f"{rec['id']}.response.textproto"), "w") as f:
                    f.write(text_format.MessageToString(response))


# ----------------------------------------------------------------------------- refs
def obj_ref(name: str) -> object_world_refs_pb2.ObjectReference:
    return object_world_refs_pb2.ObjectReference(by_name=object_world_refs_pb2.ObjectReferenceByName(object_name=name))


def node_ref_object(name: str) -> object_world_refs_pb2.TransformNodeReference:
    return object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            object=object_world_refs_pb2.ObjectReferenceByName(object_name=name)))


def node_ref_frame(obj: str, frame: str) -> object_world_refs_pb2.TransformNodeReference:
    return object_world_refs_pb2.TransformNodeReference(
        by_name=object_world_refs_pb2.TransformNodeReferenceByName(
            frame=object_world_refs_pb2.FrameReferenceByName(object_name=obj, frame_name=frame)))


def frame_ref(obj: str, frame: str) -> object_world_refs_pb2.FrameReference:
    return object_world_refs_pb2.FrameReference(
        by_name=object_world_refs_pb2.FrameReferenceByName(object_name=obj, frame_name=frame))


@dataclasses.dataclass
class Trajectory:
    """A time-parameterized joint trajectory returned by Intrinsic PlanTrajectory."""
    request_id: str
    t: np.ndarray  # (N,) seconds since start
    q: np.ndarray  # (N, 7)
    qd: np.ndarray
    qdd: np.ndarray
    planning_latency_s: float

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    def sample(self, time_s: float) -> np.ndarray:
        """Linear interpolation of joint positions at time_s (clamped)."""
        if time_s <= self.t[0]:
            return self.q[0]
        if time_s >= self.t[-1]:
            return self.q[-1]
        i = int(np.searchsorted(self.t, time_s)) - 1
        a = (time_s - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        return (1 - a) * self.q[i] + a * self.q[i + 1]


# ----------------------------------------------------------------------------- client
class IntrinsicClient:
    def __init__(self, addresses: ServerAddresses, world_id: str, log: RequestLog,
                 robot_name: str = "panda", tcp_frame: str = "flange", rpc_timeout_s: float = 120.0):
        self.world_id = world_id
        self.robot = robot_name
        self.tcp_frame = tcp_frame
        self.log = log
        self.rpc_timeout_s = rpc_timeout_s
        creds = grpc.local_channel_credentials(grpc.LocalConnectionType.LOCAL_TCP)
        self._world_ch = grpc.secure_channel(addresses.world_service, creds)
        self._mp_ch = grpc.secure_channel(addresses.motion_planner_service, creds)
        self.world = object_world_service_pb2_grpc.ObjectWorldServiceStub(self._world_ch)
        self.planner = motion_planner_service_pb2_grpc.MotionPlannerServiceStub(self._mp_ch)
        self.addresses = addresses
        try:
            grpc.channel_ready_future(self._world_ch).result(timeout=10)
            grpc.channel_ready_future(self._mp_ch).result(timeout=10)
        except grpc.FutureTimeoutError as e:
            raise IntrinsicUnavailableError(f"cannot connect to Intrinsic services at {addresses}") from e
        # Object names in SDF-loaded worlds are not "global aliases", so by-name references are
        # rejected by the world service; resolve names to ids once and reference by id.
        self._ids: Dict[str, str] = {}
        self._frame_ids: Dict[Tuple[str, str], str] = {}
        self._refresh_ids()

    def _refresh_ids(self):
        req = object_world_service_pb2.ListObjectsRequest(world_id=self.world_id, view=object_world_updates_pb2.ObjectView.BASIC)
        _, resp, _ = self._call("ListObjects", self.world.ListObjects, req, {})
        self._ids = {o.name: o.id for o in resp.objects}
        for o in resp.objects:
            for fr in o.frames:
                self._frame_ids[(o.name, fr.name)] = fr.id
        if self.robot in self._ids:
            full = self.get_object(self.robot)
            for fr in full.frames:
                self._frame_ids[(self.robot, fr.name)] = fr.id
            if not full.name_is_global_alias:
                # the motion planner service resolves the robot by *name* internally, which
                # requires the global-alias flag (SDF-loaded objects do not have it)
                req = object_world_updates_pb2.UpdateObjectNameRequest(
                    world_id=self.world_id, object=self.oref(self.robot), name=self.robot,
                    name_is_global_alias=True, view=object_world_updates_pb2.ObjectView.BASIC)
                self._call("UpdateObjectName", self.world.UpdateObjectName, req, {"object": self.robot, "global_alias": True})

    def set_tcp_frame_offset(self, link_name: str, link_t_tcp_pos, link_t_tcp_rot):
        """Set link_t_tcp directly: UpdateTransform between the robot's `link_name` entity
        (selected with node_a_filter) and the tcp frame, updating the frame. Independent of any
        joint state (fixes the pose WorldFromSdf stored for the frame, see scene_to_sdf.py)."""
        req = object_world_updates_pb2.UpdateTransformRequest(
            world_id=self.world_id, node_a=self.nref(self.robot),
            node_a_filter=object_world_refs_pb2.ObjectEntityFilter(entity_names=[link_name]),
            node_b=self.nref_frame(self.robot, self.tcp_frame),
            node_to_update=self.nref_frame(self.robot, self.tcp_frame),
            a_t_b=tf.pose_to_proto(link_t_tcp_pos, link_t_tcp_rot, pose_pb2),
            view=object_world_updates_pb2.ObjectView.BASIC)
        self._call("UpdateTransform", self.world.UpdateTransform, req, {"object": f"{self.robot}/{self.tcp_frame}", "link": link_name})

    def ensure_tcp_frame(self, link_name: str, link_t_tcp_pos, link_t_tcp_rot):
        """Create the tcp frame on the robot's `link_name` entity if the world does not have it."""
        if (self.robot, self.tcp_frame) in self._frame_ids:
            return self._frame_ids[(self.robot, self.tcp_frame)]
        req = object_world_updates_pb2.CreateFrameRequest(
            world_id=self.world_id, new_frame_name=self.tcp_frame,
            parent_object_with_filter=object_world_refs_pb2.ObjectReferenceWithEntityFilter(
                reference=self.oref(self.robot),
                entity_filter=object_world_refs_pb2.ObjectEntityFilter(entity_names=[link_name])),
            parent_t_new_frame=tf.pose_to_proto(link_t_tcp_pos, link_t_tcp_rot, pose_pb2),
            designate_as_attachment_frame=True)
        _, resp, _ = self._call("CreateFrame", self.world.CreateFrame, req, {"frame": self.tcp_frame, "link": link_name})
        self._frame_ids[(self.robot, self.tcp_frame)] = resp.id
        return resp.id

    def oid(self, name: str) -> str:
        if name == ROOT_OBJECT:
            return ROOT_OBJECT
        if name not in self._ids:
            self._refresh_ids()
        return self._ids[name]

    def fid(self, obj: str, frame: str) -> str:
        return self._frame_ids[(obj, frame)]

    def oref(self, name: str) -> object_world_refs_pb2.ObjectReference:
        return object_world_refs_pb2.ObjectReference(id=self.oid(name), debug_hint=name)

    def nref(self, name: str) -> object_world_refs_pb2.TransformNodeReference:
        return object_world_refs_pb2.TransformNodeReference(id=self.oid(name), debug_hint=name)

    def nref_frame(self, obj: str, frame: str) -> object_world_refs_pb2.TransformNodeReference:
        return object_world_refs_pb2.TransformNodeReference(id=self.fid(obj, frame), debug_hint=f"{obj}/{frame}")

    def fref(self, obj: str, frame: str) -> object_world_refs_pb2.FrameReference:
        return object_world_refs_pb2.FrameReference(id=self.fid(obj, frame), debug_hint=f"{obj}/{frame}")

    # ---------------------------------------------------------------- plumbing
    def _call(self, rpc: str, stub_method, request, summary: dict, dump=False):
        rid = f"{rpc}-{uuid.uuid4().hex[:10]}"
        t0 = time.perf_counter()
        try:
            response = stub_method(request, timeout=self.rpc_timeout_s)
            status = "OK"
            err = None
        except grpc.RpcError as e:
            response = None
            status = e.code().name
            err = e.details()
        latency = time.perf_counter() - t0
        rec = {"id": rid, "rpc": rpc, "world_id": self.world_id, "t_wall": time.time(), "latency_s": latency,
               "status": status, "error": err, **summary}
        self.log.record(rec, request if (dump or self.log.dump_dir) else None, response)
        if response is None:
            if status == "UNAVAILABLE":
                raise IntrinsicUnavailableError(f"{rpc}: Intrinsic service unavailable: {err}")
            raise IntrinsicRequestError(rpc, status, err or "", rid)
        return rid, response, latency

    # ---------------------------------------------------------------- world queries/updates
    def list_objects(self) -> List[str]:
        req = object_world_service_pb2.ListObjectsRequest(world_id=self.world_id,
                                                          view=object_world_updates_pb2.ObjectView.BASIC)
        _, resp, _ = self._call("ListObjects", self.world.ListObjects, req, {})
        return [o.name for o in resp.objects]

    def get_object(self, name: str):
        req = object_world_service_pb2.GetObjectRequest(world_id=self.world_id, object=self.oref(name),
                                                        view=object_world_updates_pb2.ObjectView.FULL)
        _, resp, _ = self._call("GetObject", self.world.GetObject, req, {"object": name})
        return resp

    def get_transform(self, a_obj: str, b_obj: str, a_frame: Optional[str] = None, b_frame: Optional[str] = None):
        req = object_world_service_pb2.GetTransformRequest(
            world_id=self.world_id,
            node_a=self.nref_frame(a_obj, a_frame) if a_frame else self.nref(a_obj),
            node_b=self.nref_frame(b_obj, b_frame) if b_frame else self.nref(b_obj))
        _, resp, _ = self._call("GetTransform", self.world.GetTransform, req, {"a": a_obj, "b": b_obj})
        return tf.proto_to_pose(resp.a_t_b)

    def set_object_world_pose(self, name: str, pos, rot):
        """Set root_t_object for an object parented to root (UpdateTransform updates the child's
        parent_t_this so that root_t_object == the given pose)."""
        req = object_world_updates_pb2.UpdateTransformRequest(
            world_id=self.world_id, node_a=self.nref(ROOT_OBJECT), node_b=self.nref(name),
            node_to_update=self.nref(name), a_t_b=tf.pose_to_proto(pos, rot, pose_pb2),
            view=object_world_updates_pb2.ObjectView.BASIC)
        self._call("UpdateTransform", self.world.UpdateTransform, req, {"object": name})

    def reparent_to_frame(self, name: str, parent_obj: str, parent_frame: str):
        req = object_world_updates_pb2.ReparentObjectRequest(
            world_id=self.world_id, object=self.oref(name), parent_frame=self.fref(parent_obj, parent_frame),
            view=object_world_updates_pb2.ObjectView.BASIC)
        self._call("ReparentObject", self.world.ReparentObject, req, {"object": name, "parent": f"{parent_obj}/{parent_frame}"})

    def reparent_to_root(self, name: str):
        req = object_world_updates_pb2.ReparentObjectRequest(
            world_id=self.world_id, object=self.oref(name),
            parent_object=object_world_refs_pb2.ObjectReferenceWithEntityFilter(
                reference=self.oref(ROOT_OBJECT),
                entity_filter=object_world_refs_pb2.ObjectEntityFilter(include_base_entity=True)),
            view=object_world_updates_pb2.ObjectView.BASIC)
        self._call("ReparentObject", self.world.ReparentObject, req, {"object": name, "parent": ROOT_OBJECT})

    def update_robot_joints(self, q: Sequence[float]):
        """Store the current robot configuration in the world (used as the default start)."""
        req = object_world_updates_pb2.UpdateObjectJointsRequest(
            world_id=self.world_id, object=self.oref(self.robot), joint_positions=[float(v) for v in q],
            view=object_world_updates_pb2.ObjectView.BASIC)
        self._call("UpdateObjectJoints", self.world.UpdateObjectJoints, req, {"object": self.robot})

    # ---------------------------------------------------------------- kinematics
    def fk(self, q: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
        """root_t_tcp for joint vector q, computed by Intrinsic (ComputeFk)."""
        req = motion_planner_service_pb2.FkRequest(world_id=self.world_id)
        req.robot_reference.object_id.CopyFrom(self.oref(self.robot))
        req.joints.joints.extend([float(v) for v in q])
        req.reference.CopyFrom(self.nref(ROOT_OBJECT))
        req.target.CopyFrom(self.nref_frame(self.robot, self.tcp_frame))
        _, resp, _ = self._call("ComputeFk", self.planner.ComputeFk, req, {"n_joints": len(q)})
        return tf.proto_to_pose(resp.reference_t_target)

    def pose_target(self, pos, rot) -> geometric_constraints_pb2.GeometricConstraint:
        """GeometricConstraint: tcp frame == root_t_target (PoseEquality)."""
        c = geometric_constraints_pb2.GeometricConstraint()
        c.cartesian_pose.moving_frame.CopyFrom(self.nref_frame(self.robot, self.tcp_frame))
        c.cartesian_pose.target_frame.CopyFrom(self.nref(ROOT_OBJECT))
        c.cartesian_pose.target_frame_offset.CopyFrom(tf.pose_to_proto(pos, rot, pose_pb2))
        return c

    def joint_target(self, q) -> geometric_constraints_pb2.GeometricConstraint:
        c = geometric_constraints_pb2.GeometricConstraint()
        c.joint_position.joints.extend([float(v) for v in q])
        return c

    def ik(self, pos, rot, seed: Sequence[float], max_solutions: int = 8,
           collision_settings: Optional[collision_settings_pb2.CollisionSettings] = None,
           ensure_same_branch: bool = False, allow_collisions: bool = False) -> Tuple[List[np.ndarray], str, float]:
        req = motion_planner_service_pb2.IkRequest(world_id=self.world_id, max_num_solutions=max_solutions)
        req.robot_reference.object_id.CopyFrom(self.oref(self.robot))
        req.target.CopyFrom(self.pose_target(pos, rot))
        req.starting_joints.joints.extend([float(v) for v in seed])
        if collision_settings is not None:
            req.collision_settings.CopyFrom(collision_settings)
        if ensure_same_branch:
            req.ensure_same_branch = True
        if allow_collisions:
            req.disable_error_on_collisions = True
        rid, resp, lat = self._call("ComputeIk", self.planner.ComputeIk, req,
                                    {"target_pos": [float(v) for v in pos], "max_solutions": max_solutions})
        return [np.array(s.joints) for s in resp.solutions], rid, lat

    def check_collisions(self, waypoints: Sequence[Sequence[float]],
                         collision_settings: Optional[collision_settings_pb2.CollisionSettings] = None):
        req = motion_planner_service_pb2.CheckCollisionsRequest(world_id=self.world_id)
        req.robot_reference.object_id.CopyFrom(self.oref(self.robot))
        for w in waypoints:
            req.waypoint.add().joints.extend([float(v) for v in w])
        if collision_settings is not None:
            req.collision_settings.CopyFrom(collision_settings)
        rid, resp, lat = self._call("CheckCollisions", self.planner.CheckCollisions, req, {"n_waypoints": len(waypoints)})
        return bool(resp.has_collision), resp.collision_debug_msg, rid

    # ---------------------------------------------------------------- planning
    def plan_trajectory(self, q_start: Sequence[float],
                        target: geometric_constraints_pb2.GeometricConstraint,
                        collision_settings: Optional[collision_settings_pb2.CollisionSettings] = None,
                        motion_type: str = "ANY", timeout_s: float = 20.0,
                        path_constraint: Optional[geometric_constraints_pb2.UniformGeometricConstraint] = None,
                        caller_id: str = "libero_bridge") -> Trajectory:
        """PlanTrajectory: collision-free, time-parameterized trajectory from q_start to target."""
        req = motion_planner_service_pb2.MotionPlanningRequest(world_id=self.world_id, caller_id=caller_id)
        req.robot_specification.robot_reference.object_id.CopyFrom(self.oref(self.robot))
        req.robot_specification.start_configuration.joints.extend([float(v) for v in q_start])
        seg = req.motion_specification.motion_segments.add()
        seg.target.CopyFrom(target)
        seg.motion_type = motion_specification_pb2.MotionSegment.MotionType.Value(motion_type)
        if collision_settings is not None:
            seg.collision_settings.CopyFrom(collision_settings)
        if path_constraint is not None:
            seg.path_constraints.CopyFrom(path_constraint)
        req.motion_planner_config.timeout_sec.seconds = int(timeout_s)
        req.motion_planner_config.timeout_sec.nanos = int((timeout_s - int(timeout_s)) * 1e9)
        rid, resp, lat = self._call("PlanTrajectory", self.planner.PlanTrajectory, req,
                                    {"motion_type": motion_type, "timeout_s": timeout_s,
                                     "target_kind": target.WhichOneof("constraint")}, dump=True)
        traj = resp.discretized
        t = np.array([d.seconds + d.nanos * 1e-9 for d in traj.time_since_start])
        q = np.array([list(s.position) for s in traj.state])
        qd = np.array([list(s.velocity) for s in traj.state]) if traj.state and traj.state[0].velocity else np.zeros_like(q)
        qdd = np.array([list(s.acceleration) for s in traj.state]) if traj.state and traj.state[0].acceleration else np.zeros_like(q)
        rec = self.log.records[-1]
        rec.update({"n_states": int(len(t)), "duration_s": float(t[-1]) if len(t) else 0.0})
        return Trajectory(rid, t, q, qd, qdd, lat)

    def plan_to_pose(self, q_start, pos, rot, **kw) -> Trajectory:
        return self.plan_trajectory(q_start, self.pose_target(pos, rot), **kw)

    def plan_to_joints(self, q_start, q_goal, **kw) -> Trajectory:
        return self.plan_trajectory(q_start, self.joint_target(q_goal), **kw)

    def close(self):
        self._world_ch.close()
        self._mp_ch.close()


# ----------------------------------------------------------------------------- collision rules
def collision_settings(exclude_pairs: Sequence[Tuple[str, str]] = (), minimum_margin: Optional[float] = None,
                       disable_all: bool = False, resolver=None) -> collision_settings_pb2.CollisionSettings:
    """Build CollisionSettings. `exclude_pairs` lists (object_a, object_b) whose contact is
    intentional (e.g. grasped object vs gripper) and must not count as collision; every other
    pair keeps Intrinsic's default checking."""
    cs = collision_settings_pb2.CollisionSettings(disable_collision_checking=disable_all)
    if minimum_margin is not None:
        cs.minimum_margin = minimum_margin
    from intrinsic.world.proto import collision_action_pb2

    for a, b in exclude_pairs:
        rule = cs.collision_rules.add()
        rule.collision_action.CopyFrom(collision_action_pb2.CollisionAction(is_excluded=True))
        rule.left.add().object.CopyFrom(resolver(a) if resolver else obj_ref(a))
        rule.right.add().object.CopyFrom(resolver(b) if resolver else obj_ref(b))
    return cs
