"""Explicit conversion of the compiled LIBERO MuJoCo scene into an Intrinsic SDF world.

LIBERO does not ship a URDF; the robot is the robosuite Panda MJCF (robots/panda/robot.xml)
with the robosuite PandaGripper MJCF merged in. Intrinsic Core loads worlds from SDF
(intrinsic/world/conversion/sdf/world_from_sdf.cc). This module reads the *compiled*
MuJoCo model (so every <compiler> convention, mesh re-centering and body merge has already
been resolved by MuJoCo) and writes:

  * one kinematic SDF model "panda" with the 7 revolute arm joints, fixed joints for the
    hand chain (link7 -> right_hand -> right_gripper -> eef), a frame "tcp" attached to the
    eef link (== MuJoCo site gripper0_grip_site), joint position limits from jnt_range,
    velocity/acceleration/jerk limits from configs (documented), collision meshes exported
    from the compiled MuJoCo meshes as binary STL, and finger geometry fixed at the fully
    OPEN configuration (conservative envelope, see docs/architecture.md);
  * one static SDF model per environment body that carries collision geometry (tables,
    fixtures, articulated parts, movable objects), each a single link at that body's
    current world pose. Box/sphere/cylinder/capsule geoms map 1:1; mesh geoms are exported
    as STL; plane geoms (the floor) are skipped and documented.

Conventions handled here explicitly:
  * MuJoCo quaternions are wxyz. SDF poses are "x y z roll pitch yaw" (radians, extrinsic
    XYZ == intrinsic ZYX, the standard SDF/URDF convention).
  * MuJoCo geom size semantics: box = half-extents, sphere = radius, cylinder/capsule =
    (radius, half-length). SDF uses full extents / full lengths.
  * The world pose of the robot base body robot0_base is the SDF model pose, so
    Intrinsic's root frame == the MuJoCo world frame.
"""
from __future__ import annotations

import dataclasses
import os
import struct
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

# MuJoCo geom types (mjtGeom)
GEOM_PLANE, GEOM_HFIELD, GEOM_SPHERE, GEOM_CAPSULE, GEOM_ELLIPSOID, GEOM_CYLINDER, GEOM_BOX, GEOM_MESH = range(8)
JNT_FREE, JNT_BALL, JNT_SLIDE, JNT_HINGE = range(4)

ROBOT_MODEL_NAME = "panda"
TCP_FRAME_NAME = "flange"  # Intrinsic requires exactly one frame named "flange" (ISO 9787) on a robot for planning; ours is at the grip site
ARM_LINK_CHAIN = [
    "robot0_base", "robot0_link0", "robot0_link1", "robot0_link2", "robot0_link3",
    "robot0_link4", "robot0_link5", "robot0_link6", "robot0_link7", "robot0_right_hand",
    "gripper0_right_gripper", "gripper0_eef",
]
ARM_JOINTS = [f"robot0_joint{i}" for i in range(1, 8)]
FINGER_BODIES = {  # finger body -> (slide joint, fully-open joint value)
    "gripper0_leftfinger": ("gripper0_finger_joint1", 0.04),
    "gripper0_rightfinger": ("gripper0_finger_joint2", -0.04),
}
FINGER_TIP_BODIES = {"gripper0_finger_joint1_tip": "gripper0_leftfinger", "gripper0_finger_joint2_tip": "gripper0_rightfinger"}
ROBOT_BODY_PREFIXES = ("robot0_", "gripper0_", "mount0_")

# Franka Panda joint dynamic limits (Franka Control Interface documentation, "Limits").
# These are not in the LIBERO MJCF (it only has position ranges and torque limits) and are
# provided explicitly as the planning limits. Velocity limits are the official Panda ones;
# acceleration and jerk are scaled down (see docs/architecture.md) because the LIBERO
# OSC_POSE controller cannot track very aggressive joint trajectories.
PANDA_VEL_LIMITS = [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]
PANDA_ACC_LIMITS = [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
PANDA_JERK_LIMITS = [7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0]
PANDA_EFFORT_LIMITS = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]


def quat_wxyz_to_rpy(q: np.ndarray) -> np.ndarray:
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_euler("xyz")


def rot_to_rpy(rot: np.ndarray) -> np.ndarray:
    return R.from_matrix(rot).as_euler("xyz")


def pose_str(pos: np.ndarray, rpy: np.ndarray) -> str:
    v = list(np.asarray(pos, dtype=float)) + list(np.asarray(rpy, dtype=float))
    return " ".join(f"{x:.9g}" for x in v)


def quat_wxyz_to_mat(q):
    return R.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()


@dataclasses.dataclass
class ExportedBody:
    """One environment body exported as a static single-link model."""
    body: str
    model_name: str
    world_pos: np.ndarray
    world_rot: np.ndarray
    n_geoms: int


@dataclasses.dataclass
class SdfWorldSpec:
    sdf_path: str
    asset_dir: str
    robot_model: str
    tcp_frame: str
    arm_joints: List[str]
    exported_bodies: List[ExportedBody]
    skipped_geoms: List[str]
    robot_base_pos: np.ndarray
    robot_base_rot: np.ndarray
    tcp_link: str = "gripper0_eef"           # link entity the tcp frame is attached to
    tcp_link_t_tcp_pos: np.ndarray = None    # site offset in that link (== gripper0_grip_site)
    tcp_link_t_tcp_rot: np.ndarray = None
    finger_objects: Dict[str, str] = None    # MuJoCo finger body -> Intrinsic object name


class MeshExporter:
    """Writes compiled MuJoCo meshes to binary STL files (deduplicated by mesh id)."""

    def __init__(self, model, asset_dir: str):
        self.m = model
        self.asset_dir = asset_dir
        os.makedirs(asset_dir, exist_ok=True)
        self._written: Dict[int, str] = {}

    def stl_path(self, mesh_id: int) -> str:
        if mesh_id in self._written:
            return self._written[mesh_id]
        m = self.m
        name = m.mesh_id2name(mesh_id) or f"mesh{mesh_id}"
        name = name.replace("/", "_")
        va, vn = int(m.mesh_vertadr[mesh_id]), int(m.mesh_vertnum[mesh_id])
        fa, fn = int(m.mesh_faceadr[mesh_id]), int(m.mesh_facenum[mesh_id])
        verts = np.asarray(m.mesh_vert[va:va + vn], dtype=np.float32).reshape(-1, 3)
        faces = np.asarray(m.mesh_face[fa:fa + fn], dtype=np.int64).reshape(-1, 3)
        path = os.path.abspath(os.path.join(self.asset_dir, f"{name}.stl"))
        tri = verts[faces]  # (F,3,3)
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        with open(path, "wb") as f:
            f.write(b"\0" * 80)
            f.write(struct.pack("<I", len(faces)))
            rec = np.zeros(len(faces), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
            rec["n"] = n
            rec["v"] = tri
            f.write(rec.tobytes())
        self._written[mesh_id] = path
        return path


class SceneToSdf:
    def __init__(self, env, out_dir: str, include_visual: bool = False):
        """env: libero_intrinsic.env.libero_env.LiberoEnv (after reset)."""
        self.env = env
        self.m = env.model
        self.d = env.data
        self.out_dir = out_dir
        self.asset_dir = os.path.join(out_dir, "meshes")
        self.meshes = MeshExporter(self.m, self.asset_dir)
        self.skipped: List[str] = []
        self.exported: List[ExportedBody] = []
        self.finger_objects: Dict[str, str] = {}  # finger body -> model name

    # ------------------------------------------------------------------ helpers
    def _bid(self, name):
        return self.m.body_name2id(name)

    def _is_collision_geom(self, gi: int) -> bool:
        m = self.m
        return int(m.geom_contype[gi]) != 0 or int(m.geom_conaffinity[gi]) != 0

    def _geom_xml(self, gi: int, name: str, extra_pos=None, extra_rot=None, indent="      ") -> Optional[str]:
        """SDF <collision> for geom gi, posed relative to its body (optionally pre-multiplied
        by an extra body-relative transform (extra_pos, extra_rot))."""
        m = self.m
        gtype = int(m.geom_type[gi])
        size = np.array(m.geom_size[gi])
        pos = np.array(m.geom_pos[gi])
        rot = quat_wxyz_to_mat(m.geom_quat[gi])
        if extra_pos is not None:
            pos = extra_pos + extra_rot @ pos
            rot = extra_rot @ rot
        if gtype == GEOM_BOX:
            shape = f"<box><size>{2*size[0]:.9g} {2*size[1]:.9g} {2*size[2]:.9g}</size></box>"
        elif gtype == GEOM_SPHERE:
            shape = f"<sphere><radius>{size[0]:.9g}</radius></sphere>"
        elif gtype == GEOM_CYLINDER:
            shape = f"<cylinder><radius>{size[0]:.9g}</radius><length>{2*size[1]:.9g}</length></cylinder>"
        elif gtype == GEOM_CAPSULE:
            shape = f"<capsule><radius>{size[0]:.9g}</radius><length>{2*size[1]:.9g}</length></capsule>"
        elif gtype == GEOM_MESH:
            path = self.meshes.stl_path(int(m.geom_dataid[gi]))
            shape = f"<mesh><uri>bypass://{path}</uri></mesh>"
        elif gtype == GEOM_ELLIPSOID:
            # approximate by bounding box (conservative)
            shape = f"<box><size>{2*size[0]:.9g} {2*size[1]:.9g} {2*size[2]:.9g}</size></box>"
            self.skipped.append(f"{name}: ellipsoid approximated by box")
        else:
            self.skipped.append(f"{name}: geom type {gtype} not exported")
            return None
        return (f'{indent}<collision name="{name}">\n'
                f"{indent}  <pose>{pose_str(pos, rot_to_rpy(rot))}</pose>\n"
                f"{indent}  <geometry>{shape}</geometry>\n"
                f"{indent}</collision>\n")

    def _body_geoms(self, bid: int) -> List[int]:
        m = self.m
        return [gi for gi in range(m.ngeom) if int(m.geom_bodyid[gi]) == bid and self._is_collision_geom(gi)]

    def _inertial_xml(self, indent="      ") -> str:
        return (f"{indent}<inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz>"
                f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>\n")

    # ------------------------------------------------------------------ robot
    def _finger_geoms_open(self, hand_body: str) -> str:
        """Finger + pad collision geoms expressed in the hand (right_gripper) frame with the
        fingers at their fully-open joint value."""
        m = self.m
        out = ""
        hand_id = self._bid(hand_body)
        for fbody, (jname, q_open) in FINGER_BODIES.items():
            fid = self._bid(fbody)
            # finger body frame relative to its parent (which must be the hand body)
            assert int(m.body_parentid[fid]) == hand_id, (fbody, m.body_id2name(m.body_parentid[fid]))
            p = np.array(m.body_pos[fid])
            rot = quat_wxyz_to_mat(m.body_quat[fid])
            jid = m.joint_name2id(jname)
            axis = np.array(m.jnt_axis[jid])
            assert int(m.jnt_type[jid]) == JNT_SLIDE
            p = p + rot @ (axis * q_open)  # slide joint translates along axis in child frame
            for gi in self._body_geoms(fid):
                s = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", p, rot)
                if s:
                    out += s
            # tip child bodies (pads)
            for tip, parent in FINGER_TIP_BODIES.items():
                if parent != fbody:
                    continue
                tid = self._bid(tip)
                tp = p + rot @ np.array(m.body_pos[tid])
                trot = rot @ quat_wxyz_to_mat(m.body_quat[tid])
                for gi in self._body_geoms(tid):
                    s = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", tp, trot)
                    if s:
                        out += s
        return out

    def robot_model_xml(self) -> Tuple[str, np.ndarray, np.ndarray]:
        m = self.m
        base_id = self._bid(ARM_LINK_CHAIN[0])
        base_pos = np.array(self.d.get_body_xpos(ARM_LINK_CHAIN[0]))
        base_rot = np.array(self.d.get_body_xmat(ARM_LINK_CHAIN[0])).reshape(3, 3)
        x = f'    <model name="{ROBOT_MODEL_NAME}">\n'
        x += f"      <pose>{pose_str(base_pos, rot_to_rpy(base_rot))}</pose>\n"
        x += f'      <intrinsic:ik_solver tip_link_name="{ARM_LINK_CHAIN[-1]}">kinematic_chain</intrinsic:ik_solver>\n'
        # links
        prev = None
        joint_xml = ""
        for i, body in enumerate(ARM_LINK_CHAIN):
            bid = self._bid(body)
            if prev is not None:
                assert int(m.body_parentid[bid]) == self._bid(prev), (body, prev)
            # link pose: relative to parent link (SDF relative_to), from compiled body_pos/quat
            if prev is None:
                x += f'      <link name="{body}">\n        <pose>0 0 0 0 0 0</pose>\n'
            else:
                x += (f'      <link name="{body}">\n        <pose relative_to="{prev}">'
                      f"{pose_str(m.body_pos[bid], quat_wxyz_to_rpy(m.body_quat[bid]))}</pose>\n")
            x += self._inertial_xml("        ")
            for gi in self._body_geoms(bid):
                s = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", indent="        ")
                if s:
                    x += s
            x += "      </link>\n"
            # joint from prev to this
            if prev is not None:
                jids = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) == bid]
                if len(jids) == 0:
                    joint_xml += (f'      <joint name="{prev}_to_{body}" type="fixed">\n'
                                  f"        <parent>{prev}</parent><child>{body}</child>\n      </joint>\n")
                elif len(jids) == 1 and int(m.jnt_type[jids[0]]) == JNT_HINGE:
                    j = jids[0]
                    jname = m.joint_id2name(j)
                    k = ARM_JOINTS.index(jname)
                    lo, hi = m.jnt_range[j]
                    jpos = np.array(m.jnt_pos[j])
                    axis = np.array(m.jnt_axis[j])
                    joint_xml += (
                        f'      <joint name="{jname}" type="revolute">\n'
                        f'        <pose relative_to="{body}">{pose_str(jpos, [0,0,0])}</pose>\n'
                        f"        <parent>{prev}</parent><child>{body}</child>\n"
                        f"        <axis>\n"
                        f"          <xyz>{axis[0]:.9g} {axis[1]:.9g} {axis[2]:.9g}</xyz>\n"
                        f"          <limit><lower>{lo:.9g}</lower><upper>{hi:.9g}</upper>"
                        f"<effort>{PANDA_EFFORT_LIMITS[k]}</effort><velocity>{PANDA_VEL_LIMITS[k]}</velocity>"
                        f"<intrinsic:acceleration>{PANDA_ACC_LIMITS[k]}</intrinsic:acceleration>"
                        f"<intrinsic:jerk>{PANDA_JERK_LIMITS[k]}</intrinsic:jerk></limit>\n"
                        f"          <dynamics><damping>{float(m.dof_damping[m.jnt_dofadr[j]]):.6g}</damping></dynamics>\n"
                        f"        </axis>\n      </joint>\n"
                    )
                else:
                    raise RuntimeError(f"unexpected joints on {body}: {[m.joint_id2name(j) for j in jids]}")
            prev = body
        x += joint_xml
        # TCP frame attached to the eef link (== site gripper0_grip_site). NOTE: WorldFromSdf
        # resolves the frame's world pose with all joints at zero, but joint 4's zero lies outside
        # its limits, so Intrinsic initialises that joint at mid-range and stores a wrong
        # link_t_frame. The client therefore re-sets link_t_tcp at runtime (UpdateTransform with
        # base_t_tcp taken from MuJoCo at the synced joint vector); see IntrinsicClient.calibrate_tcp_frame.
        site_id = m.site_name2id("gripper0_grip_site")
        assert m.body_id2name(m.site_bodyid[site_id]) == ARM_LINK_CHAIN[-1]
        self.tcp_site_pos = np.array(m.site_pos[site_id])
        self.tcp_site_rot = quat_wxyz_to_mat(m.site_quat[site_id])
        x += (f'      <frame name="{TCP_FRAME_NAME}" attached_to="{ARM_LINK_CHAIN[-1]}" intrinsic:create_attachment_entity="true">'
              f"<pose>{pose_str(self.tcp_site_pos, rot_to_rpy(self.tcp_site_rot))}</pose></frame>\n")
        x += "    </model>\n"
        return x, base_pos, base_rot

    # ------------------------------------------------------------------ fingers
    FINGER_MODELS = {"gripper0_leftfinger": "gripper_finger_left", "gripper0_rightfinger": "gripper_finger_right"}

    def finger_models_xml(self) -> str:
        """Each finger (finger body + its pad tip body) becomes a separate single-link model at
        its current world pose. At runtime WorldSync parents both to the robot's flange frame
        and updates flange_t_finger from the actual finger joint values, so the collision model
        reflects the true finger opening (open during approach, closed around a grasped object)."""
        m = self.m
        x = ""
        for fbody, model_name in self.FINGER_MODELS.items():
            fid = self._bid(fbody)
            wp = np.array(self.d.get_body_xpos(fbody))
            wr = np.array(self.d.get_body_xmat(fbody)).reshape(3, 3)
            x += f'    <model name="{model_name}">\n      <static>true</static>\n      <pose>{pose_str(wp, rot_to_rpy(wr))}</pose>\n'
            x += '      <link name="link">\n        <pose>0 0 0 0 0 0</pose>\n' + self._inertial_xml("        ")
            n = 0
            for gi in self._body_geoms(fid):
                sx = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", indent="        ")
                if sx:
                    x += sx; n += 1
            for tip, parent in FINGER_TIP_BODIES.items():
                if parent != fbody:
                    continue
                tid = self._bid(tip)
                tp = np.array(m.body_pos[tid]); trot = quat_wxyz_to_mat(m.body_quat[tid])
                for gi in self._body_geoms(tid):
                    sx = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", tp, trot, indent="        ")
                    if sx:
                        x += sx; n += 1
            x += "      </link>\n    </model>\n"
            self.finger_objects[fbody] = model_name
        return x

    # ------------------------------------------------------------------ environment
    def env_models_xml(self) -> str:
        m = self.m
        x = ""
        for bid in range(1, m.nbody):
            body = m.body_id2name(bid)
            if body.startswith(ROBOT_BODY_PREFIXES):
                continue
            geoms = self._body_geoms(bid)
            geoms = [g for g in geoms if int(m.geom_type[g]) != GEOM_PLANE or self.skipped.append(f"{m.geom_id2name(g)}: plane skipped")]
            if not geoms:
                continue
            wp = np.array(self.d.get_body_xpos(body))
            wr = np.array(self.d.get_body_xmat(body)).reshape(3, 3)
            x += f'    <model name="{body}">\n      <static>true</static>\n      <pose>{pose_str(wp, rot_to_rpy(wr))}</pose>\n'
            x += f'      <link name="link">\n        <pose>0 0 0 0 0 0</pose>\n' + self._inertial_xml("        ")
            n = 0
            for gi in geoms:
                s = self._geom_xml(gi, m.geom_id2name(gi) or f"g{gi}", indent="        ")
                if s:
                    x += s
                    n += 1
            x += "      </link>\n    </model>\n"
            self.exported.append(ExportedBody(body, body, wp, wr, n))
        return x

    def write(self, name: str = "world") -> SdfWorldSpec:
        os.makedirs(self.out_dir, exist_ok=True)
        robot_xml, bpos, brot = self.robot_model_xml()
        finger_xml = self.finger_models_xml()
        env_xml = self.env_models_xml()
        sdf = ('<?xml version="1.0"?>\n'
               '<sdf version="1.9" xmlns:intrinsic="https://intrinsic.ai/">\n'
               '  <world name="default">\n' + robot_xml + finger_xml + env_xml + "  </world>\n</sdf>\n")
        path = os.path.join(self.out_dir, f"{name}.sdf")
        with open(path, "w") as f:
            f.write(sdf)
        return SdfWorldSpec(path, self.asset_dir, ROBOT_MODEL_NAME, TCP_FRAME_NAME, list(ARM_JOINTS),
                            self.exported, list(self.skipped), bpos, brot, ARM_LINK_CHAIN[-1],
                            self.tcp_site_pos, self.tcp_site_rot, dict(self.finger_objects))
