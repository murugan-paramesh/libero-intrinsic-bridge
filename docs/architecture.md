# Architecture

```
LIBERO (robosuite/MuJoCo, OSC_POSE @ 20 Hz)                      Intrinsic Core (C++), one process
┌──────────────────────────────┐                                   ┌───────────────────────────────────┐
│ LiberoEnv (env/libero_env.py)│  MuJoCo body/joint state          │ libero_planner_server             │
│  reset_to(init i), step(a)   │ ───────────────────────────────►  │  WorldFromSdf  -> World           │
│  robot_state(), body_pose()  │  UpdateTransform / ReparentObject │  FakeWorldService (ObjectWorld-   │
│  contacts(), check_success() │  UpdateObjectJoints (gRPC)        │   Service on local TCP)           │
└──────────────┬───────────────┘                                   │  MotionPlannerServiceInProcess    │
               │ 7-D OSC actions                                   │   ComputeFk / ComputeIk /         │
┌──────────────┴───────────────┐  PlanTrajectory / ComputeIk /     │   PlanTrajectory (RRT-Connect +   │
│ TrajectoryExecutor           │  ComputeFk / CheckCollisions      │   shortcut + TOPP) /              │
│  (env/executor.py)           │ ◄──────────────────────────────►  │   CheckCollisions (COAL)          │
└──────────────┬───────────────┘  JointTrajectoryPVA               └───────────────────────────────────┘
               │
┌──────────────┴───────────────┐   ┌──────────────────────────┐   ┌──────────────────────────────┐
│ Skills (skills/*.py)         │◄──│ Predicate planner        │◄──│ TaskSpec from parsed BDDL    │
│  Pick, Place, TurnKnob, Push │   │  (skills/planner.py)     │   │  (skills/task_spec.py)       │
└──────────────────────────────┘   └──────────────────────────┘   └──────────────────────────────┘
```

## Data flow per episode (eval/runner.py)
1. `LiberoEnv.reset_to(i)`: official reset + `set_init_state(init_states[i])` + 5 zero actions.
2. Once per task: `SceneToSdf` converts the compiled MuJoCo model into an SDF world (robot chain +
   STL collision meshes + one static object per collision body). `libero_planner_server` loads it.
3. `WorldSync.sync()`: pushes every body's world pose and the joint vector into the Intrinsic world.
4. The predicate planner turns the BDDL goal atoms into a skill list. Each skill:
   grasp/contact geometry from current object state -> Intrinsic IK feasibility -> Intrinsic
   PlanTrajectory (collision settings declare intentional contacts) -> `TrajectoryExecutor` converts
   the returned joint trajectory to OSC_POSE actions (per-step TCP reference from Intrinsic
   ComputeFk) -> verification from simulator evidence (finger gap, pad contacts, relative motion).
5. Episode record: run id, revisions, task/init, observation mode, controller, Intrinsic endpoint,
   every RPC id + latency + status, planned vs executed tracking summaries, skill outcomes, official
   success (`env.check_success()`), video path.

## Exact Intrinsic Core integration
| Function | Intrinsic module / target | API used | Input | Output |
|---|---|---|---|---|
| World model from SDF | `//intrinsic/world/conversion/sdf:world_from_sdf` (`intrinsic::sdf::WorldFromSdf`) | C++ (server) | generated SDF + STL | `intrinsic::World` |
| World service | `//intrinsic/world/service/test:world_service_fake` (`intrinsic::FakeWorldService`) | gRPC `intrinsic_proto.world.ObjectWorldService` | `UpdateTransformRequest`, `ReparentObjectRequest`, `UpdateObjectJointsRequest`, `GetTransformRequest`, `ListObjectsRequest` | world state |
| Motion planner service | `//intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_in_process` (`intrinsic::MotionPlannerServiceInProcess`, wraps `intrinsic::MotionPlannerService`) | gRPC `intrinsic_proto.motion_planning.v1.MotionPlannerService` | see below | see below |
| FK | same service, `ComputeFk` | `FkRequest{robot, joints, reference=root, target=frame panda/tcp}` | `Pose root_t_tcp` |
| IK | same service, `ComputeIk` (`kinematic_chain` numeric solver from `intrinsic_kinematics`) | `IkRequest{PoseEquality(tcp == root_t_target), seed, CollisionSettings}` | joint solutions (collision-checked) |
| Collision checking | `CoalCollisionChecker` (`intrinsic/world/collision`) inside IK/planning; `CheckCollisions` RPC | `CheckCollisionsRequest{waypoints}` | has_collision + debug |
| Path + trajectory planning | `intrinsic::MotionPlanner` (`intrinsic_motion_planning/.../motion_planner`) via `PlanTrajectory` | `MotionPlanningRequest{start q, MotionSegment{target PoseEquality or JointVec, motion_type ANY/LINEAR, CollisionSettings}}` | `JointTrajectoryPVA` (positions/velocities/accelerations + timestamps) |

Belongs to the adapter (ours, not Intrinsic): MuJoCo->SDF conversion, world synchronization,
grasp/place geometry, OSC action conversion, skill sequencing, verification, logging.

## Geometric approximations (documented)
- Robot links: exact robosuite collision meshes (STL export of the compiled MuJoCo meshes,
  loaded as convex by Intrinsic for collision). Finger geometry fixed at the fully-open pose
  (conservative envelope; the closed fingers lie inside it).
- Objects: LIBERO's own box decompositions (exact); the microwave's mesh parts as convex meshes.
- Floor plane skipped (Intrinsic rejects infinite planes; the robot base is above the table).
- Articulated parts (drawer, door, knob) are separate static objects whose poses are re-synced
  from MuJoCo before every plan (no joint model in Intrinsic; equivalent for collision checking).
- Grasped objects are re-parented to the `tcp` frame with the observed grasp transform.
- Joint velocity limits: Franka Panda datasheet; acceleration/jerk limits scaled down so that the
  OSC_POSE controller can track the trajectories (see model/scene_to_sdf.py).
