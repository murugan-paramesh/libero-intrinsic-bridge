# Technical report: LIBERO-10 with a classical stack on Intrinsic Core

*(Results tables and per-task numbers in this report are generated from saved episode
records; see docs/results.md and the `runs/eval/<run>` directory referenced there.)*

## 1. Goal and scope
Solve the ten LIBERO-10 tasks with a reusable, classical manipulation system whose forward
kinematics, inverse kinematics, collision checking and motion planning are computed by
Intrinsic Core, with execution in the unmodified LIBERO/MuJoCo benchmark environment through
its standard OSC_POSE controller. No learned policy, demonstration replay, or privileged
physics manipulation is used. Perception is replaced by simulator object poses
("privileged-state classical controller"), an explicitly documented assumption (Section 7).

## 2. What Intrinsic Core computes, exactly
Intrinsic Core has no Python kinematics/planning bindings and its production deployment is a
k3s cluster (Ubuntu 26.04 + release tarball), which is not available in this environment. The
smallest supported deployment that still runs the real services is an in-process gRPC server:
`intrinsic_stack/cc/libero_planner_server.cc` (129 lines of process wiring, Apache-2.0) links

| Intrinsic target (intrinsic-core @ c61bf07) | Role |
|---|---|
| `//intrinsic/world/conversion/sdf:world_from_sdf` (`intrinsic::sdf::WorldFromSdf`) | loads our generated SDF world (robot + scene) into an `intrinsic::World` |
| `//intrinsic/world/service/test:world_service_fake` (`intrinsic::FakeWorldService`, `enable_local_tcp`) | serves `intrinsic_proto.world.ObjectWorldService` (transforms, re-parenting, joints, object listing) |
| `//intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_in_process` (`intrinsic::MotionPlannerServiceInProcess` -> `intrinsic::MotionPlannerService`) | serves `intrinsic_proto.motion_planning.v1.MotionPlannerService`: `ComputeFk`, `ComputeIk`, `PlanTrajectory`, `PlanPath`, `CheckCollisions` |

Inside those services, the computation is Intrinsic's: `intrinsic_kinematics` (Skeleton/State
FK, `kinematic_chain` numeric IK), `intrinsic/world/collision/coal_collision_checker` (COAL),
`intrinsic_motion_planning/.../motion_planner` (RRT-Connect + shortcutting, linear Cartesian
path planner with path IK, acceleration-limited time parameterization in the OSS build).

Every FK/IK/plan/collision request made by the controller is one of those RPCs; each is logged
with an id, latency and status in `intrinsic_requests.jsonl` per episode, and the id of the
`PlanTrajectory` response that produced every executed motion is stored in the episode record
(`events[*].trajectory_id`). `scripts/demo_motion.py --dump-protos` stores the full request and
response textprotos (evidence/milestone1_first_intrinsic_motion).

What is *ours* (the adapter): MuJoCo->SDF conversion, world synchronization, grasp/placement
geometry, OSC action conversion, skill sequencing, verification and logging. There is no
fallback planner: an unavailable backend raises `IntrinsicUnavailableError`
(tests/test_backend_unavailable.py).

## 3. Robot model and frame agreement
LIBERO has no URDF; the robot is the robosuite Panda MJCF with the PandaGripper merged in.
`model/scene_to_sdf.py` reads the *compiled* MuJoCo model (body_pos/body_quat, joint axes,
ranges, mesh vertices) and writes an SDF kinematic model (7 revolute joints, fixed hand chain,
STL collision meshes exported from the compiled meshes), a `flange` frame at the LIBERO tool
site `gripper0_grip_site`, and one static object per collision body. Conventions handled
explicitly: wxyz vs xyzw quaternions, SDF rpy, MuJoCo half-sizes vs SDF full sizes, base pose
per scene (living room (-0.51,0,0.42), kitchen (-0.66,0,0.912), study (-0.75,0,0.912)).

Four Intrinsic-specific details were needed (all documented in code): objects loaded from SDF
have no global-alias names, so references are by id and the robot gets the alias the planner
needs; SDF `<frame>` elements need `intrinsic:create_attachment_entity="true"`; the frame pose
is resolved at joint zero (outside joint 4's limits), so `link_t_flange` is set explicitly with
`UpdateTransform` + entity filter; the planner requires exactly one frame named `flange`.

Validation (`scripts/validate_kinematics.py`, evidence/milestone0_kinematics): over 200 random
configurations inside the LIBERO joint limits, Intrinsic `ComputeFk` vs the MuJoCo site pose:
max position error 5e-9 m, max rotation error 2e-6 deg (tolerances 1e-4 m / 0.01 deg). IK: 40
random poses seeded from random configurations, 40/40 solved; setting the returned joints in
MuJoCo reproduces the pose within 1e-6 m / 1.3e-4 deg (tolerances 2e-3 m / 0.5 deg). Collision
sanity: an arm-in-table configuration is reported by Intrinsic as link6-vs-table (MuJoCo: 17
robot contacts); the home configuration is collision-free in both.

## 4. Execution through the benchmark controller
LIBERO's action is a Cartesian delta (`+-0.05 m`, `+-0.5 rad` per 20 Hz step, world-frame
rotation delta, gripper sign). `env/executor.py` samples the Intrinsic joint trajectory at the
control rate, maps every sample to a TCP pose with Intrinsic `ComputeFk`, and commands the
saturated, gain-scaled delta toward the sample 0.1 s ahead. Intrinsic trajectories respect the
Panda velocity limits and are typically 0.3-0.7 s long, far faster than the OSC controller can
follow, so execution time is scaled automatically until consecutive TCP samples are at most
1.5 cm / 6 deg apart (the path is unchanged). Measured on the first planned motion: 4.9 mm mean
/ 7.3 mm max TCP error, final 5.6 mm / 0.9 deg.

Known limitation: OSC_POSE controls the TCP only; its null-space is pulled toward the robot's
initial posture by robosuite, so the executed *joint* path deviates from the planned one
(typically 0.05-0.3 rad). Two mitigations: free-space motions are planned to the IK solution
nearest the current configuration, and every executed path is audited afterwards with
Intrinsic `CheckCollisions` on the configurations actually reached
(`executed_path_audit` events; the unintended-collision metric in the results).

## 5. Skills and task sequencing
Goal atoms come from LIBERO's parsed BDDL (`env.parsed_problem["goal_state"]`), so no task has a
hand-written script; `skills/planner.py` maps predicates to skills (In/On -> Pick + Place,
Turnon -> TurnKnob, Close -> Push) and only object-category parameters and geometric rules are
configured. Pick: generic pinch-grasp sampler on the object's own collision boxes
(point-cloud test of the pad/finger/palm zones, top-down, tilted, or side approaches), neighbour
clearance check, partial gripper pre-opening (1 cm/step), Intrinsic IK feasibility, free-space
plan to the pre-grasp, LINEAR approach with the target as the only permitted contact, close,
verification (finger gap + finger/pad contacts), attach (ReparentObject to the flange frame),
LINEAR lift, lift verification (object rose with the tcp, small relative drift). Place:
region slots, point-cloud free-spot search, container rules (hand/finger clearance search above
the rim, fit rotation, front-loading containers entered along the approach axis), contact-
monitored lowering, release, retreat along the approach axis. Articulation: knob turning by
pinching the knob and rotating about its hinge axis in LINEAR segments; drawer/door closing by
pushing along the slide axis / hinge arc with the world re-synced between segments.

Intentional contacts are declared per request through Intrinsic `CollisionSettings` rules
(fingers vs robot, grasped object vs robot/fingers/its support, resting objects vs their
support, pushed part vs robot); nothing is disabled globally.

## 6. Evaluation protocol
Declared in `configs/eval_frozen.yaml` before the final run: development init states 0-4 per
task (used while building the skills), evaluation init states 10-19 (disjoint), one episode per
state, step budget 1200 control steps (60 s), bounded per-skill recovery counted inside the
budget, no episode re-runs, official `env.check_success()` (which is what LIBERO's `done`
returns). Success within LIBERO's default 600-step horizon is reported as a secondary number.

## 7. Assumptions and limitations
- Privileged state: object poses come from the simulator; the geometry layer only needs a
  pose plus the object's collision model, so a pose estimator could replace it.
- OSC null-space drift (Section 4). A JOINT_POSITION controller would track the planned joint
  path exactly but is a modified benchmark configuration; not used for the main numbers.
- Fingers are modelled as flange-attached objects following the real joint values; the hand
  uses the robosuite collision mesh (convex in Intrinsic).
- Containers: objects are released from above the rim when the hand cannot enter (basket:
  ~5 cm drop); this is how the benchmark's own demonstrations behave but it is a source of
  failures when objects settle badly.
- The in-process service is a test-only deployment of the same code the cluster runs; caches
  and the parameterization service are not used (OSS acceleration-limited parameterizer).

## 8. Results, failure analysis, next steps
See docs/results.md (generated) and the sections appended after the frozen evaluation.

### 8.1 Frozen evaluation (100 episodes, revision 76267bf, docs/results.md)

| task | success | notes |
|---|---|---|
| 0 both soup+sauce in basket | 10/10 | all within 600 steps (mean 348) |
| 1 cream cheese + butter in basket | 5/10 | thin boxes next to tall cartons: all 5 failures are "no collision-free IK for the pre-grasp" (neighbour blocks every vertical or tilted approach); 17 grasp attempt failures, 12 recoveries |
| 2 turn on stove, moka pot on it | 10/10 | knob turn + handle grasp; tightest tracking (max 1.0 cm) |
| 3 bowl into bottom drawer, close it | 0/10 | bowl is placed in the drawer in every episode; closing fails: no collision-free IK for the push pre-contact (link 5/6 vs the wine rack next to the cabinet, or the hand vs upper drawers) |
| 4 two mugs onto two plates | 10/10 | rim grasps; 4 grasp re-attempts recovered |
| 5 book into caddy back compartment | 2/10 | 4 releases off target after the 90 deg fit rotation (book slipped/rotated in the grasp, lowering stopped on first caddy contact), 3 adapter bugs (joint 7 pushed 0.3 mrad past its limit by the controller -> UpdateObjectJoints rejected; needs clamping), 1 unreachable grasp |
| 6 mug on plate, pudding right of plate | 9/10 | 1 placement IK collision |
| 7 soup + cream cheese in basket | 8/10 | 1 approach tracking error (18 deg), 1 joint-limit violation during a pre-grasp execution |
| 8 both moka pots on stove | 8/10 | 2 goal-not-reached: second pot released 2 cm off and resting on the first (grasp drift 2.3 cm during lift) |
| 9 mug into microwave, close door | 0/10 | no collision-free horizontal grasp of the mug near table height (hand/link 5 vs table); the door-closing skill was never exercised |
| **all** | **62/100 (95% CI 52-71%)** | 0 infrastructure errors; 0 planning timeouts; 1,580 PlanTrajectory calls, mean latency 30 ms, max 110 ms; 62/62 successes within 600 steps |

Tracking: max TCP tracking error per task 1.0-5.7 cm (transients of fast transports; mean
per-motion error 2-5 mm), except task 5 (43 cm: an aborted execution). Executed-path audits
(Intrinsic CheckCollisions on the configurations actually reached): 17 of 1,017 audited
motions reported a collision, all in tasks 1/4/5/6/7 (finger or hand contact with a neighbour
after OSC null-space drift), none in tasks 0/2/3/8/9.

### 8.2 Failure analysis
- **Reachability of side/horizontal grasps (tasks 9, 3).** The Panda base sits at table height in
  the kitchen scenes; horizontal hand orientations 5 cm above the table put link 5 into the
  table or a neighbouring fixture. A lower-elbow posture would need explicit posture
  constraints in the IK request (Intrinsic supports `JointPositionLimits` constraints), which
  we did not add.
- **Cluttered thin objects (task 1).** The 20 cm-wide hand cannot descend next to a 19 cm-tall
  carton; pushing the carton aside first (a pre-manipulation skill) is the classical remedy.
- **In-hand slip during wrist rotation (task 5, 8).** Rim/handle grasps with 1-2 cm of finger gap
  slip when the wrist rotates 90 deg or the pot swings; grasp verification passed but the
  object moved. A re-grasp after rotation or a slower rotation would help.
- **Adapter bug (task 5, 3 episodes).** The controller can push joint 7 fractionally past the
  MJCF limit; the world update must clamp to limits.
- **What did not fail:** the Intrinsic backend (no timeouts, no unavailable errors), the FK/IK
  agreement, the LINEAR re-plan protocol (749 re-plans, all succeeded on the second call).

### 8.3 Next improvements
1. Posture-constrained IK (Intrinsic `JointPositionLimits` / `JointPositionSumLimit`) for
   low horizontal grasps; 2. clamp synced joints; 3. pre-manipulation (push aside) for blocked
   objects; 4. re-grasp after fit rotations; 5. a JOINT_POSITION-controller condition to
   quantify how much the OSC null-space drift costs; 6. the simple Cartesian servo baseline
   on the same init states (not run: out of time, listed as not evaluated).
