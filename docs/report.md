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
  agreement, the LINEAR re-plan protocol (547 re-plans; 16 genuine plan failures in 1,580 calls).

### 8.3 Next improvements
1. Posture-constrained IK (Intrinsic `JointPositionLimits` / `JointPositionSumLimit`) for
   low horizontal grasps; 2. clamp synced joints; 3. pre-manipulation (push aside) for blocked
   objects; 4. re-grasp after fit rotations; 5. a JOINT_POSITION-controller condition to
   quantify how much the OSC null-space drift costs; 6. the simple Cartesian servo baseline
   on the same init states (not run: out of time, listed as not evaluated).

## 9. Improvement loop after the baseline (candidate revision 5b713c6)

The baseline (62/100, revision 76267bf, archived under `evaluations/baseline_76267bf`) was left
unchanged. The loop was: read the baseline episode records and RPC payloads -> hypothesis ->
smallest classical change -> dev runs on the development init states (0-4, disjoint from the
evaluation states 10-19) -> regression runs on the tasks sharing the component -> keep only
with evidence. Every row of `docs/failure_table.md` section 2 carries the observed evidence
(record events, RPC error payloads, video frames) separately from the hypothesis.

### 9.1 Root causes found and the changes kept
1. **Transport collision rule (task 5, affects every container placement).** The baseline
   excluded the carried object from collision checking against its *future* support for the
   whole transport (needed only for the final contact). Intrinsic's planner therefore swept the
   book through the caddy wall and the book was knocked out of the fingers; the baseline
   records had classified this as "grasp slip" (13-26 cm offsets). Fix: the exclusion is
   applied only to the contact-monitored lowering (and to retries that start inside the
   container); the transport is planned with the carried object fully checked. The "pendulum
   slip / slow transport" hypothesis was tested (0/3) and dropped.
2. **Release-pose search** (`PlaceSkill._release_search`, pure geometry before any RPC):
   hand/finger clearance, carried-object clearance against non-support bodies (8 mm) and
   container walls (5 mm), the object above the rim at the pre-place pose (3 cm, Intrinsic's
   exact check rejected 1 cm), footprint centring (the book's origin is 8 mm off its footprint
   centre). The same search ranks grasp candidates in `PickSkill` (placement-aware grasp
   selection: feasible release first, smallest drop height next). For the bowl/drawer task this
   is what makes the hand stay in front of the cabinet.
3. **Fit rotations by principal axis.** A diagonally held object is yawed by the angle that
   aligns its footprint's PCA axis with the region box; all equally fitting angles are probed
   with Intrinsic IK and the one with the largest joint-limit margin is used.
4. **Region-box ledge rule (task 5).** LIBERO's `In` tests the object *origin* against the
   region box. The book's origin is its bottom face and the back-compartment box starts
   1.2 cm above the caddy floor, so a book standing on the floor can never satisfy the goal
   (verified from the pinned LIBERO source and the MJCF; clean insertions failed 0/2). The
   valid resting states are on the container's internal structure. The rule finds the lowest
   horizontal ledge (top face of a collision box of the support) inside the box whose long
   side follows the region's long axis (the 6.7 cm divider), places the object's bottom on it
   with a 4 mm outward offset, lowers under the contact monitor and releases; the object tips
   onto the outer wall. Dev 5/5. This is a benchmark-specific but legitimate consequence of
   the predicate; it is reported as such.
5. **Drawer push geometry and segmentation (task 3).** IK probe tables (Intrinsic ComputeIk,
   scratch scripts) showed that only a closing axis *along* the push with a tilt <= 20 deg has
   collision-free solutions at the contact pose. Long pushes are split into <= 4 cm segments,
   each planned after a world sync so that objects carried by the drawer (the bowl) are at
   their true poses. The bowl placement, which blocked every baseline episode of this task
   after the push, now succeeds (2/2 dev); the push closes 13 of the 16 cm.
6. **Pick robustness (task 1 regression introduced by the previous session's exact
   support-plane test).** The conservative gripper zones carry 3-5 mm margins, so 4 mm of zone
   below the plane is tolerated and dipping candidates are lifted while the pads still overlap
   the object; a candidate whose grasp IK is in collision no longer aborts the attempt.

### 9.2 Intrinsic involvement in the new pieces
All kinematics, collision checking and planning remain Intrinsic RPCs: the release search and
grasp ranking are geometric pre-filters whose chosen poses are then solved by `ComputeIk`
(nearest-IK joint targets) and planned by `PlanTrajectory` under the per-phase
`CollisionSettings` (the only change to those settings is *fewer* exclusions during transport).
Push segments are LINEAR `PlanTrajectory` requests after `UpdateObjectJoints`/`UpdateTransform`
world syncs; the probe tables are `ComputeIk` calls with the real collision rules. No motion
is generated outside Intrinsic.

### 9.3 Development results (before -> after, dev init states; records in runs/dev2)
| task | 0b28d05 (post-baseline, previous session) | 5b713c6 | shared component |
|---|---|---|---|
| 0 | - | 2/2 | regression check (Pick/Place) |
| 1 | 0/2 (76267bf: 1/2) | 3/3 | pick plane test + candidate loop |
| 2 | - | 2/2 | regression check (knob + Place) |
| 3 | 0/2 (place blocked) | 0/2 (place OK, push 13/16 cm) | release search, grasp ranking, push |
| 4 | - | 2/2 | regression check |
| 5 | 1/3 | 5/5 | transport rule, fit rotations, centring, ledge rule |
| 6 | - | 2/2 | regression check |
| 7 | - | 2/2 | regression check |
| 8 | - | 2/2 | regression check (slip compensation) |
| 9 | - | not run (blocked, see 8.2) | - |

### 9.5 Regressions and blockers
- Task 3: the last ~3 cm of the drawer travel are unreachable for every probed hand pose
  (the closed drawer front is flush with the upper drawer fronts; the wrist/forearm or the
  hand collides with them, and horizontal pushes put link 5 into the wine rack). A different
  strategy is needed (e.g. a posture-constrained IK with the elbow out, or pushing the drawer
  front with the side of a horizontally held hand from the right where the rack is not).
- Task 9: unchanged blocker (open microwave door between the robot and the mug).
- The region-box ledge rule depends on the container's internal geometry being available as
  collision boxes (true for LIBERO's caddy). Where no ledge exists the rule is a no-op.

### 9.4 Repeated evaluation of the candidate (100 episodes, revision 5b713c6)
Same protocol as 8.1 (`configs/eval_frozen.yaml`: tasks 0-9, official init states 10-19, seed 0,
budget 1,200 steps, same retry accounting and predicates), run after the inspection above, so it
is a *repeated* evaluation of the same states, not a held-out one. Records are archived under
`evaluations/candidate_5b713c6` (generated table `docs/results_5b713c6.md`); the per-episode
before/after table is `docs/comparison_5b713c6.md`.

| task | baseline 76267bf | candidate 5b713c6 | delta | newly solved (init) | newly failed (init) |
|---|---|---|---|---|---|
| 0 soup + sauce in basket | 10/10 | 10/10 | 0 | - | - |
| 1 cream cheese + butter in basket | 5/10 | 9/10 | +4 | 10, 11, 12, 16 | - |
| 2 stove knob + moka pot | 10/10 | 10/10 | 0 | - | - |
| 3 bowl in drawer + close | 0/10 | 0/10 | 0 | - | - |
| 4 two mugs on plates | 10/10 | 10/10 | 0 | - | - |
| 5 book in caddy | 2/10 | 8/10 | +6 | 10, 11, 12, 13, 15, 16 | - |
| 6 mug on plate + pudding | 9/10 | 9/10 | 0 | 16 | 15 |
| 7 soup + cream cheese in basket | 8/10 | 9/10 | +1 | 11, 16 | 18 |
| 8 two moka pots on stove | 8/10 | 8/10 | 0 | - | - |
| 9 mug in microwave + close | 0/10 | 0/10 | 0 | - | - |
| **all** | **62/100** (CI 52-71%) | **73/100** (CI 64-81%) | **+11** | 13 | 2 |

Aggregates (candidate): 1,204 `PlanTrajectory` calls, mean latency 24 ms, max 121 ms (task 3's
IK-limited push segments reach 5.0 s in `ComputeIk`), 0 timeouts, 0 infrastructure errors;
executed-path audits: 11 of 1,204 in collision (tasks 1, 5, 7, 9; finger/hand contact with a
neighbour after OSC null-space drift); max TCP tracking error per task 1.0-7.1 cm (task 5:
43 cm, one aborted execution); 73/73 successes within 600 steps.

Failure stages of the 27 candidate failures (from the records): task 3: 7x ARTIC/REACH (push
segment IK: wrist/forearm vs cabinet fronts), 3x PLAN (lowering LINEAR path through the cabinet
with an extended placement candidate, "a point along the planned trajectory is invalid");
task 9: 9x REACH (no reachable side grasp), 1x IK; task 1: 1x REACH (butter, no reachable grasp);
task 5: 1x REACH (no reachable grasp), 1x PLACE (book slid off the ledge and stands on the
floor, origin outside the box); task 6: 1x PLACE (pudding lost during the lowering, ended
0.67 m away); task 7: 1x PLAN (second pick: LINEAR approach start configuration in collision);
task 8: 2x GRASP/PLACE (second pot slipped 1.5-2.3 cm in the grasp and was released 7.6 cm off).

Two episodes that the baseline solved failed with the candidate (6/15, 7/18); both are
single-episode effects of changed grasp/placement choices on the same states, not of a changed
rule, and are listed as regressions. The net change is +11 with 13 newly solved episodes.

### 9.6 Held-out check (reserved init states 20-29, candidate 5b713c6)
A separate run on official init states 20-29, never used for development (0-4) or for the
frozen evaluation (10-19), with the same protocol otherwise (budget 1,200, seed 0). Records:
`evaluations/heldout_5b713c6`, table `docs/heldout_5b713c6.md`. It is reported separately and
is not merged with the 100-episode protocol numbers above.

| task | held-out 20-29 | protocol 10-19 (candidate) | failure stages on 20-29 |
|---|---|---|---|
| 0 | 10/10 | 10/10 | - |
| 1 | **1/10** | 9/10 | 9x REACH/PLAN: no reachable grasp for the butter (8) or the cream cheese (1); the IK collision pairs are finger/hand vs the milk carton (95 of 110 rejected solutions), fingers vs basket (12) - the cluttered-thin-object case of 8.2, much more frequent on these states |
| 2 | 10/10 | 10/10 | - |
| 3 | 0/10 | 0/10 | 9x push segments (IK/plan), 1x lowering path |
| 4 | 10/10 | 10/10 | - |
| 5 | 8/10 | 8/10 | 1x goal not reached (book not resting in the box), 1x no grasp candidate |
| 6 | 8/10 | 9/10 | 1x goal not reached, 1x pick plan (IK collision) |
| 7 | 9/10 | 9/10 | 1x pick LINEAR plan |
| 8 | 9/10 | 8/10 | 1x goal not reached (second pot) |
| 9 | 0/10 | 0/10 | 8x no reachable side grasp, 2x place IK/plan |
| **all** | **65/100** (CI 55-74%) | **73/100** (CI 64-81%) | 1,152 plans, mean 25 ms, max 114 ms, 0 timeouts; 13/1,152 executed-path audits in collision; 65/65 within 600 steps |

Reading: the gains on tasks 5, 7 and 8 and the unchanged tasks 0, 2, 4 transfer to unseen
states; the task 1 gain does not (1/10 vs 9/10): on states 20-29 the butter lies against the
milk carton in nine of ten states and no top-down or tilted pinch is collision-free, which the
baseline also could not do (the baseline was not run on 20-29, so no before/after is claimed
for this set). The overall held-out estimate (65/100) is inside the protocol CI; the two
sets differ mainly through task 1.

### 9.7 Follow-up after the evaluation (commit 25249ae, dev-verified only)
The task 8 failures (second moka pot slipping 1.5-2.3 cm in the handle grasp during the lift,
release raised 5 cm because the search used the attachment captured at attach time) led to one
more change after the evaluation: the attachment is re-measured at the start of every place
(and pushed to the Intrinsic world). Dev: task 8 inits 0-3 4/4, tasks 0 and 5 inits 0-1 2/2
each. The 100-episode protocol was not re-run for it, so the evaluated candidate remains
5b713c6 and this change is reported as unevaluated.

## 10. Second improvement loop: method families, Tasks 3 and 9, adapter integrity (candidate ce7685b)

The method catalogue, what the pinned Intrinsic release verifiably exposes for each family, and
the method-selection table (failure | candidate method | Intrinsic support | experiment |
outcome | retain/reject) are in `docs/methods.md`. This section summarises what changed.

### 10.1 Verified Intrinsic capabilities used in this loop (category 1)
`JointPositionLimits` IK constraints (posture families), IK seed diversity with
`max_num_solutions`, `ensure_same_branch` (continuity between the pre-contact solution and the
contact pose), per-pair collision exclusion rules for bounded recoveries, and `ReparentObject`
for the episode-start reset. Not exposed and therefore not used: planner algorithm selection,
force/impedance control (no force sensing in LIBERO's OSC_POSE setup).

### 10.2 Task 3 (drawer): what was established
1. The bowl placement works (release search with path validation, release separation when the
   bowl's rim is hooked on a finger after opening: dev 2/2).
2. Stage A (top-down push) closes ~10 of 16 cm (partial progress is a success; an IK probe
   with 4 seeds and elbow-up limits confirms no collision-free top-down pose exists at the end
   of travel).
3. Stage B (horizontal hand) has collision-free end-of-travel IK solutions when the bowl is
   not against the front panel (6-8 solutions, probes), and the run-time contact selection
   (7 lateral offsets x 5 heights x 4 seeds x 2 posture families, each pose checked by Intrinsic
   IK) names the blocker when it fails: on init 1 the wine bottle (forearm), after relocating
   the bottle the bowl, which slides against the panel while stage A pushes.
4. Obstacle relocation (task-level replanning by real manipulation: pick the named movable
   non-goal blocker, place it on a free table spot, retry) executed correctly on init 1
   (165 steps) but did not unblock the push because of the bowl.
5. Open: the 7.4 cm hand does not fit the 6.9 cm panel height band once the bowl rests
   against the panel. Candidates not tried: pushing with the finger tips only at a lower height
   with a tilted-up hand (geometry says the hand extends further), or holding the bowl back
   while pushing (two contacts), which the single gripper cannot do.

### 10.3 Task 9 (microwave): what was established
1. Pick: the level side hand collides with the neighbouring porcelain mug (wrist) at every
   probed spot; oblique (20/35 deg pitched) side grasps and low grasp heights were added and
   the pick now succeeds on both dev states, including a slip detected by lift verification
   with a successful second candidate.
2. The region's opening normal is derived from the owner geometry (`region_opening_normal`)
   and side grasps are restricted to approaches within 25 deg of it (the earlier runs grasped
   along +x and tried to enter the opening sideways).
3. Insertion: the cavity is 20.7 cm wide and 15 cm high; the 21.8 cm hand cannot enter and a
   pitched hand holding the mug 5-8 cm above its bottom rises above the roof. The release
   search now rejects such poses (and the place skill verifies the origin against the region
   box instead of reporting a success with the mug on the roof). No candidate among the
   cleared grasps is placement-compatible on the dev states; a level grasp 2-4 cm above the
   mug's bottom would fit by geometry but was not among the cleared candidates (the hand
   would dip below the table plane at the pick).
4. Door closing: the arc push works once reached (dev init 0, segment 4 satisfied).
5. Open: a level, low side grasp with the fingertips just above the table (needs the pick
   plane tolerance for side approaches), then the insertion with the hand outside the opening.

### 10.4 Adapter integrity fix (affects the evaluated candidate)
The Intrinsic world persists across the episodes of a task session. An object left attached
to the flange by a failed place in episode N kept moving with the robot in episode N+1
(observed: task 9 init 1, pre-grasp IK rejected with "mug vs microwave"). Every movable body
is now re-parented to the world root at episode start. Candidate 5b713c6 was evaluated with
this defect; episodes that followed a failed place in the same session (tasks 3 and 9) may
have been affected. The protocol was therefore re-run on ce7685b (10.5).

### 10.5 Repeated evaluation of candidate ce7685b (100 episodes, same protocol)
Records `evaluations/candidate_ce7685b`, table `docs/results_ce7685b.md`, comparisons
`docs/comparison_ce7685b.md` (vs the baseline) and `docs/comparison_5b713c6_to_ce7685b.md`
(vs the first candidate). Same init states 10-19, so again a repeated evaluation.

| task | baseline 76267bf | candidate 5b713c6 | candidate ce7685b | changed episodes (5b713c6 -> ce7685b) |
|---|---|---|---|---|
| 0 | 10 | 10 | 10 | - |
| 1 | 5 | 9 | 10 | +18 |
| 2 | 10 | 10 | 10 | - |
| 3 | 0 | 0 | 0 | - |
| 4 | 10 | 10 | 10 | - |
| 5 | 2 | 8 | 8 | +17, -12 |
| 6 | 9 | 9 | 9 | +15, -18 |
| 7 | 8 | 9 | 9 | +18, -11 |
| 8 | 8 | 8 | 10 | +11, +15 |
| 9 | 0 | 0 | 0 | - |
| **all** | **62** (CI 52-71%) | **73** (CI 64-81%) | **76** (CI 67-83%) | 6 newly solved, 3 newly failed |

Aggregates: 1,321 `PlanTrajectory` calls, mean 22 ms, max 96 ms, 0 timeouts, 0 infrastructure
errors; 21 of 1,321 executed-path audits in collision; 76/76 successes within 600 steps.
Failure stages of the 24 failures: task 3 10x ARTIC (3x stage A LINEAR approach, 7x stage B
contact selection after the bottle relocation: hand vs the bowl against the panel or no
solution); task 9 10x PLACE (no release pose: hand vs the cavity/roof); task 5 1x goal not
reached, 1x no reachable grasp; task 6 1x goal not reached; task 7 1x pre-grasp tracking.
Cost: task 3 episodes now take 13 min wall time on average (max 26 min; the run-time contact selection issues up
to 70 x 4 poses x 4 seeds IK requests, each unreachable pose costing the solver's 5 s
timeout); the simulated-step budget is unaffected (307-534 steps). Every other task is
unchanged in cost.

### 10.6 What was retained, rejected, and what remains open
Retained (dev- and protocol-verified): placement-aware grasp ranking, release search with
path validation and roof cap, release separation, start-state recovery, episode-start
attachment reset, attachment refresh at place start (task 8: 8 -> 10), pitched/low side grasps
with the opening normal (task 9 pick now succeeds), IK seed clamping, posture-margin and
same-branch pre-contact selection, the two-stage drawer push with obstacle relocation (correct
behaviour, no success yet). Rejected with evidence: pendulum/slow transport (task 5), top-down
end-of-travel pushes under posture constraints (task 3), staging the mug elsewhere (task 9,
no feasible spot), footprint centring in wide cavities. Open: the task 3 end-of-travel push
(hand vs the bowl against the panel; the 7.4 cm hand does not fit the 6.9 cm panel band), the
task 9 insertion (a level grasp 2-4 cm above the mug bottom and the pick plane tolerance for
side approaches are the next concrete experiment), and task 1 on the held-out states (butter
against the milk carton: pre-manipulation of the carton would be the next classical step).
