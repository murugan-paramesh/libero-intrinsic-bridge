# Walkthrough: LIBERO-10 with a classical Intrinsic Core stack

This is the reviewer's guide. Every claim below points to code, a command and a saved artifact.
Numbers are the measured protocol results recorded in `evaluations/` (see section 15).

## 1. Assignment objective
Solve the ten LIBERO-10 tasks with a classical (non-learned) manipulation system whose
kinematics, collision checking and motion planning are computed by Intrinsic Core, executed in
the unmodified LIBERO/MuJoCo environment through LIBERO's standard OSC_POSE controller, and
evaluated under a declared, reproducible protocol.

## 2. System architecture
```
LIBERO task (BDDL goal) + MuJoCo simulator state
        | task_spec.py: goal atoms -> skill sequence (planner.py)
        v
classical skills: pick / place / push (drawer, door) / knob / relocation composites
        | object-relative grasp and contact candidates (geometry.py, placement.py)
        v
Intrinsic Core (pinned c61bf07, in-process gRPC server intrinsic_stack/cc/libero_planner_server.cc)
        |-- world: one SDF model per LIBERO body (scene_to_sdf.py), poses re-synchronised every skill step
        |-- ComputeFk, ComputeIk (collision checked, JointPositionLimits posture families, seeds)
        |-- CheckCollisions (executed-path audit)
        '-- PlanTrajectory (ANY / LINEAR / JOINT, per-segment CollisionSettings, TOPP timing)
        v
returned joint trajectory -> TCP references via Intrinsic FK -> OSC_POSE deltas at 20 Hz (executor.py)
        v
measured reached state (joint error, TCP error, contacts) -> world sync -> replan / recovery
        v
official LIBERO success predicate (libero/lifelong/metric.py semantics: done at any control step)
```
Module map: `docs/architecture.md`. Every executed motion carries the id of the `PlanTrajectory`
request that produced it (`plan` and `execute` events in each `episode.json`;
`intrinsic_requests.jsonl` holds every RPC with status, latency and error text).

## 3. Why this is a classical robotics solution
No learned policy, no demonstration replay, no vision model. Decisions are made from geometry
(object boxes and point clouds from the simulator's collision model), from Intrinsic's kinematic
and collision answers, and from measured execution feedback. Object poses are read from the
simulator (privileged state) and labelled as such everywhere; perception is replaceable.

## 4. LIBERO -> Intrinsic bridge
`src/libero_intrinsic/model/scene_to_sdf.py` converts the compiled MuJoCo model (Panda chain,
fingers, every environment body with collision geometry) into SDF; the C++ server loads it with
`WorldFromSdf`, serves `ObjectWorldService` and `MotionPlannerService`. `world_sync.py` pushes
robot joints, object poses (including fixtures, whose poses LIBERO re-samples on every reset),
finger offsets and grasp attachments (`ReparentObject`) before every plan. The world is reset at
episode start (attachment leak fix, report 10.4).

## 5. Forward kinematics
`client.fk` -> `ComputeFk`. Validation against MuJoCo over random configurations:
`python scripts/validate_kinematics.py --task 0` (evidence: `evidence/milestone0_kinematics`).
The executor converts every planned joint sample to a TCP reference with Intrinsic FK.

## 6. Inverse kinematics
`client.ik` -> `ComputeIk` with `starting_joints` seeds, `max_num_solutions`, CollisionSettings,
and `JointPositionLimits` intersected with the pose target for posture families (shoulder
forward, elbow bent, task-specific). Grasp, pre-grasp, release, pre-contact, mid- and end-of-travel
poses are all IK-probed before any motion (`ik` events; rejection pairs in `push_contact_selection`).

## 7. Collision checking
Every IK and plan request is collision checked by Intrinsic with explicit, physically justified
exclusions only: fingers vs the grasped object, resting-object/support pairs (sub-millimetre
MuJoCo penetration), the pushed body during a push, the carried object vs its support while
lowering. `CheckCollisions` audits the configurations the OSC controller actually reached
(`executed_path_audit` events). Model-vs-reality checks of this pass: the drawer/cabinet point
clouds matched MuJoCo geometry exactly; fixture poses are re-sampled by LIBERO at every reset and
are synchronised per episode (section 12).

## 8. Motion planning
`client.plan_to_pose` / `plan_to_joints` -> `PlanTrajectory` with MotionType ANY (free space),
LINEAR (approach, insertion, push segments, lowering), JOINT (settle); the LINEAR re-plan protocol
(target joint configuration reached by Intrinsic's path IK) is logged as a `note`. Pushes are split
into <= 4 cm segments, each planned after a world sync so moved objects are at their true poses.

## 9. Grasp generation
`geometry.grasp_candidates` (top-down pinches on geom centres and cross sections, yaws every
22.5 deg, tilt variants) and `side_grasp_candidates` (level/pitched side grasps restricted to a
container's opening normal). Candidates pass: scene clearance (hand/finger zones vs point clouds)
-> placement-aware ranking (`PlaceSkill.grasp_compatible`: the same release search that runs at
place time, cost = drop height + slot distance) -> Intrinsic IK at pre-grasp and grasp -> LINEAR
dry-run of the approach (side grasps) -> execution with grasp verification (finger gap, pad contacts, lift).

## 10. Trajectory execution
`env/executor.py`: TCP references from Intrinsic FK, OSC_POSE position/orientation deltas,
velocity-scaled timing, tracking-error abort (8 cm), final-pose check, planned-vs-reached joint
error recorded for every motion (`execute` events: `pos_err_max`, `joint_err_final`, ...).

## 11. OSC_POSE limitation
The benchmark controller tracks the TCP pose only; its null space drifts, so the joint
configuration reached can differ from the planned one by 0.3-2.3 rad while the TCP is within
1 cm. Consequences handled explicitly: pre-grasp/pre-contact configurations are chosen among the
IK solutions nearest to the current configuration (same branch), every next segment is planned
from the measured configuration, LINEAR failures from a drifted branch fall back to a
collision-checked ANY plan to the same pose (recorded), start-state collisions trigger a bounded
retreat. Documented limit: report 11.2.

## 12. Task 3 strategy and outcome (unsolved)
Backward chain: closed drawer <- flat bowl deep in the drawer <- release/retreat <- grasp <-
push posture. Measured first divergence (`runs/lab/t3_*`, stage lab): the bowl was released
3-4 cm above a spot whose footprint overlapped the drawer's inner front wall and landed tilted
30 deg. Every deeper spot is rejected because a level top-down hand over a flat bowl intersects
the handles of the drawers above (handle points at y >= 0.197, z 1.007-1.097 vs a 10.4 cm tall
hand); release tilts of 10-20 deg move the hand clear but then the bowl's rim touches the middle
handle; the hand's wide axis must lie across the drawer (yaw-90 grasps), which the ranking now
covers. Pushing: a top-down push cannot finish the travel (hand above the panel hits the upper
handles); a horizontal push has no collision-free contact pose on the tested states (forearm vs the
wine rack fixture, wrist vs table, no IK), with or without relocating the wine bottle, in three
posture families; a side-push family (hand pointing along the reach direction) was blocked by the
relocated bottle in the arm corridor; a steeply pitched family (45-70 deg, 19 candidates, third
pass) is rejected at the end of the travel because the closed panel face lies in the handle plane
of the drawers above (`evidence/dev_task3_lab_pitched`). No feasible solution was found within
the tested classical search; the search space, rejection reasons and geometry are in
`docs/failure_table.md` sections 10 and 12 and `docs/report.md` sections 12.2 and 13.1.

## 13. Task 9 strategy and outcome
Backward chain: door closed <- hand withdrawn <- mug released on the cavity floor inside the
heating region <- level insertion along the opening normal <- level side grasp 4.5 cm above the
mug bottom <- the porcelain mug relocated out of the approach. Measured first divergence of the
previous candidates: the slip-compensation step mistook the 8 cm pre-place offset of a side
approach for grasp slip and drove the hand 8 cm deeper into the microwave (fixed); then the
door push: the pre-contact configuration 2.3 rad from the current one (fixed by same-branch
ordering), LINEAR segments failing near the wrist limit (configuration-space fallback for later
segments), the door rebounding after the retreat (the goal is re-checked after the retreat and
the push repeated). Under 5e236de: complete official successes on development states 1, 3, 4;
states 0 and 2 fail at the pick after relocation (pre-grasp posture at the joint-2 limit).
Candidate C4 (fbabe38, third pass) adds a physically executed staging regrasp before the side
pick (top-down pick, place on the opening-normal line 0.60-0.70 m from the base, side pick from
there), start-state recovery before the door pre-contact plan and a configuration-space fallback
for the first push segment after a completed approach: development states 0-4 all succeed
(`evidence/dev_task9_success_c4`). Protocol results of both: section 15.

## 14. Evaluation methodology
`configs/eval_frozen.yaml`: tasks 0-9 in order, official init states 10-19 (development states
0-4, held-out 20-29 used once for 5b713c6, held-out 30-39 used once for fbabe38), seed 0, budget 1,200 control steps, bounded per-skill
retries, no episode re-runs, privileged object poses. Success = LIBERO's own metric semantics
(`libero/lifelong/metric.py`: an episode counts when `done` is reported at any step; the runner
terminates at that step and also records `success_at_end` and the goal atoms). Candidates before
C3 evaluated the predicate only at skill boundaries and at the end (stricter); their numbers are
unchanged in the tables. Totals are generated from episode records (`eval/report.py`).

## 15. Final results
See `README.md` (headline table) and `docs/results_*.md`; the per-task before/after tables are
`docs/comparison_*.md`. Recommended: fbabe38 (C4, 83/100; task 9 7/10; one-time held-out states 30-39: 85/100);
fallbacks 5e236de (80) and ce7685b (76). The reasons for the choice are stated in `docs/report.md` sections 12.6 and 13.5.

## 16. Limitations
Privileged object poses; OSC_POSE null-space drift; task 3 unsolved; task 9 partially solved
(7/10: one release IK rejection, one door segment, one approach error after the staging); planner randomness makes individual episodes vary between runs
(documented flips on tasks 6 and 7); runtime of task 3 episodes (IK probe matrix with 5 s solver
timeouts).

## 17. Exact reproduction
```bash
# setup: README "Setup (pinned)" (Intrinsic Core c61bf07, LIBERO 8f1084e, Bazel 8.8.1, Python 3.10)
python -m pytest -q
python scripts/validate_kinematics.py --task 0          # FK/IK agreement
python scripts/demo_motion.py --task 0 --init 0         # one Intrinsic-planned motion
python scripts/run_task.py --task 9 --inits 3            # task 9 complete episode (dev state)
python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/review/A & python scripts/evaluate.py --tasks 5 6 7 8 9 --out runs/review/B; wait
python -m libero_intrinsic.eval.report runs/review --out runs/review/results.json
python scripts/compare_evaluations.py evaluations/candidate_ce7685b runs/review --labels ce7685b rerun
python scripts/stage_lab.py --task 3 --init 0 --until place --execute 4 --out runs/lab/t3   # task 3 diagnosis (dev only)
```
