# Classical method catalogue and selection table

Scope: classical robotics methods considered for the remaining failures (see
`docs/failure_table.md`), what the pinned Intrinsic Core release
(c61bf075f2335371c6367b61117e8a62bb960c3b) verifiably exposes for each, and what was tried.
Categories: **(1)** available through a verified Intrinsic API, **(2)** custom classical
orchestration around genuine Intrinsic computation, **(3)** unsupported or unsuitable here.
Nothing in category 2 is attributed to Intrinsic; nothing in category 3 replaces Intrinsic planning.

## 1. What the pinned release exposes (verified from the proto/source files)

| capability | where verified | status |
|---|---|---|
| Motion types ANY (configuration-space, collision-free; RRT-Connect in `path_planning/`), LINEAR (Cartesian straight line), JOINT (joint interpolation) | `motion_planning/proto/v1/motion_specification.proto` `MotionSegment.MotionType` | (1) used: ANY for free-space, LINEAR for approach/lower/push |
| Per-segment `CollisionSettings` (exclusion rules per object pair, minimum margin, disable) | same proto, `collision_settings = 16` | (1) used for every phase |
| Per-segment `JointLimitsUpdate` (position/velocity/acceleration/jerk) and `DynamicCartesianLimits` | same proto, fields 17/18 | (1) available; position-limit updates = posture restriction for a whole segment (not yet used in the pipeline, probed below) |
| Uniform path constraints along a segment: `RotationCone`, `PositionBoundingBox`, `JointPositionSumLimit`, `PointAt` | `geometric_constraints.proto` `UniformGeometricConstraint` | (1) available (e.g. keep a carried object upright with a RotationCone); not yet used |
| IK targets: `PoseEquality`, `PositionEquality`, `RotationEquality`, `RotationCone`, `PositionBoundingBox`, `PointAt`, `JointPositionLimits`, `JointPositionSumLimit`, `ConstraintIntersection` | `kinematics/utils/compute_ik_util.cc` `ResolveJointPositionLimitsConstraints` switch | (1) used: PoseEquality (+JointPositionLimits probe). Position-only / cone targets give the solver orientation freedom (candidate for pushes) |
| IK seed (`starting_joints`), `max_num_solutions`, `ensure_same_branch`, `prefer_same_branch`, `disable_error_on_collisions`, `ik_debug_information` (per-solution rejection reasons incl. collision pairs) | `motion_planner_service.proto` `IkRequest/IkResponse` | (1) seed and max solutions used; debug info parsed from the error text; same-branch flags unused |
| `MotionPlannerConfiguration`: timeout, `path_planning_step_size`, `shortcutting_combine_collinear_segments`, collision checker config, `collision_check_spacing_override`, strict trajectory fallback | `motion_planner_config.proto` | (1) timeout used; step size / check spacing available (continuous vs discrete checking resolution) |
| Blending between segments (`BlendingParameters`) | `motion_specification.proto` | (1) available; single-segment requests used (every segment is executed and re-synced) |
| Time parameterization with velocity/acceleration/jerk limits (TOPP) | `trajectory_planning/topp/` | (1) every `PlanTrajectory` result is time-parameterized by Intrinsic; our executor only rescales time for the OSC controller |
| Swept volumes of a planned motion (`compute_swept_volume`) | `MotionPlanningRequest` field 5 | (1) available, unused |
| `CheckCollisions` on explicit joint waypoints | service proto | (1) used for executed-path audits |
| Force/impedance/admittance control | no force sensor in the LIBERO robot, OSC_POSE position controller only | (3) not available; contact strategy is position-based with contact monitoring from simulator contacts (privileged) |
| Planner algorithm selection (RRT vs PRM vs optimization) | not exposed in the request protos | (3) not selectable; ANY = Intrinsic's configuration-space planner |
| Task planning (symbolic preconditions/effects, sequencing search) | none in Intrinsic Core | (2) ours: predicate planner (`skills/planner.py`), bounded recovery per skill |
| Grasp candidate generation / ranking, placement search, ledge rule | none in Intrinsic Core | (2) ours (geometry), validated by Intrinsic IK/planning |

## 2. Method-selection table

Failure | Candidate method | Intrinsic support | Experiment | Outcome | Retain/reject
---|---|---|---|---|---
T3 drawer last 3 cm (wrist/forearm vs upper drawer fronts) | Posture diversity: 4 seeds + elbow-up `JointPositionLimits` for the top-down end pose | (1) IK seed + joint-limit constraint | scratch `probe_posture.py 3 0`: tilt -40/-20/0, yaw 0/90, 5 seeds, elbow-up limits | every top-down end pose: only colliding solutions (link5/6 vs cabinet top, hand vs middle drawer) or no solution under the limits | reject (top-down family at end of travel) |
T3 same | Different contact strategy: horizontal hand (tcp z = push axis, wide axis horizontal) at the panel | (2) orchestration; IK (1) | `probe_posture.py 3 0` horizontal, lateral offsets 0/-6/+6 cm; `probe_win2.py` with the bowl excluded (it rides in the drawer) | end-of-travel pose: 6-8 collision-free solutions at x = 0 and -5 cm; q3 (3/4 travel) 7 sol; shoulder-forward limits 8 sol; contact/early poses blocked by the wine rack (link5/6) | retain as **stage B** of a two-stage push (stage A top-down with partial progress, stage B horizontal) |
T3 same | Run-time contact-point selection by IK feasibility of pre/contact/mid/end poses, with seed diversity (current, home, two elbow postures) and posture families (`JointPositionLimits`: shoulder forward) | (1) ComputeIk, max_solutions 8 | `planner.drawer_close_geometry` (event `push_contact_selection`); dev inits 0-1 | with 4 seeds x 2 posture families x 7 lateral offsets x 5 heights, every end-of-travel pose is rejected: init 1 "link5 vs wine bottle" (all), after relocating the bottle "gripper vs bowl" (the bowl slides against the front panel during stage A; the 7.4 cm hand cannot stay between the table and the 6.9 cm panel) or no solution | retain the mechanism (it is what diagnoses the blocker), task still unsolved |
T3 wine bottle blocks the forearm (stage B) | **Task-level replanning: obstacle relocation** (pick the movable non-goal blocker named by the IK probes, place it on a free table spot >= 25 cm from the push corridor, retry stage B) | (2) composite skill over Pick/Place (Intrinsic IK/planning for every motion) | `planner.DrawerCloseComposite` (events `obstacle_relocation`, `relocation_spot`); dev init 1 | executed by real manipulation (bottle moved to (-0.27, 0.18), 165 steps); the bottle collision disappears from the probes, the remaining blocker is the bowl against the panel | retain (correct, bounded, budgeted); insufficient alone |
T3 stage A LINEAR approach "FinePathIK excessive change" (init 0) | Continuity: pre-contact IK solution pre-validated with `ensure_same_branch` at the contact pose, largest joint-limit margin among branch-consistent solutions | (1) IkRequest.ensure_same_branch | `PushSkill` (event `precontact_branch_check`) | 0 of 8 solutions branch-consistent on init 0 at every attempt: the top-down approach pose itself has no branch-consistent descent here; falls back to the margin-sorted solution and the LINEAR segment still fails | keep (no cost, informative); init 0 unsolved |
T3 bowl hooked on a finger after release | Observation-based release separation (finger contacts with the released object -> move 2.5 cm away horizontally before withdrawing) | (2) + LINEAR plan | `PlaceSkill._retreat` (event `release_separation`); init 0 | contacts 2 -> 1 after two moves; the bowl no longer leaves the drawer with the retreat | retain |
T3 any | Posture-constrained IK for the top-down end pose (elbow-up / shoulder-forward) | (1) | `probe_posture.py 3`, `probe_win2.py` | top-down: only collisions or no solution; horizontal with shoulder-forward limits: 8 solutions at the end pose when the bowl is excluded | reject for top-down; informs stage B |
T3 bowl placement path | Validate the straight descent between pre-place and release (5 samples) against hand/object clearance, not only the endpoints | (2) geometry; the plan itself is Intrinsic LINEAR | `_release_search.valid()`; failure seen in eval (3/10, carried bowl vs middle drawer along the lowering path) | pending re-evaluation | retain (no cost; removes a known failure mode) |
T3/T9 plan rejected at start ("Invalid initial joint configuration", finger touching the just-released object) | Bounded recovery: retreat 5 cm up with only the offending pair excluded, re-sync, re-plan once | (1) collision rules + LINEAR plan | `_recover_start_state` (event `start_state_recovery`) | pending | retain if no regression |
T9 side grasp blocked (wrist vs neighbouring mug, forearm vs open door) | Grasp geometry: oblique (pitched-down 20/35 deg) side approaches | (2) candidate generation; IK (1) | `probe_t9b.py`: level side hand -> wrist vs porcelain mug at every spot; 30 deg pitch at the microwave mouth: 8 solutions | dev init 0: pick succeeds (pitch 35), init 1: second candidate succeeds after a slip detected by lift verification | retain |
T9 staging the mug elsewhere first (TAMP re-sequencing) | Pick-place to a staging spot, then side grasp | (2) | same probe: staging spots at x >= 0.08 have no IK solution for a +y horizontal hand; spots at x <= 0 collide with the porcelain mug (wrist) | no feasible staging spot found in the probed set | reject for now |
T9 mug released on the microwave roof | Rim rule restricted to top-down approaches; release raised only while the object stays under the region box top; place verification against the region box (origin inside the box, else the skill fails) | (2) | dev inits 0-1 | the skill now reports the failure instead of a false success; the mug is no longer dropped on the roof | retain |
T9 insertion: every release candidate rejected ("hand vs microwave") | Geometry diagnosis (scratch `t9hand.py`): the cavity is 20.7 cm wide (x -0.153..0.054) and 15 cm high (0.944..1.088) with a 8.5 cm control bezel right of the opening; the 21.8 cm hand cannot enter, a hand pitched 20-35 deg rises above the roof when the mug is held 5-8 cm above its bottom; the chosen grasps approached along +x (65 deg) although the opening faces +y | Opening-normal side grasps (`region_opening_normal`, yaws within +-25 deg of it), low grasp heights (2.5/4.5 cm above the bottom), placement-aware ranking with the reason logged | (2) geometry; IK (1) | dev inits 0-1 | pick succeeds on both states; 0 of 13-16 candidates placement-compatible (all pitched candidates: hand above the roof); level low candidates were not among the cleared set | retain the diagnosis; task unsolved |
Any task, episode N+1 after a failed place | **Adapter integrity**: an object left attached to the flange in the Intrinsic world by a failed place in the previous episode of the same task session kept moving with the robot (task 9 init 1: pre-grasp IK rejected with "mug vs microwave") | (1) ReparentObject | `WorldSync.__init__` re-parents every movable body to the world root at episode start | fixed on dev; affects the evaluated candidate 5b713c6 wherever an episode followed a failed place (tasks 3, 9) | retain; re-evaluate |
T9 door closing arc | Arc path with per-segment orientations (hinge axis from the model) | (2) over LINEAR plans (1) | `probe_door.py 0`: pre and first 3 arc poses 8 solutions with every seed; later poses collide only with the mug left on the roof | door closed in dev init 0 (segment 4 satisfied) once reached | retain |
T5 book not resting in the box | Region-box ledge rule (place on an internal horizontal ledge) | (2) | dev 5/5, eval 8/10 | retained earlier (report 9.1) | retain |
T8 second pot slip | Re-measure the attachment at place start | (2) | dev 4/4 | retained (report 9.7) | retain |

Session 5 additions:

Failure | Candidate method | Intrinsic support | Experiment | Outcome | Retain/reject
---|---|---|---|---|---
T9 level grasps blocked at the pick by the porcelain mug (hand body overlap: 14-17 candidates) | Assess-first obstacle relocation (sequence change): relocate the movable non-goal blocker by pick/place before the goal pick | (2) composite; IK/planning (1) | `PickWithRelocation`, dev inits 0-1 (`runs/dev2/task9_reloc*`) | relocation executed (xy error 4 mm), 8 level grasps become placement-compatible, grasp reached and verified (6-7 pad contacts) | retain |
T9 release pose rejected by the exact check with a 25 deg off-normal grasp (fingers vs cavity wall) | Insertion-alignment term in the grasp ranking (0.1 x approach angle to the opening normal) | (2) | `task9_reloc3` | aligned (yaw 90) grasps ranked first | retain |
T9 / T3 / T7 "FinePathIK ... excessive change" on the LINEAR approach | Continuity pre-validation: LINEAR dry-run `PlanTrajectory` from each pre-grasp/pre-contact IK solution (several seeds, posture families), free-space motion to the validated configuration, joint-space re-convergence | (1) PlanTrajectory/ComputeIk | `task3_dryrun`, `task3_settle`, `task9_reloc4` | dry run finds a feasible configuration in 1 test, but the reached configuration differs by 0.6-1.0 rad in joint space (OSC null-space drift) and the LINEAR plan fails from it; `ensure_same_branch` IK returns INTERNAL for every solution here although LINEAR plans exist (branch semantics differ from path feasibility) | retain the dry run (cheap, informative); re-convergence cannot work with OSC_POSE (documented limit) |
same | Collision-checked ANY approach when LINEAR is infeasible from the reached posture | (1) ANY planning | `task3_anyfb`, `task9_anyfb` | executes; T3 stage A reaches the panel on init 0 but the next segment fails; T9 approach reaches the grasp on init 1, pre-grasp tracking 3-5 cm off at the joint-2 limit | retain (explicit event; no regression in reg3 16/16) |
T3 bowl against the panel | Placement-depth term in the grasp ranking | (2) | `task3_depth` | chosen grasp/placement unchanged (hand clearance forces the front spot for every grasp) | keep the term (neutral), task unsolved |
| T9 insertion: Intrinsic IK rejects release/pre-place poses the adapter's box model accepted (hand vs microwave_1_main, 7/10 protocol states) | Intrinsic collision verdict inside the release search: collision-checked `ComputeIk` at each geometrically valid release (support excluded) and pre-place (full transport settings) pose, rejected candidates skipped (budget 12) | (1) ComputeIk with CollisionSettings | dev inits 0-4 (`runs/dev2/task9_verdict*`, records in `evidence/dev_task9_verdict`), patch `experiments/task9_intrinsic_verdict_hook.patch` | never triggered: on inits 0-2 the grasp after relocation fails (TRACK, as on the protocol); on inits 3-4 the verdict accepted the same candidate as the box model, the transport to the pre-place pose executed but ended 8 cm short (`place_slip_compensation` xy_correction 0.079-0.081 m with 1 mm attachment drift), and the LINEAR correction was rejected: invalid initial configuration, `robot0_link6` vs `microwave_1_microdoorroot` at the reached posture; after the failed correction the mug slipped 6-8 cm in the fingers and the second attempt found no collision-free hand pose | reject (no effect in 5 dev episodes; source reverted to 9b02207, patch kept as a record) |
Provenance | Revision captured at process start + dirty flag | - | all runs | removes the per-episode mislabelling (report 10.7) | retain |

Session 7 (third pass) additions:

Failure | Candidate method | Intrinsic support | Experiment | Outcome | Retain/reject
---|---|---|---|---|---
T3 push: no contact pose (top-down cannot finish, horizontal has no IK) | steeply pitched hand family (45-70 deg, 60-80 % panel height, lateral offsets, overshoot 2-3 cm) through the stage lab's geometry -> IK -> LINEAR dry-run funnel | (1) ComputeIk, PlanTrajectory LINEAR (never reached) | `evidence/dev_task3_lab_pitched` (state 0, 19 candidates) | all rejected at the end pose by the hand model (hand vs cabinet middle/top/base; lateral ones vs the bottle at mid travel) | reject (no feasible candidate); task 3 unsolved |
T9 pre-grasp after relocation at the joint-2 limit (4/10 protocol, 2/5 dev) | staging regrasp: top-down pick + place on the opening-normal line at 0.60-0.70 m from the base, then the level side pick | (1) ComputeIk margins (measured, not discriminating), PlanTrajectory for the extra pick/place | `runs/dev3/t9_stage*`, `t9_door5`, `t9_c4` (records in `evidence/dev_task9_success_c4`) | dev 5/5 (vs 3/5); +150 steps per episode | retain (C4) |
T9 door after the staging | start-state recovery before the pre-contact plan; ANY fallback for the first push segment after a completed approach | (1) PlanTrajectory ANY/JOINT | `runs/dev3/t9_door5` | dev states 1, 3 succeed | retain (C4) |
T9 pre-grasp error (alternatives) | re-validation LINEAR correction loop; longer joint settle; approach creep | (1) | `runs/dev3/t9_reval`, `t9_settle`, `creep_A/B` | 2/3, 0/2, no gain (tasks 1/7 unchanged) | reject (reverted) |

Session 8 (task 3 pass) additions:

Failure | Candidate method | Intrinsic support | Experiment | Outcome | Retain/reject
---|---|---|---|---|---
T3 closure blocked at -0.05 m (link5 vs cabinet top; stage B link5/6 vs wine rack) | pitched push on the handle bar, 30-50 deg forward-down, probed directly with Intrinsic at the closed end pose (no box pre-filter) | (1) ComputeIk at pre/contact/mid/end, PlanTrajectory LINEAR dry-runs | `scripts/t3_lab.py --probe handle` (dev state 0) | 30/40 deg: pre-contact IK rejected; 50 deg: fully feasible and executed | retain (C5 stage A) |
T3 bowl tilted on the inner wall | fingertips in the cavity pushing the far inner wall; near-rim regrasp; lateral rim regrasp + carry; near-rim fingertip push | (1) ComputeIk, PlanTrajectory with the attached bowl | `--probe chain/insert` | hand vs drawer; palm vs cabinet; attached bowl vs middle drawer; drawer slides instead of the bowl | reject (recorded) |
T3 bowl tilted on the inner wall | drawer configuration first: inside push to the joint limit (+1-2 cm of cavity in front of the handle bars) before the place | (1) PlanTrajectory LINEAR/ANY, ComputeIk | `--probe open` dev 0, 1; `run_task.py` dev 0-4 | level landing 7/7, official success 5/5 + reproduction | retain (C5) |
T3 place rejected the across-the-drawer rotation (22 deg grasp yaw) | exact-lateral carried-object rotation options for drawer regions | (2) geometry; (1) ComputeIk in the rotation assessment | dev 0-4 | chosen on 1/5 states (drop 1.5 cm), landing level on all | retain (drawer regions only) |

## 3. Catalogue of families considered (and why not pursued further)

| family | relevant failure | assessment |
|---|---|---|
| Behaviour-tree / FSM sequencing with symbolic preconditions and effects | all | present: predicate planner + per-skill preconditions/recovery; the drawer composite adds a bounded replan (obstacle relocation). A general backward-chaining planner was not needed: each task has a fixed goal set whose only non-trivial ordering is "place before close" and "knob before place". |
| Discrete search over manipulation order | T1 (butter vs cream cheese), T0/T7 | the second object's pick is not affected by the first placement in the cluttered cases (the blocker is a non-goal carton), so order search cannot help; not implemented. |
| Geometric feasibility before committing (placement-aware grasp ranking; stage B feasibility probes) | T3, T5, T9 | implemented and retained (section 2). |
| Backtracking after a failed grasp/placement choice | all | per-skill bounded attempts with candidate exclusion (`_tried`), release/start-state recovery; retained. |
| Receding-horizon execution with state refresh | pushes, places | every segment re-syncs the world (`UpdateObjectJoints`/`UpdateTransform`) and re-plans; attachment re-measured after transport and at place start; retained. |
| Antipodal / force-closure grasp reasoning | T1, T8 | pinch candidates are evaluated against the object's own geometry (opposing contact faces, width, contact count); no friction-cone/force-closure computation is made and none is claimed. Grasp verification uses finger gap, pad contacts, lift displacement and drift. |
| Multiple IK seeds, posture constraints, same-branch continuity | T3, T9 | all three are available (1) and now used in the push selection, the pre-contact check and the probes. Manipulability/Jacobian conditioning is not exposed by the IK response and was not computed locally (would be category 2 over FK Jacobians; not needed by the evidence). |
| Trajectory optimisation / planner selection / PRM | - | not exposed by the pinned release (3); ANY = Intrinsic's configuration-space planner, LINEAR = Cartesian line; path step size and collision check spacing are configurable (1) but the defaults were adequate (no executed-path collision attributable to check spacing). |
| Cartesian path constraints (RotationCone) during transport | T5 slip hypothesis | available (1); the slip hypothesis was falsified (section 2), so not used. |
| Impedance / admittance / hybrid force control | T3, T9 contact phases | no force sensing or compliant controller in LIBERO's OSC_POSE setup (3). Contact phases are position-controlled LINEAR segments with contact monitoring from simulator contacts (privileged, documented in report 7). |
| Swept-volume checks of the pushed door/drawer | T9 door, T3 | the arc/segment poses are re-planned after each sync with the articulated body at its true pose (1: planner collision checking); no separate swept-volume request was needed. |

## 4. Measured benefit and cost (protocol states 10-19)

| revision | success | plans | mean / max plan latency | executed-path collisions | notes |
|---|---|---|---|---|---|
| 76267bf baseline | 62/100 | 1,580 | 30 / 110 ms | 17 / 1,017 | - |
| 5b713c6 | 73/100 | 1,204 | 24 / 121 ms | 11 / 1,204 | evaluated with the cross-episode attachment defect |
| ce7685b (recommended) | 76/100 | 1,321 | 22 / 96 ms | 21 / 1,321 | task 3 episodes 13 min wall on average, max 26 min (IK probes with 5 s solver timeouts), others unchanged |
| 9b02207 | 74/100 | 1,388 | 22 / 92 ms | 30 / 1,175 | run interrupted after 26 episodes and resumed under the protocol (report 11.6); task 3 episodes 6.6 min on average (max 10.5); loses one episode each on tasks 1 and 4, no gains; task 9 now fails at the insertion (hand vs microwave body in Intrinsic's IK) instead of before it |
| 5e236de (C3, verified fallback) | 80/100 | 1,296 | 26 / 135 ms | 44 / 1,220 | official done-at-any-step accounting; task 9 4/10, tasks 5/6 +1, task 1 -2; 937 rejected task-3 probe requests (13 min per task 3 episode) |
| fbabe38 (C4, verified fallback; held-out states 30-39 once: 85/100) | 83/100 | 1,387 | 22 / 88 ms | 58 / 1,314 | staging regrasp + door fixes; task 9 7/10, no other task changed; 984 rejected probe requests (task 3: 46, task 9: 44 per episode); task 3 episodes 6-12 min |
| d5fca8d (C5, recommended) | 92/100 | 1,346 | 21 / 111 ms | 40 / 1,265 | task 3 9/10 (open-to-limit push, level landing, 50-deg handle push), every other task identical to C4; task 3 episodes 300-370 steps, 5 min |

Unresolved feasibility questions: (a) task 3: is any single-contact push able to close the last
3-6 cm with the bowl against the panel, or must the bowl be held back (two contacts) or placed
deeper (unreachable for the hand)? (b) task 9: does a level grasp 2-4 cm above the mug's
bottom exist that clears the table at the pick and the roof at the insertion (geometry says
yes, the candidate generator did not produce it)? (c) task 1 held-out: pre-manipulation of the
milk carton vs a thinner-finger approach; (d) whether Intrinsic's `DynamicCartesianLimits` on
transport segments reduces the residual slip/goal-not-reached cases (tasks 5, 6).

## 5. Ranked work queue (end of session 5, evidence-based)

| rank | item | failures affected | evidence strength | reusable fix? | cost | risk |
|---|---|---|---|---|---|---|
| 1 | Task 9 insertion with a level, low side grasp (H9a) | 10/100 protocol + 10/100 held-out | geometry diagnosis (cavity 20.7 x 15 cm, hand 21.8 cm; level low hand fits) | side-grasp family | 1 dev run | low (task 9 only) |
| 2 | Task 3: place the bowl deeper (grasp ranked by placement depth) so it does not reach the panel during stage A; posture families for the stage-A pre-contact | 10/100 (+ 10 held-out) | run-time probes name the bowl/panel blocker; 3/10 stage-A LINEAR failures | ranking shared by all picks (regression suite required) | 2 dev runs (13 min each) | medium |
| 3 | Task 1 held-out: butter against the milk carton | 9/100 held-out (protocol 10/10) | IK collision pairs (finger/hand vs carton 95x) | pre-manipulation skill (push carton) | high | medium |
| 4 | Tasks 5/6 goal-not-reached after a completed place (2-3/100) | 3/100 | records only (no video review yet) | unknown | low to diagnose | low |
| 5 | Task 7 pre-grasp tracking error (1/100) | 1/100 | tracking log | executor gains | low | medium (all tasks) |

Update after the 9b02207 protocol run (report 11.6):

| rank | item | failures affected | evidence strength | reusable fix? | cost | risk |
|---|---|---|---|---|---|---|
| 1 | Task 9: the transport into the microwave posture ends up to 8 cm short of the pre-place pose (OSC tracking near the joint-2 limit) and the LINEAR correction starts in contact (link 6 vs the open door). Candidate fix: ANY re-plan from the reached configuration to the pre-place pose with the full collision check (Intrinsic PlanTrajectory, category 1) instead of a LINEAR correction, and a posture="margin" IK target for the transport | 7/100 (PLACE) + dev 3-4 | dev records `evidence/dev_task9_verdict` (xy_correction 0.08 m, start-state pair link6/door) | place skill (regression required) | 2 dev runs + regression | medium |
| 1b | Task 9: Intrinsic verdict in the release search | - | rejected (see section 2, session 5 additions): never triggered | - | - | - |
| 2 | Task 3: stage B needs a contact pose with the bowl against the panel: two-contact strategy (hold the bowl back, or push on the panel edge beside the bowl) | 10/100 | 10 records (5 stage A, 5 stage B) | task-specific composite | 2-3 dev runs (6 min each) | medium |
| 3 | Pre-grasp tracking near joint limits (tasks 9: 3, 7: 1, 1: 1, 4: 1) | 6/100 | tracking logs; joint-2 limit posture (report 11.2) | posture="margin" IK for pre-grasp, re-plan from the reached pose | 1 dev run + regression | medium (all tasks) |
| 4 | Transport slip (task 6: 1, task 5: 1) | 2/100 | grasp verified then object lost; flips between runs | DynamicCartesianLimits on transport segments (Intrinsic, category 1) | 1 run | low |

Session 6 (final pass) additions:

Failure | Candidate method | Intrinsic support | Experiment | Outcome | Retain/reject
---|---|---|---|---|---
T3 bowl tilted at the drawer front | Release search: no raising over object-vs-wall blockers; carried-object rotation chosen by the release search; release-tilt family; finger zone from the measured gap; tilted grasps in the placement assessment | (2) geometry; (1) ComputeIk for the rotation tie-break | stage lab, states 0-1 | deep flat placement still rejected (handles of the drawers above); tilted/raised front placement unchanged | retain the general corrections (no raise over walls, gap-based finger zone, rotation choice); tilted-for-placement OFF (task 1/7 regressions) |
T3 push: no contact pose | stage B after a failed stage A; riders excluded from the probe; relocation only of free-joint bodies; higher contacts; base-turned posture; side-push family; corridor-aware relocation spot (worktree only) | (1) ComputeIk posture families | runs/dev2/task3_full_b..d, wt_t3 sidepush | all contact poses rejected (link 5 vs wine rack, wrist vs table, no IK; side push blocked by the relocated bottle) | retain the general fixes; side push and corridor spot not merged (untested effect) |
T9 insertion stops at the pre-place | slip-compensation prediction at the release (bug fix) | - | task9_slipfix* | insertion + release inside on dev 3, 4 | retain |
T9 door: wrong branch, FinePathIK, rebound | same-branch pre-contact ordering; ANY fallback for later segments; joint-space start-state retreat; hold + re-check after retreat; slower push | (1) PlanTrajectory ANY/JOINT, ComputeIk | task9_door*, task9_official* | complete successes dev 1, 3, 4; protocol half B (C2) 4/10 | retain |
Success accounting | LIBERO metric semantics (done at any step), termination on first done, final-state audit | - | all runs of the pass | +1 on tasks 5 and 6 (goal reached before the final retreat moved the object) | retain (documented) |
T1/T7 picks (regression of this pass) | validated-configuration pre-grasp + joint settle for top-down picks | (1) | protocol runs 4c20556, C2 | 8 cm approach tracking errors | reject for top-down picks (C3 uses ce7685b's nearest-IK path); kept for side grasps |
