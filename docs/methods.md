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
T3 same | Run-time contact-point selection by IK feasibility of pre/contact/mid/end poses, with seed diversity and posture families | (1) ComputeIk, max_solutions 8 | `planner.drawer_close_geometry` (event `push_contact_selection`); dev inits 0-1 | hybrid run 1: stage A +6 cm, stage B all candidates "end: collision" with the single post-push seed (4 solutions); run 2 with seeds: init 1 all "link5 vs wine bottle"; run 3 adds shoulder-forward posture family | pending (see 9.x) |
T3 bowl placement path | Validate the straight descent between pre-place and release (5 samples) against hand/object clearance, not only the endpoints | (2) geometry; the plan itself is Intrinsic LINEAR | `_release_search.valid()`; failure seen in eval (3/10, carried bowl vs middle drawer along the lowering path) | pending re-evaluation | retain (no cost; removes a known failure mode) |
T3/T9 plan rejected at start ("Invalid initial joint configuration", finger touching the just-released object) | Bounded recovery: retreat 5 cm up with only the offending pair excluded, re-sync, re-plan once | (1) collision rules + LINEAR plan | `_recover_start_state` (event `start_state_recovery`) | pending | retain if no regression |
T9 side grasp blocked (wrist vs neighbouring mug, forearm vs open door) | Grasp geometry: oblique (pitched-down 20/35 deg) side approaches | (2) candidate generation; IK (1) | `probe_t9b.py`: level side hand -> wrist vs porcelain mug at every spot; 30 deg pitch at the microwave mouth: 8 solutions | dev init 0: pick succeeds (pitch 35), init 1: second candidate succeeds after a slip detected by lift verification | retain |
T9 staging the mug elsewhere first (TAMP re-sequencing) | Pick-place to a staging spot, then side grasp | (2) | same probe: staging spots at x >= 0.08 have no IK solution for a +y horizontal hand; spots at x <= 0 collide with the porcelain mug (wrist) | no feasible staging spot found in the probed set | reject for now |
T9 mug released on the microwave roof | Rim rule restricted to top-down approaches; release raised only while the object stays under the region box top; place verification against the region box | (2) | dev inits 0-1 | pending | pending |
T9 door closing arc | Arc path with per-segment orientations (hinge axis from the model) | (2) over LINEAR plans (1) | `probe_door.py 0`: pre and first 3 arc poses 8 solutions with every seed; later poses collide only with the mug left on the roof | door closed in dev init 0 (segment 4 satisfied) once reached | retain |
T5 book not resting in the box | Region-box ledge rule (place on an internal horizontal ledge) | (2) | dev 5/5, eval 8/10 | retained earlier (report 9.1) | retain |
T8 second pot slip | Re-measure the attachment at place start | (2) | dev 4/4 | retained (report 9.7) | retain |
