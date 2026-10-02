# Failure table

Stage labels: SEQ task sequencing/goal, FRAME frame/world sync, REACH unreachable target or
posture, IK IK branch/joint limit, PLAN collision/planning, GRASP alignment/contact/slip,
TRACK tracking/controller, PLACE placement/release/retreat, ARTIC articulation, INFRA timeout/runtime.
"Observed" = read from episode records / RPC error payloads / video frames; "Hypothesis" = inferred.

## 1. Baseline (revision 76267bf, 100 episodes, evaluations/baseline_76267bf)

| task | failures | stage | observed evidence | hypothesis / fix candidate |
|---|---|---|---|---|
| 1 | 5/10 | PLAN (pre-grasp IK collision) | ComputeIk collisions: hand vs table (2: tilted grasp dips below the table plane), finger/hand vs basket (2), finger vs milk carton (1) | coarse point-cloud clearance misses thin overlaps -> exact support-plane corner test; container walls need finer sampling |
| 3 | 10/10 | REACH (push pre-contact) | 30/30 ComputeIk errors: robot link5/link6 vs wine_rack when reaching the drawer front horizontally | the wine rack (33 cm tall) occupies the corridor of a frontal push -> push from above with the hand tilted away from the cabinet |
| 5 | 8/10 | GRASP slip + PLACE (4), INFRA-adapter (3), REACH (1) | release 16-20 cm off target after the 90 deg fit rotation with lift drift ~0 (slip during rotation, stale tcp_t_obj); 3x UpdateObjectJoints OUT_OF_RANGE (joint 7 0.3 mrad past limit); 1x no IK | re-measure attachment after transport (slip compensation); clamp synced joints |
| 6 | 1/10 | PLACE (lowering IK) | PlanTrajectory: carried mug vs table at the lowering target | stale attachment offset (mug slipped down) -> slip compensation incl. height |
| 7 | 2/10 | TRACK/IK | pre-grasp executions with joint error 1.2-1.5 rad, joint-limit violation (>1 mrad), 18 deg orientation error on approach | large reconfigurations tracked poorly by OSC; limit tolerance too strict (MuJoCo limits are soft) |
| 8 | 2/10 | GRASP slip + PLACE | second pot: lift drift 2.3 cm, released 2 cm off, resting on the first pot (z 1.003 vs 0.997) | slip compensation |
| 9 | 10/10 | PLAN/REACH (side-grasp IK) | ComputeIk collisions: link5/hand vs the OPEN microwave door (21), link6 vs porcelain mug (6), unreachable (3); elbow-up JointPositionLimits constraints: still 0 solutions | the open door stands between robot and mug at hand height; needs an approach from +/-y with a steep forearm, or a different manipulation strategy |

## 2. Improvement loop after the baseline (dev init states 0-4; records under runs/dev2, not committed)

Each row: the hypothesis from section 1, what the dev reruns actually showed, the change kept, and the dev evidence.
Revisions: 0b28d05 = previous session's post-baseline commit, 5b713c6 = candidate frozen for the repeated evaluation.

| task | stage (revised) | observed in dev reruns | change kept (classical) | dev evidence |
|---|---|---|---|---|
| 5 | **PLAN (transport collision rule)**, not GRASP slip | Video frames (runs/dev2/task5_b, init 0, steps 92-146): the carried book is swept through the caddy wall during the free-space transport and knocked out of the fingers; `place_slip_compensation` then measured 13-26 cm "drift". Cause in code: `transport_collision_settings(support)` excluded the (attached object, support) pair for the whole transport, so Intrinsic's planner was allowed to pass the book through the caddy. The "pendulum slip" hypothesis (slow transport, time_scale 2.5) changed nothing (0/3) and was removed. | The exclusion now applies only to the contact-monitored lowering; the transport is planned with the carried object checked against the support (`place_transport_rule` event). | drift after transport 2-5 mm (was 130-260 mm) on inits 0-4 |
| 5 | PLACE (orientation) | The book is held diagonally (footprint principal axis 40-60 deg off the compartment axis); `fit_rotation` only tried quarter turns, so the 11 x 2.9 cm footprint did not fit the 12.4 x 5.6 cm compartment. | `fit_rotations`: principal-axis (PCA) alignment angles added to the candidates; every equally fitting angle is IK-probed for joint-limit margin. | `place_fit_rotation` -46..-60 deg chosen, overhang 0 |
| 5 | PLACE (release height) | With the aligned book the release was still raised 11 cm: the book's origin is 8 mm off its footprint centre, leaving < 5 mm to one compartment wall (`place_release_raised.blockers` = objwall). | Footprint centring in the release search (origin target shifted by the footprint offset, origin kept inside the region box). | release raised 3 cm (fingers above the rim) instead of 11 cm |
| 5 | **SEQ/PLACE (predicate geometry)** | A clean insertion (book standing on the compartment floor, xy error 1.5-3.5 mm) FAILS the task: LIBERO `In` = contact AND object-origin-in-region-box; the book's origin is its bottom face (local z 0.002) and the back-compartment box starts 1.2 cm above the caddy floor (verified: `black_book` geom g1, region half z 0.0605, floor top 0.9008). The five earlier "successes" were books that tumbled and came to rest leaning on the 6.7 cm divider (origin z 1.00-1.04). Releasing at the entry pose (5 cm drop): 1/5, book stands. Releasing at the mouth (14 cm drop): the centred book falls straight and stands. | Region-box ledge rule (`PlaceSkill._region_box_targets`): if the origin resting on the floor would be below the region box, place the object's bottom on the lowest horizontal ledge of the container inside the box (top face of a collision box; the divider), offset 4 mm outward and centred along the ledge, contact-monitored lowering, release; the object tips onto the outer wall. | **5/5** (runs/dev2/task5_m); 3/5 with a wrongly detected end-wall ledge (fixed: ledge long side must follow the region's long axis) |
| 3 | PLACE (hand vs upper drawers) -> solved | At 0b28d05 the bowl placement was blocked: hand zone vs cabinet middle/top (release raised 14 cm, then the carried bowl hit the upper drawer) and, after adding carried-object clearance, every spot rejected (bowl rim 2 mm from the cabinet frame at the region centre). | Placement-aware grasp ranking (grasp admitting a valid release first), extended container candidates (origin may use the whole region box: the predicate tests the origin), carried-object clearance in the release search. | bowl placed in the drawer on inits 0-1 (xy error 7 mm), 2/2 |
| 3 | **REACH/PLAN (drawer push, last 3 cm)** | Push pre-contact with the previous geometry (closing axis perpendicular to the push, 50 deg tilt): "no IK solution" (kinematic). IK probe tables (scratch, Intrinsic ComputeIk at init 0, 1 call per pose): closing axis along the push, tilt 0/20 deg -> pre-contact and contact poses solvable; perpendicular -> contact in collision (finger vs drawer); tilt >= 35 deg -> no solution. With the new geometry the drawer closes 13 of 16 cm; every end-of-travel pose probed (top-down: yaw 0/90, tilt -30..+15, lateral offset 0/+-8 cm, heights 25/50 %; horizontal hand: roll 0/90, offsets, heights) has only colliding IK solutions: wrist/link5 vs the cabinet top or the hand vs the middle drawer front (the closed drawer front is flush with the upper drawer fronts, everything above z 0.984 and beyond y 0.197 is blocked), and horizontal pushes put link5 into the wine rack / the hand into the wine bottle. | Closing axis along the push, 20 deg tilt; pushes split into <= 4 cm segments planned after a world sync (objects carried by the drawer at their true poses: the one-segment end pose reported finger vs bowl). | 0/2 (push reaches q ~ -0.03..-0.04 of the required > 0.0) |
| 1 | PLAN (pre-grasp) -> **regression at 0b28d05 fixed** | 0b28d05 scored 0/2 where 76267bf scored 1/2 (same dev states): the exact support-plane test rejected every candidate (the cream cheese is 1.8 cm tall; the conservative gripper zones dip 2-7 mm below the table even for feasible grasps) and a candidate's grasp-IK collision (hand vs milk) aborted the whole attempt instead of trying the next candidate. | Zone margin of 4 mm tolerated below the plane; dipping candidates lifted (pads must still overlap the object); per-candidate IK errors are non-fatal. | **3/3** (inits 0-2) |
| 0,2,4,6,7,8 | regression checks | shared Pick/Place/Push code paths | - | 2/2 each on inits 0-1 (runs/dev2/task*_reg) |
| 9 | PLAN/REACH | not revisited (no new idea beyond section 1) | - | still 0 expected |

Probe scripts (not part of the pipeline; they only issue ComputeIk requests against the synced
world): scratch `probe_push*.py`; their tables are summarised above and in docs/report.md 9.

## 3. Candidate 5b713c6, repeated evaluation (100 episodes, evaluations/candidate_5b713c6)

| task | failures | stage | observed evidence |
|---|---|---|---|
| 1 | 1/10 | REACH | init 18: butter, no reachable grasp after the clearance filter (neighbours) |
| 3 | 10/10 | ARTIC/REACH (7), PLAN (3) | push segments 1-2: only colliding IK solutions (wrist/link5 vs cabinet top, hand vs middle drawer front); inits 10/17/18: lowering LINEAR path invalid (bowl/hand vs cabinet along the path for an extended placement candidate) |
| 5 | 2/10 | REACH (1), PLACE (1) | init 18: no reachable grasp; init 17: ledge placement executed (ledge z 0.986) but the book slid off and stands on the floor (origin z 0.899 < box 0.912, xy error 3.5 cm) |
| 6 | 1/10 | PLACE | init 15 (baseline solved): pudding lost during the lowering, final position 0.67 m from the target |
| 7 | 1/10 | PLAN | init 18 (baseline solved): second pick, LINEAR approach rejected: start configuration in collision |
| 8 | 2/10 | GRASP slip + PLACE | second pot: lift drift 1.5-2.3 cm, released 7.6 cm off (inits 11, 15) |
| 9 | 10/10 | REACH (9), IK (1) | no reachable horizontal grasp of the mug next to the open door (unchanged) |

Probe 5 (strongly tilted-away top-down pushes, tilt -45/-60 deg, yaw 0/90, two lateral offsets,
two heights, end-of-travel poses): every pose has only colliding IK solutions (link5/6 vs the
cabinet top or middle, link5 vs the wine bottle). The drawer's last 3 cm are unreachable for a
fingertip push in this scene with the hand poses probed.

## 4. Candidate 5b713c6, held-out init states 20-29 (evaluations/heldout_5b713c6, 65/100)

| task | failures | stage | observed evidence |
|---|---|---|---|
| 1 | 9/10 | PLAN (grasp IK collision) / REACH | butter (8) or cream cheese (1): every cleared or raw candidate rejected by ComputeIk; collision pairs over all rejected solutions: finger/hand vs milk carton 95, fingers vs basket 12, hand vs orange juice 3 |
| 3 | 10/10 | ARTIC/REACH (7), PLAN (3) | push segment IK as in section 3 (7), push start configuration invalid (2), lowering path (1) |
| 5 | 2/10 | PLACE (1), REACH (1) | book not resting inside the box after the ledge placement (1); no grasp candidate after the clearance filter (1) |
| 6 | 2/10 | PLACE (1), PLAN (1) | goal not reached after a completed place (1); pick plan: IK collision (1) |
| 7 | 1/10 | PLAN | pick LINEAR plan rejected |
| 8 | 1/10 | GRASP slip + PLACE | second pot (see 9.7 follow-up) |
| 9 | 10/10 | REACH (8), IK/PLAN (2) | unchanged |

## 5. Second improvement loop (dev states 0-1; records under runs/dev2/task3_*, task9_*, reg)

| task | stage | observed (dev) | change kept | result |
|---|---|---|---|---|
| 3 | PLACE: bowl rim hooked on a finger after release, lifted out with the retreat (video frames) | finger contacts 2 after opening; the next plan's start state in collision with the bowl | release separation (move 2.5 cm away while contacts persist), start-state recovery (retreat with the pair excluded) | bowl stays in the drawer |
| 3 | ARTIC: top-down push ends ~6 cm short | IK probes: no collision-free top-down end pose (4 seeds, elbow-up limits) | stage A with partial progress + stage B horizontal hand with run-time IK contact selection | 10/16 cm closed |
| 3 | ARTIC/PLAN: stage B blocked | init 1: link5 vs wine bottle for all 70 probes; after relocating the bottle: hand vs bowl (bowl against the panel) or no solution; init 0: stage A LINEAR approach "FinePathIK excessive change" (0 of 8 pre-contact solutions branch-consistent) | obstacle relocation composite (executed, 165 steps), same-branch pre-contact check | unsolved (0/2) |
| 9 | PLAN: level side grasp, wrist vs porcelain mug | probes at 6 spots | pitched side grasps, low heights, opening-normal yaws | pick 2/2 |
| 9 | INFRA (adapter): mug still attached in the Intrinsic world from the previous episode | pre-grasp IK "mug vs microwave" in init 1 | episode-start re-parenting of all bodies | fixed |
| 9 | PLACE: insertion | every release candidate rejected: hand above the roof (pitched hand) or hand wider than the cavity; earlier runs released the mug on the roof and reported success | roof cap, region-box verification (reports failure) | unsolved (0/2) |
| 0,1,2,4,5,6,7,8 | regression (inits 0-1) | - | - | 16/16 |

## 6. Candidate ce7685b, repeated evaluation (100 episodes, evaluations/candidate_ce7685b, 76/100)

| task | failures | stage | observed evidence |
|---|---|---|---|
| 3 | 10/10 | ARTIC (stage A 3, stage B 7) | stage A: LINEAR approach to the panel rejected (FinePathIK) on 3 states; stage B: after relocating the wine bottle by real manipulation, every probed horizontal contact pose is rejected (hand vs the bowl that slid against the panel, or no solution) |
| 5 | 2/10 | PLACE (1), REACH (1) | book not resting inside the box after the ledge placement (12); no reachable grasp (18) |
| 6 | 1/10 | PLACE | goal not reached after a completed place (18) |
| 7 | 1/10 | TRACK | pre-grasp execution final pose error 2.9 cm / 1.9 deg (11) |
| 9 | 10/10 | PLACE | pick succeeds on every state (pitched side grasps); no release pose inside the cavity: the hand is wider than the 20.7 cm cavity and a pitched hand rises above the roof |

## 7. Third loop (dev states 0-1; records under runs/dev2/task9_reloc*, task9_anyfb, task3_dryrun, task3_settle, task3_anyfb, reg2, reg3)

| task | stage | observed | change kept | result |
|---|---|---|---|---|
| 9 | PLAN (pick): level grasps along the opening normal blocked by the porcelain mug (hand body), pitched grasps blocked at the insertion (roof) | `grasp_candidates.blockers` = porcelain mug 14-17, table 6; 0 placement-compatible of 13-16 | assess-first obstacle relocation of the porcelain mug (real pick/place, ~90 steps) | 8 compatible grasps, pick reaches the grasp |
| 9 | PLAN/TRACK (approach): LINEAR approach from the reached posture fails ("FinePathIK"), ANY fallback reaches the grasp on init 1 but the pre-grasp tracking error (3-5 cm, 9-11 deg) at the joint-2 limit aborts the attempts | joint error after the free-space motion 0.6-1.0 rad although the TCP is within 2 cm (OSC null-space drift) | LINEAR dry-run validation + ANY fallback (both recorded) | 0/2; insertion stage not reached |
| 3 | PLAN (stage A approach): same FinePathIK mechanism on init 0 | `precontact_path_check` feasible, `execute.joint_err_final` 0.64-0.99 rad, LINEAR seg0 fails | ANY fallback reaches the panel; segment 1 fails | 0/1 |
| 0,1,2,4,5,6,7,8 | regression | - | - | 16/16 at b205f2a and 16/16 at 9b02207 |

## 8. Candidate 9b02207, repeated evaluation (100 episodes, evaluations/candidate_9b02207, 74/100; interrupted at 26 episodes by a container restart and resumed under the same protocol, see resume_manifest.json)

| task | failures | stage | observed evidence |
|---|---|---|---|
| 1 | 1/10 | REACH after TRACK | second pick: approach tracking error 8.2 cm, then no reachable grasp for two attempts (17); 10/10 at ce7685b |
| 3 | 10/10 | ARTIC (stage A 5, stage B 5) | stage A: ANY-fallback approach reaches the panel, next LINEAR segment NOT_FOUND (10, 13, 14, 18); pre-contact unreachable (12); stage B after relocating the wine bottle: every probed horizontal contact pose rejected (11, 15, 16, 17, 19) |
| 4 | 1/10 | PLACE | both places completed (xy 2.6 / 1.2 cm), goal predicate false; 8.6 cm pre-grasp tracking error near the first placed mug (11); 10/10 at ce7685b |
| 5 | 2/10 | PLACE (1), REACH (1) | same states and reasons as ce7685b (12, 18) |
| 6 | 1/10 | PLACE | second mug 76 cm from target after transport, released outside the region box (19); state 18 (failed at ce7685b) succeeded |
| 7 | 1/10 | TRACK | same as ce7685b (11) |
| 9 | 10/10 | PLACE (7), TRACK (3) | relocation executed on 10/10; mug grasped on 7/10 and a release target inside the cavity found, but Intrinsic IK reports hand (gripper0_right_gripper) vs microwave_1_main.link at the pre-place/release configuration (11, 13, 15, 16, 17, 18, 19); pre-grasp after relocation 2.7-4.3 cm / 7-11 deg off at the joint-2 limit (10, 12, 14) |

Evidence episodes (video + record + request log) for this run: `evidence/eval_episodes_9b02207`
(task 9 insertion failure 13, task 9 tracking failure 10, task 3 stage A 10 and stage B 11,
task 1 regression 17, task 6 slip 19). The other 94 episodes exist only as records
(`evaluations/candidate_9b02207`); their videos are in the temporary `runs/` directory.

## 9. Task 9 dev experiment after the 9b02207 run (states 0-4, runs/dev2/task9_verdict*, records of states 3-4 in evidence/dev_task9_verdict; code = 9b02207 + experiments/task9_intrinsic_verdict_hook.patch, not retained)

| state | stage | observed evidence | hypothesis |
|---|---|---|---|
| 0, 1, 2 | TRACK | pre-grasp after relocation 3.2-4.6 cm / 8-11 deg off | joint-2-limit posture (report 11.2) |
| 3, 4 | TRACK then PLAN | verdict IK accepted the release/pre-place poses (0 rejections); transport plan OK and executed; arm 7.9-8.1 cm short of the pre-place pose with 1 mm attachment drift; LINEAR correction rejected (invalid initial configuration: robot0_link6 vs microwave_1_microdoorroot); start-state recovery IK NOT_FOUND; mug then slipped 6-8 cm; second attempt: no collision-free hand pose (fingers/hand vs microwave_1_main) | OSC does not reach the planned pre-place configuration near the joint-2 limit; the reached posture touches the open door |

## 10. Final pass diagnostics (development states; records in evidence/dev_task3_lab, evidence/dev_task9_success, runs/lab, runs/dev2)

| task | stage | observed evidence | classification |
|---|---|---|---|
| 3 | PLACE (first divergence) | bowl released 3 cm above a spot overlapping the inner front wall; lands tilted 30 deg (contacts: bowl g4 vs drawer inner wall g35, bowl bottom vs floor); 24 deeper spots rejected: palm vs upper drawers' handles (y >= 0.188, z 1.007-1.097) for every rotation and release tilt | MODEL (box clearance) + geometry: 9.2 cm entry width for a 10.8 cm bowl |
| 3 | ARTIC stage A | pre-contact reached with 0.99 rad joint error (reach limit, elbow straight); LINEAR seg0 FinePathIK; ANY fallback ends 2.4 cm short at the joint-4 limit | EXECUTION (OSC branch) |
| 3 | ARTIC stage B | 84/84 contact poses rejected in 3 posture families: link 5 vs wine rack (contact), link 6 vs table (end), no IK (contact/end); after relocating the bottle: same; side-push family: 84/84 end poses blocked by the relocated bottle (link 5/6) | PLANNING (collision) |
| 9 | PLACE (first divergence, fixed) | slip compensation moved the hand 8 cm deeper (xy_correction 0.079-0.081 m with 1 mm drift); lowering plan: hand vs microwave_1_main | TASK-LOGIC bug |
| 9 | ARTIC (fixed) | pre-contact 2.3 rad from the current configuration; seg1 FinePathIK at the joint-5 limit; door rebounds to -0.28 rad after the retreat | EXECUTION (OSC branch) + CONTACT |
| 9 | TRACK (open) | states 0, 2 (dev) and 10, 12, 14, 19 (protocol, C2 half B): pre-grasp after relocation 2.7-4.6 cm / 7-11 deg off at the joint-2 limit | EXECUTION |
| 9 | PLACE (open) | states 11, 18 (protocol, C2 half B): ComputeIk NOT_FOUND at the release/pre-place (hand vs microwave frame) | PLANNING (collision) |
| 1, 7 | GRASP/TRACK (regression, fixed in C3) | 4c20556 / C2: approach tracking 8 cm after a validated-configuration pre-grasp; ce7685b path tracks the same approach with 0.1 rad error | EXECUTION |
