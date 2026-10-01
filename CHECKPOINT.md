# CHECKPOINT (living file: completed work, exact commands, blockers, next actions)

Updated: 2026-09-30 (session 1)

## Pinned upstream revisions
- intrinsic-core: c61bf075f2335371c6367b61117e8a62bb960c3b (2026-09-30, shallow clone in third_party/intrinsic-core)
- LIBERO: 8f1084e3132a39270c3a13ebe37270a43ece2a01 (2025-03-15, shallow clone in third_party/LIBERO)
- bazel-central-registry: shallow clone in third_party/bazel-central-registry (needed because bcr.bazel.build is blocked by egress policy)

## Machine (cloud workspace)
4 vCPU, 15 GiB RAM, no GPU, ~30 GiB writable disk allowance. Ubuntu 24.04. Docker CLI present but no daemon; no k3s.
Rendering: Mesa EGL + OSMesa installed via apt (libegl1 libegl-mesa0 libgl1-mesa-dri libosmesa6). MUJOCO_GL=egl works on CPU.

## Completed
1. Python 3.10 venv (.venv) with robosuite==1.4.0, mujoco==2.3.7, numpy==1.23.5, bddl==1.0.1, gym==0.25.2, libero (editable via .pth). 
   LIBERO's benchmark loader needs torch only for torch.load of init files; we load them torch-free (src/libero_intrinsic/env/init_states.py), torch removed to save 6 GB.
2. LIBERO-10 env smoke test: all 10 tasks enumerated; OffScreenRenderEnv reset + set_init_state + step works (20 steps ~10 s on CPU incl. 128x128 EGL render).
3. Source surveys of both repos (see docs/contract.md).
4. Bazelisk installed (BAZELISK_BASE_URL=https://github.com/bazelbuild/bazel/releases/download since releases.bazel.build is blocked). Bazel 8.8.1.

## In progress
- Bazel build (background) of Intrinsic in-process planner service:
  cd third_party/intrinsic-core && bazel --output_base=/home/user/bazel_out build \
    --registry=file:///home/user/libero-intrinsic-bridge/third_party/bazel-central-registry \
    --jobs=3 --local_resources=memory=11000 --keep_going \
    //intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_in_process \
    //intrinsic/world/service/test:world_service_fake //intrinsic/world/conversion/sdf:sdf_to_world
  Log: build_logs/bazel_build_1.log

## Blockers / risks
- Egress policy blocks bcr.bazel.build, releases.bazel.build, mirror.bazel.build, sourceforge.net, go.dev, *.googlesource.com. Worked around with a local file registry; any dep hosted only on a blocked host will fail to fetch.
- Disk: LLVM toolchain + deps ~12 GB in /home/user/bazel_out; ~11 GB free after clearing caches.
- Intrinsic full runtime (k3s + release tarball, Ubuntu 26.04) is not deployable here -> use MotionPlannerServiceInProcess + FakeWorldService (both are official intrinsic-core C++ targets) served over local TCP gRPC.

## Next actions
1. Write docs/contract.md (verified facts), env wrapper, MJCF->SDF Panda converter.
2. C++ intrinsic_stack server: load SDF worlds, expose ObjectWorldService + MotionPlannerService on TCP.
3. Python gRPC client via Intrinsic's own py protos; FK/IK validation vs MuJoCo.

## Session 1, later (14:50 UTC)
Completed since last entry:
- src/libero_intrinsic/env/libero_env.py (env wrapper, torch-free task enumeration), env/executor.py (OSC action conversion + tracking metrics)
- src/libero_intrinsic/model/scene_to_sdf.py: compiled MuJoCo -> SDF (robot kinematic model + STL collision meshes + static objects). Verified: Python re-implementation of the exported chain reproduces MuJoCo TCP FK to 1e-15 m over 50 random configs (runs/dev/task0_world/task0.sdf generated).
- src/libero_intrinsic/model/transforms.py + tests/test_transforms.py (5 tests pass).
- src/libero_intrinsic/intrinsic/client.py: gRPC client on Intrinsic's own protos; IntrinsicUnavailableError, per-request JSONL log with ids; world_sync.py.
- intrinsic_stack/cc/libero_planner_server.cc + BUILD (symlinked as third_party/intrinsic-core/libero_bridge).
- scripts/gen_intrinsic_protos.py generated 91 pb2 modules into build/intrinsic_py.
Blocker: Bazel build still compiling (~7.9k/16.9k actions at 14:50). Server binary target to build next:
  bazel --output_base=/home/user/bazel_out build --registry=file://.../bazel-central-registry --jobs=3 //libero_bridge:libero_planner_server

## Session 1, 15:05 UTC
- All 10 task SDF worlds generate (runs/dev/worlds). Articulated parts confirmed: flat_stove_1_button (hinge), white_cabinet_1_bottom_level (slide, open -0.146), microwave_1_microjoint (hinge, open -1.58).
- Generic pinch-grasp sampler (skills/geometry.py grasp_candidates) finds candidates for every goal object incl. moka pot handle (1.3 cm), bowl rim, mug rims.
- Executor tuned on a synthetic joint trajectory with a MuJoCo-FK stub: tcp tracking 2 mm mean / 3 mm max; joint null-space deviation up to 0.30 rad (OSC pulls toward init posture) -> executed-path collision audit added (Intrinsic CheckCollisions on executed samples).
- Physics-only stepping (use_camera_obs=False): 26 ms/step vs 170 ms with camera obs; video rendered on demand every 2nd step.
- Frozen protocol declared in configs/eval_frozen.yaml (dev inits 0-4, eval inits 10-19, budget 1200 steps, also report <=600).
- Bazel: 10.4k/16.9k actions at 15:05; server build queued (build_logs/bazel_build_server.log).
Next: validate_kinematics -> demo_motion -> Task 0 dev runs -> fix skills -> all tasks -> frozen eval -> report.

## Session 2 (16:35 UTC) - Intrinsic integration proven
- Library build finished (102 min, 16,930 actions); server: build/bin/libero_planner_server -> bazel-out/haswell-opt/bin/libero_bridge/libero_planner_server (77 MB).
- Fixes for SDF-loaded worlds: (1) by-name object references need name_is_global_alias -> resolve ids via ListObjects and set the alias on the robot (UpdateObjectName) because the planner resolves the robot by name; (2) SDF <frame> needs intrinsic:create_attachment_entity="true"; (3) WorldFromSdf resolves the frame pose at q=0 (outside joint-4 limits) -> set link_t_frame via UpdateTransform with node_a_filter=eef link; (4) the planner needs exactly one frame named "flange" -> our tcp frame is named flange.
- Milestone 0 (evidence/milestone0_kinematics): 200 random configs: FK max err 5e-9 m / 2e-6 deg (Intrinsic ComputeFk vs MuJoCo site); IK 40/40 solved, max 1e-6 m / 1.3e-4 deg after setting the solution in MuJoCo; CheckCollisions flags link6/link5 vs table for an arm-in-table config (MuJoCo: 17 contacts) and OK at home.
- Milestone 1 (evidence/milestone1_first_intrinsic_motion): PlanTrajectory (75 states, 0.32 s, 30 ms latency) executed via OSC_POSE with auto time scaling 3.2x: 23 steps, tcp err 4.9 mm mean / 7.3 mm max, final 5.6 mm / 0.9 deg.
Commands: scripts/validate_kinematics.py --task 0 ; scripts/demo_motion.py --task 0 --init 0

## Session 2 (17:10 UTC) - Task 0 solved
- Milestone 2 (evidence/milestone2_task0_first_success): Task 0, init 0: SUCCESS in 324 steps (< LIBERO's 600). 18 PlanTrajectory calls (all <= 60 ms), 4 ComputeIk, 12 CheckCollisions (executed-path audits, all collision-free), 451 ComputeFk (per-step references). Video + JSONL request log saved.
- Fixes on the way: LINEAR re-plan protocol (Intrinsic's linear planner rejects pose targets whose IK differs from the path-IK end configuration by >1e-3 rad -> re-plan with the reached configuration, as Intrinsic recommends); fingers modeled as separate objects parented to the flange frame and synced to the real finger joints (open envelope could not enter the basket); container-aware release height (hand must clear the rim; measured hand bottom = 31 mm above tcp); placement slots perpendicular to the closing axis inside the region; free-spot search with release-height override; grasp contacts count finger bodies too.
- Running: task0 inits 1-4 (runs/dev/task0_dev), tasks 1 & 7 inits 0-1.
Next: articulation tasks (2, 3, 8, 9), mugs/plates (4, 6), book (5); then frozen eval.

## Session 2 (18:10 UTC) - dev results so far (init states 0-4 are development states)
| task | dev episodes | result |
|---|---|---|
| 0 | inits 0-4 | 5/5 success (324-363 steps) |
| 1 | init 1 | success (393); inits 0,2 failed before the latest placement fixes (rerunning) |
| 2 | init 0 | success (278) - knob turned to 0.79 rad, moka pot placed on burner |
| 3 | init 0 | fail: hand hit the upper drawer at the placement spot -> hand-aware spot selection added (rerunning) |
| 5 | init 0 | fail: book vs caddy slot -> fit rotation + geometric release height added (rerunning) |
| 8 | init 0 | success (458) - both moka pots on the burner |
| 4,6,7,9 | init 0 | running |
Key generalizations added today: partial gripper pre-opening (1 cm/step), neighbour clearance filter + tilted approaches, support-contact rules for thin objects and lifting, fingers as flange-attached objects synced to joint state, LINEAR re-plan protocol, container-aware placement (rim/hand/finger clearance search, fit rotation, slots perpendicular to closing axis, point-cloud free-spot search), nearest-IK joint targets for free-space motions.

## Session 2 (20:40 UTC) - frozen evaluation done
- 100 episodes (tasks 0-9 x init states 10-19) at revision 76267bf: 62/100 success (CI 52-71%); per task 10,5,10,0,10,2,9,8,8,0. docs/results.md (generated), docs/results.json, evidence/eval_episodes (videos + records + RPC logs per task, MANIFEST.json), docs/report.md sections 8.1-8.3.
- Not done: servo baseline comparison; posture-constrained IK for tasks 3/9; joint clamp bug (task 5).
Commands: python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/eval/frozen_A ; --tasks 5 6 7 8 9 --out runs/eval/frozen_B ; python -m libero_intrinsic.eval.report runs/eval --out docs/results.json ; python scripts/collect_evidence.py runs/eval

## Session 3 (2026-10-01 00:20 UTC) - improvement loop on the frozen baseline
Baseline preserved: commit 4f58be6/ec18bb3 results, evaluations/baseline_76267bf, docs/results.md unchanged.
Candidate commit 5b713c6 (dev-verified; details in docs/failure_table.md section 2 and docs/report.md section 9):
- Task 5 root cause was NOT grasp slip: the transport excluded the carried object vs its future support, so the Intrinsic plan swept the book through the caddy wall (video frames). Transport rule fixed; principal-axis fit rotations; footprint centring; region-box ledge rule (the book standing on the caddy floor can never satisfy LIBERO's In predicate: origin below the region box; verified in LIBERO source + MJCF). Dev 5/5.
- Task 3: placement now works (placement-aware grasp ranking, extended candidates, carried-object clearance); the drawer push (closing axis along the push, 20 deg tilt, 4 cm synced segments) closes 13/16 cm; every probed end-of-travel pose has only colliding IK solutions (scratch probe tables in docs/failure_table.md). Still 0 on dev.
- Task 1 regression from 0b28d05's exact support-plane test fixed (3/3 dev).
- Regression checks (inits 0-1): tasks 0,2,4,6,7,8 all 2/2.
- scripts/compare_evaluations.py: per-task before/after table with Wilson CIs.
Running now: repeated 100-episode protocol of 5b713c6 -> runs/eval_5b713c6/{A,B} (same configs/eval_frozen.yaml, inits 10-19, budget 1200, seed 0).
Commands: python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/eval_5b713c6/A ; --tasks 5 6 7 8 9 --out runs/eval_5b713c6/B ;
  python -m libero_intrinsic.eval.report runs/eval_5b713c6 --out docs/results_5b713c6.json ;
  python scripts/compare_evaluations.py evaluations/baseline_76267bf runs/eval_5b713c6 --labels baseline_76267bf candidate_5b713c6 --out docs/comparison_5b713c6.md
Resume steps if interrupted: check runs/eval_5b713c6/*.log for 50 run_id lines each; if incomplete, rerun the missing half from scratch (never merge partial reruns); then the report/compare commands above; archive records: python scripts/archive_evaluation.py runs/eval_5b713c6 evaluations/candidate_5b713c6 ; update README headline + report 9.4; commit; push.

## Session 3 (01:15 UTC) - repeated evaluation done
- Candidate 5b713c6, same 100-episode protocol (inits 10-19): **73/100** (CI 64-81%), per task 10,9,10,0,10,8,9,9,8,0; baseline 62/100 unchanged. 13 newly solved, 2 newly failed (6/15, 7/18). Records: evaluations/candidate_5b713c6; tables docs/results_5b713c6.md, docs/comparison_5b713c6.md; report 9.4-9.6; failure_table section 3. Commit ced2635, pushed.
- Running: held-out check on reserved init states 20-29 -> runs/heldout_5b713c6/{A,B} (A.log/B.log; 50 run_id lines each when done). When finished:
  PYTHONPATH=src python -m libero_intrinsic.eval.report runs/heldout_5b713c6 --out docs/heldout_5b713c6.json ; python scripts/archive_evaluation.py runs/heldout_5b713c6 evaluations/heldout_5b713c6 ; add the per-task line to docs/report.md 9.6 and README; commit; push. If interrupted, rerun the incomplete half from scratch (do not merge partial reruns).
- Open blockers: task 3 push last 3 cm (probe tables), task 9 side grasp; task 8 second-pot slip (2/10 both revisions).

## Session 3 (02:05 UTC) - held-out check done, session end
- Held-out (init states 20-29, candidate 5b713c6): **65/100** (CI 55-74%), per task 10,1,10,0,10,8,8,9,9,0; records evaluations/heldout_5b713c6, table docs/heldout_5b713c6.md, report 9.6, failure_table section 4. Task 1 gain does not transfer (butter against the milk carton on 9/10 states).
- Follow-up commit 25249ae (attachment refresh at place start; task 8 slip) is dev-verified only, NOT evaluated on the protocol; the evaluated candidate stays 5b713c6 (73/100).
- Resume: everything is committed and pushed on claude/youthful-galileo-0wtslh. To evaluate the follow-up, rerun the two evaluate.py halves at HEAD into runs/eval_<rev>, then report/compare/archive as in the session-3 commands. Open: task 3 last 3 cm of the drawer push (probe tables), task 9 side grasp, task 1 cluttered butter (pre-manipulation / push-aside skill), task 5 ledge placement robustness (2 slips in 20 eval episodes).

## Session 4 (15:45 UTC) - second improvement loop (method families), re-evaluation running
- docs/methods.md: verified Intrinsic capabilities table, method-selection table, family catalogue.
- Code (commit ce7685b): opening-normal side grasps + pitched/low side candidates; two-stage drawer push (stage A top-down partial, stage B horizontal with run-time IK contact selection: seeds + posture families); DrawerCloseComposite with obstacle relocation (pick/place the movable blocker); same-branch pre-contact check; start-state recovery; release separation; release search path validation + raise-outer ordering + roof cap; place verification against the region box; conditional footprint centring; posture-margin IK option; IK seed clamping; episode-start attachment reset (cross-episode leak found: affects 5b713c6's tasks 3/9 episodes after a failed place).
- Dev: tasks 0,1,2,4,5,6,7,8 inits 0-1 = 16/16 (runs/dev2/reg); task 3 0/2 (bowl against the panel blocks the final push), task 9 0/2 (hand cannot enter the cavity with the grasps found).
- Running: repeated 100-episode protocol of ce7685b -> runs/eval_ce7685b/{A,B}. When done: PYTHONPATH=src python -m libero_intrinsic.eval.report runs/eval_ce7685b --out docs/results_ce7685b.json ; python scripts/compare_evaluations.py evaluations/baseline_76267bf runs/eval_ce7685b --labels baseline_76267bf candidate_ce7685b --out docs/comparison_ce7685b.md ; python scripts/compare_evaluations.py evaluations/candidate_5b713c6 runs/eval_ce7685b --labels candidate_5b713c6 candidate_ce7685b --out docs/comparison_5b713c6_to_ce7685b.md ; python scripts/archive_evaluation.py runs/eval_ce7685b evaluations/candidate_ce7685b ; fill report 10.5 + README; commit; push. If interrupted: rerun the incomplete half from scratch, never merge partial reruns.

## Session 4 (18:30 UTC) - candidate ce7685b evaluated: 76/100
- Repeated protocol (inits 10-19): **76/100** (CI 67-83%), per task 10,10,10,0,10,8,9,9,10,0; vs 5b713c6 (73): +1/18, +5/17, +6/15, +7/18, +8/11, +8/15 solved, -5/12, -6/18, -7/11 failed. Records evaluations/candidate_ce7685b; docs/results_ce7685b.md; comparisons; report 10.5-10.6; failure_table 6; methods.md 4; README.
- Open items and next experiments: docs/methods.md section 4. Everything committed and pushed; raw runs with videos under runs/ (not committed).

## Session 5 (23:45 UTC) - verification, Task 9 sequence change, candidate 9b02207 pending
- Verified chain (report 10.7/11.1). Recorded-revision caveat documented; provenance now captured at process start.
- Code 9b02207: process-start provenance; placement-depth + insertion-alignment grasp ranking; pick-stage obstacle relocation (assess-first) for roofed containers; LINEAR dry-run validation of pre-grasp/pre-contact configurations; joint-space re-convergence attempt; ANY approach fallback on FinePathIK (documented controller limit, report 11.2); cheaper push probes.
- Dev: reg2 (b205f2a) 16/16; reg3 (9b02207) running -> runs/dev2/reg3. Task 9: relocation + grasp reached, approach/pre-grasp tracking fails (0/2). Task 3: 0/1.
- Recommended candidate so far: ce7685b (76/100, evaluations/candidate_ce7685b). 9b02207 is NOT evaluated on the protocol yet.
- Evidence for ce7685b committed: evidence/eval_episodes_ce7685b (13 episodes: videos, records, RPC logs, MANIFEST.json).
- Next: if reg3 is clean, run the protocol on 9b02207: python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/eval_9b02207/A ; --tasks 5 6 7 8 9 --out runs/eval_9b02207/B (about 2.5 h wall: task 3 episodes 10-25 min each). Then report/compare/archive as for ce7685b (docs/results_9b02207, docs/comparison_9b02207.md, evaluations/candidate_9b02207). If interrupted: the partial run is NOT a result; keep ce7685b as the recommended candidate and report 9b02207 as "interrupted before final validation".
