# libero-intrinsic-bridge

Classical (non-learned) manipulation for the LIBERO-10 benchmark whose forward/inverse
kinematics, collision checking and motion planning are computed by **Intrinsic Core**
(`intrinsic-ai/intrinsic-core`), executed in the unmodified LIBERO/MuJoCo environment.

Status, evidence and honest limitations: see `docs/report.md` (technical report),
`docs/results.md` (baseline results table), `docs/results_5b713c6.md` (candidate results table), `docs/comparison_5b713c6.md` (before/after), `docs/failure_table.md` (failure stages with evidence), `docs/contract.md` (facts verified from the pinned
sources), `docs/architecture.md`, `CHECKPOINT.md` (work log).

## Headline result
Repeated frozen evaluation (same protocol as the baseline: 100 episodes, 10 official init states
per task disjoint from development states, budget 1,200 steps, seed 0):

| revision | result | per task (0-9) |
|---|---|---|
| baseline `76267bf` (archived, unchanged) | **62/100** (Wilson 95% CI 52-71%) | 10, 5, 10, 0, 10, 2, 9, 8, 8, 0 |
| candidate `5b713c6` | **73/100** (CI 64-81%) | 10, 9, 10, 0, 10, 8, 9, 9, 8, 0 |
| candidate `ce7685b` (verified fallback) | **76/100** (CI 67-83%) | 10, 10, 10, 0, 10, 8, 9, 9, 10, 0 |
| candidate `9b02207` (archived, not recommended) | 74/100 (CI 65-82%) | 10, 9, 10, 0, 9, 8, 9, 9, 10, 0 |
| candidate `5e236de` (C3, verified fallback) | **80/100** (CI 71-87%) | 10, 8, 10, 0, 10, 9, 10, 9, 10, 4 |
| candidate `fbabe38` (**C4, recommended**; third pass) | **83/100** (CI 74-89%) | 10, 8, 10, 0, 10, 9, 10, 9, 10, 7 |

Candidate fbabe38 (C4) is the third pass's result: task 9 (mug in the microwave, door closed) is
solved on 7 of 10 protocol states (a physically executed staging regrasp before the side pick plus
two door-push fixes, docs/report.md 13.2) and no other task changed an episode relative to C3
(docs/comparison_5e236de_to_fbabe38.md). Success follows LIBERO's own metric
(`libero/lifelong/metric.py`: done at any control step; the episode ends there); every success
record carries the goal atoms evaluated by LIBERO's predicate functions (83/83 true). The 1,387
planned motions of that run (mean 22 ms, max 88 ms, no timeouts) are all Intrinsic
`PlanTrajectory` results; the 984 "plan failures" counted are rejected IK/plan probes of the task 3
contact search and the task 9 release/door probes (every one logged with its reason). A one-time
held-out evaluation on untouched official states 30-39, declared before it ran
(`evaluations/heldout_fbabe38/MANIFEST.json`) and never used for tuning, gave **85/100**
(CI 77-91%; per task 10, 9, 10, 0, 10, 10, 10, 10, 10, 6; `docs/heldout_fbabe38.md`,
docs/report.md 13.7); those states are now consumed.
Per-task before/after with the episodes that changed: `docs/comparison_fbabe38.md`,
`docs/comparison_5e236de_to_fbabe38.md`, `docs/comparison_5e236de.md`,
`docs/comparison_ce7685b_to_5e236de.md`, `docs/comparison_ce7685b.md`,
`docs/comparison_5b713c6_to_ce7685b.md`, `docs/comparison_ce7685b_to_9b02207.md` (the later
candidate loses one episode each on tasks 1 and 4 and gains none, so ce7685b stays recommended;
its run was interrupted by a container restart and resumed under the protocol, see
`docs/report.md` 11.6 and `evaluations/candidate_9b02207/resume_manifest.json`). Every motion executed was an Intrinsic
`PlanTrajectory` result (candidate ce7685b: 1,321 plans, mean 22 ms, max 96 ms, no timeouts);
every success is within LIBERO's 600-step horizon. Tasks 3 and 9 remain unsolved
(docs/report.md 10.2-10.3, docs/methods.md). A separate held-out run of candidate 5b713c6 on
reserved init states 20-29 gave 65/100 (CI 55-74%; per task 10, 1, 10, 0, 10, 8, 8, 9, 9, 0):
the task 1 gain did not transfer to those states (docs/report.md 9.6). The method families
investigated, their Intrinsic support and the experiments are in `docs/methods.md`. This is a
privileged-state result and is not comparable to vision-only policies.

## Layout
```
src/libero_intrinsic/      project code (adapter, skills, evaluation)
  env/        LIBERO wrapper, torch-free init-state loader, OSC trajectory executor
  model/      compiled-MuJoCo -> SDF converter (robot + scene), frame conventions
  intrinsic/  gRPC client on Intrinsic's protos, server process manager, world sync
  skills/     skill framework, pick/place, articulation (knob/drawer/door), predicate planner, grasp geometry
  eval/       episode runner (records + video), metrics/report generator
intrinsic_stack/cc/        C++ planner server linking Intrinsic Core (symlinked into the Bazel workspace)
scripts/                   proto generation, validation, demo, task runs, evaluation
tests/                     unit tests (frames, joint/action conversion, accounting, backend-unavailable)
third_party/               upstream clones (not committed): intrinsic-core, LIBERO, bazel-central-registry
```

## Setup (pinned)
Tested on Ubuntu 24.04, x86-64, CPU only (4 cores / 15 GiB). ~30 GB disk for the Bazel build.

```bash
# 1. upstream sources (pinned revisions in docs/contract.md)
git clone https://github.com/intrinsic-ai/intrinsic-core third_party/intrinsic-core && git -C third_party/intrinsic-core checkout c61bf075f2335371c6367b61117e8a62bb960c3b
git clone https://github.com/Lifelong-Robot-Learning/LIBERO third_party/LIBERO && git -C third_party/LIBERO checkout 8f1084e3132a39270c3a13ebe37270a43ece2a01
# 2. python env (LIBERO side)
uv venv --python 3.10 .venv && source .venv/bin/activate
uv pip install -r requirements.txt            # robosuite 1.4.0, mujoco 2.3.7, numpy 1.23.5, bddl 1.0.1, grpcio 1.74 ...
echo "$PWD/third_party/LIBERO" > .venv/lib/python3.10/site-packages/libero_src.pth
mkdir -p ~/.libero && python scripts/write_libero_config.py   # non-interactive ~/.libero/config.yaml
sudo apt-get install -y libegl1 libegl-mesa0 libgl1-mesa-dri libosmesa6   # CPU rendering (MUJOCO_GL=egl)
# 3. Intrinsic Core planner server (Bazel 8.8.1 via bazelisk; see docs/report.md for the network workarounds)
ln -sfn "$PWD/intrinsic_stack/cc" third_party/intrinsic-core/libero_bridge
(cd third_party/intrinsic-core && bazel --output_base=$HOME/bazel_out build --jobs=3 //libero_bridge:libero_planner_server)
# 4. python stubs for Intrinsic's protos
python scripts/gen_intrinsic_protos.py
python -m pytest -q
```

## Commands
```bash
python scripts/validate_kinematics.py --task 0                # FK/IK agreement Intrinsic vs MuJoCo
python scripts/demo_motion.py --task 0 --init 0               # one Intrinsic-planned motion executed in LIBERO
python scripts/run_task.py --task 0 --inits 0 1 2             # Task 0 on development init states
python scripts/run_task.py --task 3 --inits 25                # any task / init state
python scripts/evaluate.py --config configs/eval_frozen.yaml  # frozen evaluation suite (all tasks, eval init states)
python scripts/evaluate.py --tasks 0 1 --inits 0 1 --out runs/dev/x   # subset / dev states (reported as such)
python -m libero_intrinsic.eval.report runs/eval --out docs/results.json   # results table from saved episode records
python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/eval_<rev>/A ; python scripts/evaluate.py --tasks 5 6 7 8 9 --out runs/eval_<rev>/B   # repeated evaluation of a new candidate (same protocol)
python scripts/compare_evaluations.py evaluations/baseline_76267bf runs/eval_<rev> --out docs/comparison_<rev>.md   # per-task before/after
python scripts/archive_evaluation.py runs/eval_<rev> evaluations/candidate_<rev>   # commit the episode records (no videos)
```

## Evidence index (committed)
| what | where |
|---|---|
| 100 episode records per evaluated revision (success flag, steps, every Intrinsic request id, skill events, final goal atoms for 5e236de and fbabe38) | `evaluations/baseline_76267bf`, `evaluations/candidate_5b713c6`, `evaluations/heldout_5b713c6`, `evaluations/candidate_ce7685b`, `evaluations/candidate_9b02207`, `evaluations/candidate_5e236de`, `evaluations/candidate_fbabe38`, `evaluations/heldout_fbabe38` (manifest + records of the one-time states 30-39 run) |
| results tables and per-task comparisons | `docs/results*.md`, `docs/comparison_*.md`, `docs/heldout_5b713c6.md`, `docs/heldout_fbabe38.md` |
| videos + records + Intrinsic request logs for the recommended candidate fbabe38 (one success per solved task incl. task 9, failures for tasks 1, 3, 5, 7, 9) | `evidence/eval_episodes_fbabe38` (`MANIFEST.json` maps task -> episode dir) |
| same for the fallback candidates 5e236de and ce7685b; same for the held-out states 30-39 run of fbabe38 | `evidence/eval_episodes_5e236de`, `evidence/eval_episodes_ce7685b`, `evidence/heldout_episodes_fbabe38` |
| task 9 complete development successes with video and RPC log (C3: states 1, 3, 4; C4: states 0-4); task 3 stage-lab traces (incl. the third-pass pitched family); matched ce7685b comparisons; C4 regression summary | `evidence/dev_task9_success`, `evidence/dev_task9_success_c4`, `evidence/dev_task3_lab`, `evidence/dev_task3_lab_pitched`, `evidence/dev_matched_ce7685b`, `evidence/regression_c4` |
| partial/experimental protocol runs of this pass (labelled, not recommended) | `evaluations/partial_4c20556`, `evaluations/partial_c2` |
| videos + records for the later candidate's new failure modes (task 9 insertion, task 3 stages, task 1/6 regressions) | `evidence/eval_episodes_9b02207` |
| failure stages with evidence per run | `docs/failure_table.md` |
| method families, Intrinsic support, experiments, outcomes | `docs/methods.md` |

Videos of the other episodes exist only in the temporary `runs/` directory of the session
container and are not part of the repository.

## Final review sequence
```bash
# 0. setup as above (pinned sources, venv, Bazel build of the planner server, proto stubs), then:
python -m pytest -q
python scripts/validate_kinematics.py --task 0                                # Intrinsic FK/IK vs MuJoCo
python scripts/demo_motion.py --task 0 --init 0                               # one planned motion in LIBERO
git checkout fbabe38   # recommended candidate C4 (detached HEAD; `git checkout -` returns; HEAD's code is identical); 5e236de and ce7685b are the fallbacks
python scripts/evaluate.py --tasks 0 1 2 3 4 --out runs/review/A && python scripts/evaluate.py --tasks 5 6 7 8 9 --out runs/review/B   # ~2.5 h on 4 cores, run the halves in parallel
git checkout -
python -m libero_intrinsic.eval.report runs/review --out runs/review/results.json
python scripts/compare_evaluations.py evaluations/candidate_fbabe38 runs/review --labels archived_fbabe38 your_rerun
cat evidence/eval_episodes_fbabe38/MANIFEST.json                              # task -> video/record
python scripts/run_task.py --task 9 --inits 3                                 # task 9 complete episode on a development state
```

## Frozen dependency record (final pass)
| component | pinned value |
|---|---|
| Intrinsic Core | `intrinsic-ai/intrinsic-core` commit `c61bf075f2335371c6367b61117e8a62bb960c3b` (no tag; main of 2026-09-30), not updated during this work |
| Bazel | 8.8.1 (`third_party/intrinsic-core/.bazelversion`, via bazelisk); build target `//libero_bridge:libero_planner_server` (symlink of `intrinsic_stack/cc`) |
| Intrinsic APIs used | `intrinsic_proto.motion_planning.v1.MotionPlannerService` (ComputeFk, ComputeIk, CheckCollisions, PlanTrajectory), `intrinsic_proto.world.ObjectWorldService` (object poses, joints, reparenting); Python stubs generated by `scripts/gen_intrinsic_protos.py` into `build/intrinsic_py` |
| LIBERO | `Lifelong-Robot-Learning/LIBERO` commit `8f1084e3132a39270c3a13ebe37270a43ece2a01` |
| Python environment | Python 3.10, `requirements.txt` (robosuite 1.4.0, mujoco 2.3.7, numpy 1.23.5, bddl 1.0.1, grpcio) |
| Controller | robosuite OSC_POSE, LIBERO defaults, 20 Hz |

## Exact Intrinsic Core integration
See `docs/architecture.md` (table of modules, RPCs, inputs, outputs) and `docs/report.md`.
In one sentence: a C++ binary (`intrinsic_stack/cc/libero_planner_server.cc`) links Intrinsic's
`WorldFromSdf`, `FakeWorldService` and `MotionPlannerServiceInProcess` and serves
`intrinsic_proto.world.ObjectWorldService` and
`intrinsic_proto.motion_planning.v1.MotionPlannerService` over local gRPC; every FK, IK,
collision check and trajectory plan used by the controller is one of those RPCs, logged with an
id that appears in the episode record.
