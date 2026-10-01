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
| candidate `ce7685b` (current) | **76/100** (CI 67-83%) | 10, 10, 10, 0, 10, 8, 9, 9, 10, 0 |

Per-task before/after with the episodes that changed: `docs/comparison_ce7685b.md`,
`docs/comparison_5b713c6_to_ce7685b.md`. Every motion executed was an Intrinsic
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

## Exact Intrinsic Core integration
See `docs/architecture.md` (table of modules, RPCs, inputs, outputs) and `docs/report.md`.
In one sentence: a C++ binary (`intrinsic_stack/cc/libero_planner_server.cc`) links Intrinsic's
`WorldFromSdf`, `FakeWorldService` and `MotionPlannerServiceInProcess` and serves
`intrinsic_proto.world.ObjectWorldService` and
`intrinsic_proto.motion_planning.v1.MotionPlannerService` over local gRPC; every FK, IK,
collision check and trajectory plan used by the controller is one of those RPCs, logged with an
id that appears in the episode record.
