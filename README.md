# libero-intrinsic-bridge

Classical (non-learned) manipulation for the LIBERO-10 benchmark whose forward/inverse
kinematics, collision checking and motion planning are computed by **Intrinsic Core**
(`intrinsic-ai/intrinsic-core`), executed in the unmodified LIBERO/MuJoCo environment.

Status, evidence and honest limitations: see `docs/report.md` (technical report),
`docs/results.md` (generated results table), `docs/contract.md` (facts verified from the pinned
sources), `docs/architecture.md`, `CHECKPOINT.md` (work log).

## Headline result
Frozen evaluation (100 episodes, 10 official init states per task disjoint from development states): **62/100 successes** (Wilson 95% CI 52-71%); per task 10, 5, 10, 0, 10, 2, 9, 8, 8, 0 of 10. Every motion executed was an Intrinsic `PlanTrajectory` result (1,580 plans, mean 30 ms, no timeouts). Tasks 3 and 9 are not solved (see docs/report.md 8.2). This is a privileged-state result and is not comparable to vision-only policies.

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
```

## Exact Intrinsic Core integration
See `docs/architecture.md` (table of modules, RPCs, inputs, outputs) and `docs/report.md`.
In one sentence: a C++ binary (`intrinsic_stack/cc/libero_planner_server.cc`) links Intrinsic's
`WorldFromSdf`, `FakeWorldService` and `MotionPlannerServiceInProcess` and serves
`intrinsic_proto.world.ObjectWorldService` and
`intrinsic_proto.motion_planning.v1.MotionPlannerService` over local gRPC; every FK, IK,
collision check and trajectory plan used by the controller is one of those RPCs, logged with an
id that appears in the episode record.
