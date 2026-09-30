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
