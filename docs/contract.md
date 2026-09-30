# Technical contract (verified from pinned sources)

All statements below were verified by reading the pinned sources or by running them in this
workspace. Paths are relative to `third_party/`.

Pinned revisions
- `intrinsic-core` @ `c61bf075f2335371c6367b61117e8a62bb960c3b` (main, 2026-09-30)
- `LIBERO` @ `8f1084e3132a39270c3a13ebe37270a43ece2a01` (master, 2025-03-15)
- robosuite 1.4.0, mujoco 2.3.7 (pip), bddl 1.0.1, numpy 1.23.5, Python 3.10.18

## LIBERO-10

Task list: `LIBERO/libero/libero/benchmark/libero_suite_task_map.py` (`libero_task_map["libero_10"]`).
Ordering: `benchmark/__init__.py` `task_orders[0] = [0..9]` (identity) is the default. Language
strings are derived from the file name (`grab_language_from_filename`), not read from BDDL; we
re-implement that function (`src/libero_intrinsic/env/libero_env.py`) because the upstream
benchmark module imports torch at import time.

| # | task | goal (BDDL, verbatim) |
|---|------|------|
| 0 | LIVING_ROOM_SCENE2_put_both_the_alphabet_soup_and_the_tomato_sauce_in_the_basket | `(And (In alphabet_soup_1 basket_1_contain_region) (In tomato_sauce_1 basket_1_contain_region))` |
| 1 | LIVING_ROOM_SCENE2_put_both_the_cream_cheese_box_and_the_butter_in_the_basket | `(And (In cream_cheese_1 basket_1_contain_region) (In butter_1 basket_1_contain_region))` |
| 2 | KITCHEN_SCENE3_turn_on_the_stove_and_put_the_moka_pot_on_it | `(And (Turnon flat_stove_1) (On moka_pot_1 flat_stove_1_cook_region))` |
| 3 | KITCHEN_SCENE4_put_the_black_bowl_in_the_bottom_drawer_of_the_cabinet_and_close_it | `(And (Close white_cabinet_1_bottom_region) (In akita_black_bowl_1 white_cabinet_1_bottom_region))` |
| 4 | LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate | `(And (On porcelain_mug_1 plate_1) (On white_yellow_mug_1 plate_2))` |
| 5 | STUDY_SCENE1_pick_up_the_book_and_place_it_in_the_back_compartment_of_the_caddy | `(And (In black_book_1 desk_caddy_1_back_contain_region))` |
| 6 | LIVING_ROOM_SCENE6_put_the_white_mug_on_the_plate_and_put_the_chocolate_pudding_to_the_right_of_the_plate | `(And (On porcelain_mug_1 plate_1) (On chocolate_pudding_1 living_room_table_plate_right_region))` |
| 7 | LIVING_ROOM_SCENE1_put_both_the_alphabet_soup_and_the_cream_cheese_box_in_the_basket | `(And (In alphabet_soup_1 basket_1_contain_region) (In cream_cheese_1 basket_1_contain_region))` |
| 8 | KITCHEN_SCENE8_put_both_moka_pots_on_the_stove | `(And (On moka_pot_1 flat_stove_1_cook_region) (On moka_pot_2 flat_stove_1_cook_region) (Turnon flat_stove_1))` |
| 9 | KITCHEN_SCENE6_put_the_yellow_and_white_mug_in_the_microwave_and_close_it | `(And (In white_yellow_mug_1 microwave_1_heating_region) (Close microwave_1))` |

Articulated objects actually used: `flat_stove` (knob hinge; tasks 2, 8 - in task 8 the stove
is already on in `:init`), `white_cabinet` (slide drawer, task 3, bottom drawer starts open),
`microwave` (door hinge, task 9, starts open). No other appliance interactions exist.

Predicates (`LIBERO/libero/libero/envs/predicates/base_predicates.py`,
`envs/object_states/base_object_states.py`, `envs/objects/site_object.py`):
- `In(obj, region)`: region site `check_contain` = object body origin inside the site box
  (lower z relaxed by 0.01); `check_contact` of a site is always True.
- `On(obj, other_body)`: `obj_z <= other_z` **and** contact(obj, other) **and** xy distance < 0.03.
- `On(obj, region_site)`: `size_z-0.005 < dz < size_z+0.10`, |dxy| < size_xy, and contact with the
  site's parent object if any (table target zones have no parent).
- `Turnon(flat_stove)`: knob joint qpos >= 0.5 (range [-0.005, 2.1]).
- `Close(white_cabinet_1_bottom_region)`: drawer joint qpos > 0.0 (range [-0.16, 0.01], open < -0.14).
- `Close(microwave_1)`: door joint qpos > -0.005 (range [-2.094, 0], open < -1.3).

Success: `<Problem>._check_success` = conjunction of all goal atoms; `BDDLBaseDomain.step`
returns `done = self._check_success()`, so **`done` means goal satisfied at that step, not
timeout**. robosuite's own horizon (`ControlEnv` default 1000) raises on further steps; we set
`horizon` large and enforce our own episode budget.

Init states: `init_files/libero_10/<task>.pruned_init`, 50 states/task, rows =
flattened `MjSimState [time, qpos, qvel]`; zip-format torch pickles of numpy arrays (loaded
torch-free by `src/libero_intrinsic/env/init_states.py`). Official reset recipe
(`libero/lifelong/metric.py`): `env.reset()`, `env.set_init_state(s)`, 5 zero-action steps.

## Robot, controller, action, observations

- Robot: robosuite Panda MJCF (`robosuite/models/assets/robots/panda/robot.xml`) subclassed as
  `MountedPanda` (kitchen/study, RethinkMount) or `OnTheGroundPanda` (living room); gripper
  `PandaGripper` (`grippers/panda_gripper.xml`). No URDF anywhere in LIBERO.
- Joint order `robot0_joint1..7`, limits (rad) from the compiled model:
  `[-2.8973,2.8973] [-1.7628,1.7628] [-2.8973,2.8973] [-3.0718,-0.0698] [-2.8973,2.8973] [-0.0175,3.7525] [-2.8973,2.8973]`.
- Base pose (world): living room `(-0.51, 0, 0.42)`, kitchen `(-0.66, 0, 0.912)`, study
  `(-0.75, 0, 0.912)` (identity orientation); read at runtime from body `robot0_base`.
- Controller: `OSC_POSE` (`robosuite/controllers/config/osc_pose.json`): control_freq 20 Hz,
  kp 150, damping ratio 1, input [-1,1] -> output +-0.05 m and +-0.5 rad per step, `uncouple_pos_ori`
  true, `control_delta` true, no interpolator, no position/orientation limits.
  Action (7): `a[0:3]` delta position of `gripper0_grip_site` (world), `a[3:6]` delta rotation
  axis-angle pre-multiplied in the **world** frame (`goal_ori = R(delta) @ ee_ori`), `a[6]` gripper:
  only `sign` is used, -1 open / +1 close, fingers move 0.01 per sim substep (25 substeps/step).
- TCP: site `gripper0_grip_site` (body `gripper0_eef`, 0.097 m along z of `gripper0_right_gripper`,
  which is rotated -90 deg about z from `robot0_right_hand`). `robot0_eef_pos` is this site, but
  `robot0_eef_quat` is the `right_hand` body (xyzw). We read the site frame directly from MuJoCo.
- Observations: `robot0_joint_pos`, `robot0_gripper_qpos`, `<obj>_pos`, `<obj>_quat` (xyzw),
  `agentview_image`, `robot0_eye_in_hand_image` (vertically flipped; LIBERO flips with `[::-1]`
  when saving video). Raw MuJoCo `body_xquat` is wxyz.
- Rendering: robosuite forces `MUJOCO_GL=egl` on Linux; Mesa EGL works on CPU here.

## Object geometry
Collision geometry of LIBERO objects is a set of primitive **box** geoms (group 0); visual meshes
are non-colliding. Robot arm links and the gripper hand use **mesh** collision geoms; finger pads
are boxes. The floor is a plane. (Verified on the compiled Task 0 model: geom types {plane, box, mesh}.)

## Intrinsic Core
- Build: Bazel 8.8.1 (bzlmod), LLVM toolchain, C++20. No pip package, no Python bindings for
  kinematics/planning; the Python SDK only talks gRPC.
- Robot models: SDF via `intrinsic/world/conversion/sdf/world_from_sdf.cc` (URDF only through an
  sdformat `model://` include). No Franka/Panda model exists in the tree (grep: none).
  Custom tags: `<intrinsic:ik_solver tip_link_name=...>kinematic_chain</intrinsic:ik_solver>`,
  `<limit><intrinsic:acceleration>`, `<intrinsic:jerk>` (`intrinsic_sdk/intrinsic/scene/sdf/custom_tags.h`).
  Mesh URIs `bypass://<absolute path>` resolve to that path (`scene/sdf/sdf_path_resolver.cc`).
  Supported collision shapes: box, cylinder, sphere, capsule, ellipsoid, mesh (convex for
  collision); plane is rejected (`scene/sdf/sdf_util.cc` ParseGeometry).
- Kinematics: `intrinsic_kinematics/intrinsic/kinematics/` (Skeleton, State FK/Jacobian,
  `ik/` with registered solver `kinematic_chain` = numeric chain IK).
- Collision: `intrinsic/world/collision/coal_collision_checker.{h,cc}` (COAL/hpp-fcl).
- Planning: `intrinsic_motion_planning/intrinsic/motion_planning/motion_planner/motion_planner.{h,cc}`
  (RRT-Connect + shortcutting + TOPP-style parameterization; OSS build uses
  `AccelerationLimitedTrajectoryParameterizer`).
- Service: `intrinsic_proto.motion_planning.v1.MotionPlannerService` with `ComputeFk`, `ComputeIk`,
  `PlanTrajectory`, `PlanPath`, `CheckCollisions` (proto:
  `intrinsic_motion_planning/intrinsic/motion_planning/proto/v1/motion_planner_service.proto`).
  World: `intrinsic_proto.world.ObjectWorldService` (`intrinsic/world/proto/object_world_service.proto`):
  `GetTransform`, `UpdateTransform`, `ReparentObject`, `UpdateObjectJoints`, `CheckCollisions`, ...
- Local deployment used here: `intrinsic::MotionPlannerServiceInProcess`
  (`motion_planning/service/motion_planner_service_in_process.{h,cc}`) + `intrinsic::FakeWorldService`
  (`intrinsic/world/service/test/world_service_fake.{h,cc}`, `enable_local_tcp=true`). Both are
  official intrinsic-core targets (testonly) that run the full services in one process with gRPC
  `LocalCredentials(LOCAL_TCP)`. The production deployment (k3s cluster + release tarball,
  Ubuntu 26.04) is not possible in this container (no docker daemon, no k3s).
