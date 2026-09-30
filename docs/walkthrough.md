# Demonstration walkthrough (and how to explain the system)

## 10-minute demo
```bash
source .venv/bin/activate && export MUJOCO_GL=egl
# 1. The robot model agrees with LIBERO: Intrinsic FK/IK vs MuJoCo over random configurations
python scripts/validate_kinematics.py --task 0          # -> runs/dev/kinematics_validation.json
# 2. One Intrinsic-planned motion executed by the benchmark controller (video + request/response protos)
python scripts/demo_motion.py --task 0 --init 0         # -> runs/dev/demo_motion/{demo_motion.mp4,summary.json,intrinsic_protos/}
# 3. Task 0 end to end on a development init state (video + episode record + RPC log)
python scripts/run_task.py --task 0 --inits 0           # -> runs/dev/task0/episodes/<run_id>/
# 4. The results table from saved records
python -m libero_intrinsic.eval.report runs/eval        # -> docs/results.md is the committed copy
```
What to look at in an episode directory: `episode.json` (goal atoms from BDDL, skill outcomes,
every executed trajectory id, tracking errors, executed-path collision audits, official success),
`intrinsic_requests.jsonl` (one line per Intrinsic RPC with id, latency, status), `agentview.mp4`.

## How to explain it (talking points)
1. **Where Intrinsic is used.** LIBERO gives us a MuJoCo scene; we convert the compiled model to
   SDF and load it into Intrinsic's world (`WorldFromSdf`). Intrinsic's world service holds the
   scene (we push object poses before every plan, re-parent grasped objects to the flange frame)
   and Intrinsic's motion-planner service computes FK, IK, collision checks and collision-free,
   time-parameterized trajectories (RRT-Connect + shortcutting + linear Cartesian planner + TOPP
   in the OSS build). Our C++ file only wires those services into one process because the
   production k3s runtime cannot run here; the algorithms are untouched.
2. **Why we trust the model.** 200 random joint configurations: FK error 5e-9 m; IK round trip
   through the LIBERO robot 1e-6 m; the collision checker flags an arm-in-table configuration
   that MuJoCo also reports as contact.
3. **How a plan becomes benchmark actions.** LIBERO only accepts Cartesian deltas at 20 Hz. We
   sample the Intrinsic joint trajectory, map each sample to a TCP pose with Intrinsic FK, and
   command the delta toward it; the trajectory is slowed until the OSC controller can follow it
   (2-5 mm error). Because OSC controls only the TCP, we audit the configurations actually reached
   with Intrinsic `CheckCollisions` after every motion.
4. **Classical skills, one library.** Goal atoms are read from the BDDL; predicates map to
   Pick/Place/TurnKnob/Push. Grasps are sampled on the object's own collision boxes and checked
   geometrically (pad/finger/palm zones), then by Intrinsic IK with collision settings that name
   the intentional contacts (target object, its support, fingers). Placement is object-relative
   (region slots, free-spot search, container rules).
5. **Honest evaluation.** Dev init states 0-4 were used for development; evaluation is on states
   10-19, one episode each, budget 1200 steps, recovery counted, no re-runs, official
   `check_success()`. Infrastructure errors are counted separately from task failures.
6. **What does not work yet** (see docs/report.md Section 8): closing the drawer after placing
   the bowl (arm posture vs the wine rack), inserting the mug into the microwave (a horizontal
   grasp near table height is not reachable/collision-free from this base pose), and the book
   insertion occasionally releases off-target after the wrist rotation.
