# Failure table (baseline 76267bf, 100 episodes, evaluations/baseline_76267bf)

Stage labels: SEQ task sequencing/goal, FRAME frame/world sync, REACH unreachable target or
posture, IK IK branch/joint limit, PLAN collision/planning, GRASP alignment/contact/slip,
TRACK tracking/controller, PLACE placement/release/retreat, ARTIC articulation, INFRA timeout/runtime.
"Observed" = read from logs/RPC errors; "Hypothesis" = inferred.

| task | failures | stage | observed evidence | hypothesis / fix candidate |
|---|---|---|---|---|
| 1 | 5/10 | PLAN (pre-grasp IK collision) | ComputeIk collisions: hand vs table (2: tilted grasp dips below the table plane), finger/hand vs basket (2), finger vs milk carton (1) | coarse point-cloud clearance misses thin overlaps -> exact support-plane corner test; container walls need finer sampling |
| 3 | 10/10 | REACH (push pre-contact) | 30/30 ComputeIk errors: robot link5/link6 vs wine_rack when reaching the drawer front horizontally | the wine rack (33 cm tall) occupies the corridor of a frontal push -> push from above with the hand tilted away from the cabinet |
| 5 | 8/10 | GRASP slip + PLACE (4), INFRA-adapter (3), REACH (1) | release 16-20 cm off target after the 90 deg fit rotation with lift drift ~0 (slip during rotation, stale tcp_t_obj); 3x UpdateObjectJoints OUT_OF_RANGE (joint 7 0.3 mrad past limit); 1x no IK | re-measure attachment after transport (slip compensation); clamp synced joints |
| 6 | 1/10 | PLACE (lowering IK) | PlanTrajectory: carried mug vs table at the lowering target | stale attachment offset (mug slipped down) -> slip compensation incl. height |
| 7 | 2/10 | TRACK/IK | pre-grasp executions with joint error 1.2-1.5 rad, joint-limit violation (>1 mrad), 18 deg orientation error on approach | large reconfigurations tracked poorly by OSC; limit tolerance too strict (MuJoCo limits are soft) |
| 8 | 2/10 | GRASP slip + PLACE | second pot: lift drift 2.3 cm, released 2 cm off, resting on the first pot (z 1.003 vs 0.997) | slip compensation |
| 9 | 10/10 | PLAN/REACH (side-grasp IK) | ComputeIk collisions: link5/hand vs the OPEN microwave door (21), link6 vs porcelain mug (6), unreachable (3); elbow-up JointPositionLimits constraints: still 0 solutions | the open door stands between robot and mug at hand height; needs an approach from +/-y with a steep forearm, or a different manipulation strategy |
