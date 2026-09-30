#!/usr/bin/env python
"""Generate Python protobuf/gRPC stubs for the Intrinsic Core services we use.

Starting from a set of root .proto files, this resolves all transitive imports across the
intrinsic-core proto roots (intrinsic_apis, the repo root, intrinsic_sdk, intrinsic_motion_planning,
intrinsic_control, ..., googleapis) and runs grpc_tools.protoc once. Output goes to
build/intrinsic_py, a sys.path root providing the `intrinsic.*_pb2` modules.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
CORE = os.path.join(ROOT, "third_party", "intrinsic-core")
OUT = os.path.join(ROOT, "build", "intrinsic_py")

import grpc_tools  # noqa: E402

WKT = os.path.join(os.path.dirname(grpc_tools.__file__), "_proto")
GOOGLEAPIS = [p for p in [os.environ.get("GOOGLEAPIS_DIR", "/home/user/bazel_out/external/googleapis+"),
                          os.path.join(ROOT, "third_party", "googleapis")] if os.path.isdir(p)]
INCLUDE_ROOTS = [os.path.join(CORE, d) for d in [
    "intrinsic_apis", "", "intrinsic_sdk", "intrinsic_motion_planning", "intrinsic_control",
    "intrinsic_kinematics", "intrinsic_perception", "intrinsic_runtime", "intrinsic_hardware", "intrinsic_inference",
]] + GOOGLEAPIS + [WKT]
INCLUDE_ROOTS = [os.path.normpath(p) for p in INCLUDE_ROOTS if os.path.isdir(p)]

ROOTS = [
    "intrinsic/motion_planning/proto/v1/motion_planner_service.proto",
    "intrinsic/world/proto/object_world_service.proto",
    "intrinsic/world/proto/object_world_updates.proto",
    "intrinsic/motion_planning/proto/motion_target.proto",
    "intrinsic/logging/proto/context.proto",
]


def find(rel):
    for r in INCLUDE_ROOTS:
        p = os.path.join(r, rel)
        if os.path.isfile(p):
            return p
    raise FileNotFoundError(rel)


def main():
    seen, order, stack = set(), [], list(ROOTS)
    while stack:
        rel = stack.pop()
        if rel in seen:
            continue
        seen.add(rel)
        path = find(rel)
        order.append(rel)
        with open(path) as f:
            for m in re.finditer(r'^\s*import\s+(?:public\s+)?"([^"]+)"', f.read(), re.M):
                stack.append(m.group(1))
    todo = [r for r in order if not r.startswith("google/protobuf/")]
    os.makedirs(OUT, exist_ok=True)
    cmd = [sys.executable, "-m", "grpc_tools.protoc"] + [f"-I{r}" for r in INCLUDE_ROOTS] + [
        f"--python_out={OUT}", f"--grpc_python_out={OUT}"] + todo
    print(f"generating {len(todo)} protos into {OUT}")
    subprocess.check_call(cmd)
    with open(os.path.join(OUT, "PROTO_SOURCES.txt"), "w") as f:
        f.write("\n".join(sorted(todo)) + "\n")


if __name__ == "__main__":
    main()
