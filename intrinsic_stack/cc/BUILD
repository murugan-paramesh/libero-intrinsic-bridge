# BUILD file for the libero-intrinsic-bridge planner server. This directory is symlinked into
# the intrinsic-core Bazel workspace as //libero_bridge so it can link the Intrinsic libraries.
load("//bazel:cc_macros.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libero_planner_server",
    testonly = True,  # world_service_fake and motion_planner_service_in_process are testonly
    srcs = ["libero_planner_server.cc"],
    deps = [
        "//intrinsic/world",
        "//intrinsic/world/conversion/sdf:world_from_sdf",
        "//intrinsic/world/proto:object_world_service_cc_grpc_proto",
        "//intrinsic/world/service/test:world_service_fake",
        "//intrinsic/util/status:status_macros",
        "//intrinsic_control/intrinsic/icon/release/portable:init_intrinsic",
        "//intrinsic_motion_planning/intrinsic/motion_planning/motion_planner:motion_planner_flags",
        "//intrinsic_motion_planning/intrinsic/motion_planning/service:motion_planner_service_in_process",
        "@abseil-cpp//absl/flags:flag",
        "@abseil-cpp//absl/log",
        "@abseil-cpp//absl/log:check",
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/synchronization",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_grpc_grpc//:grpc_security_base",
        "@or_tools//ortools/base:file",
    ],
)
