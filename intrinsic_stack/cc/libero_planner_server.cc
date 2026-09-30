// Copyright 2026 libero-intrinsic-bridge authors. Apache-2.0.
//
// libero_planner_server: the smallest genuine Intrinsic Core planning deployment
// that runs on a workstation without the k3s runtime.
//
// It links the *unmodified* Intrinsic Core libraries and serves them over gRPC:
//   * intrinsic::sdf::WorldFromSdf            -- loads our generated SDF worlds
//   * intrinsic::FakeWorldService             -- hosts ObjectWorldService + WorldService
//                                                (world state, transforms, reparenting,
//                                                collision queries) on local TCP
//   * intrinsic::MotionPlannerServiceInProcess -- hosts MotionPlannerService (ComputeFk,
//                                                ComputeIk, PlanTrajectory, PlanPath,
//                                                CheckCollisions) on local TCP, backed by
//                                                the same world service.
//
// Nothing about kinematics, IK, collision checking or path planning is implemented here;
// this file is only process wiring. The Python side talks to the two services with the
// protos from intrinsic_apis / intrinsic-core.

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_split.h"
#include "absl/synchronization/notification.h"
#include "grpc/grpc.h"
#include "grpc/grpc_security_constants.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "intrinsic/icon/release/portable/init_intrinsic.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner_flags.h"
#include "intrinsic/motion_planning/service/motion_planner_service_in_process.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/conversion/sdf/world_from_sdf.h"
#include "intrinsic/world/service/test/world_service_fake.h"
#include "intrinsic/world/world.h"
#include "ortools/base/file.h"
#include "ortools/base/options.h"

ABSL_FLAG(std::vector<std::string>, worlds, {},
          "Comma separated list of <world_id>=<path/to/world.sdf> entries to load at "
          "startup.");
ABSL_FLAG(std::string, address_file, "",
          "If set, the two service addresses are written to this file as JSON.");
ABSL_FLAG(bool, concurrent_collision_checking, false,
          "Enable Intrinsic's multithreaded edge validation during path planning.");
ABSL_FLAG(int, collision_threads, 2, "Threads for concurrent collision checking.");

namespace intrinsic {
namespace {

absl::StatusOr<World> LoadWorldFromSdfFile(const std::string& path) {
  std::string sdf_text;
  INTR_RETURN_IF_ERROR(file::GetContents(path, &sdf_text, file::Defaults()));
  sdf::WorldFromSdf world_from_sdf;
  INTR_RETURN_IF_ERROR(world_from_sdf.Parse(sdf_text));
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<World> world, world_from_sdf.GetWorld());
  return std::move(*world);
}

absl::Status MainImpl() {
  FakeWorldService::CreateOptions options;
  options.enable_local_tcp = true;
  INTR_ASSIGN_OR_RETURN(std::unique_ptr<FakeWorldService> world_service,
                        FakeWorldService::Create(options));

  for (const std::string& entry : absl::GetFlag(FLAGS_worlds)) {
    std::pair<std::string, std::string> id_and_path = absl::StrSplit(entry, absl::MaxSplits('=', 1));
    if (id_and_path.first.empty() || id_and_path.second.empty()) {
      return absl::InvalidArgumentError(absl::StrCat("bad --worlds entry: ", entry));
    }
    INTR_ASSIGN_OR_RETURN(World world, LoadWorldFromSdfFile(id_and_path.second));
    INTR_RETURN_IF_ERROR(world_service->AddWorld(id_and_path.first, std::move(world)));
    LOG(INFO) << "Loaded world '" << id_and_path.first << "' from " << id_and_path.second;
  }

  MotionPlannerFlags flags;
  flags.enable_concurrent_collision_checking = absl::GetFlag(FLAGS_concurrent_collision_checking);
  flags.concurrent_collision_checking_thread_count = absl::GetFlag(FLAGS_collision_threads);

  // Same wiring as MotionPlannerServiceInProcess::Create(FakeWorldService*), but with our
  // planner flags.
  INTR_ASSIGN_OR_RETURN(std::string world_address, world_service->GetAddress());
  auto channel = ::grpc::CreateChannel(world_address, grpc::experimental::LocalCredentials(LOCAL_TCP));
  auto object_world_stub = intrinsic_proto::world::ObjectWorldService::NewStub(channel);
  std::unique_ptr<MotionPlannerServiceInProcess> planner = MotionPlannerServiceInProcess::Create(
      /*world_service=*/nullptr, object_world_stub.get(), world_service->GetGeometryLibrary(), flags);

  const std::string planner_address = planner->GetAddress();
  std::string json = absl::StrCat("{\"world_service\": \"", world_address, "\", \"motion_planner_service\": \"",
                                  planner_address, "\", \"credentials\": \"local_tcp\"}");
  std::printf("%s\n", json.c_str());
  std::fflush(stdout);
  if (const std::string f = absl::GetFlag(FLAGS_address_file); !f.empty()) {
    std::ofstream out(f);
    out << json << std::endl;
  }
  LOG(INFO) << "ObjectWorldService at " << world_address << ", MotionPlannerService at " << planner_address;

  absl::Notification never;
  never.WaitForNotification();  // serve until killed
  return absl::OkStatus();
}

}  // namespace
}  // namespace intrinsic

int main(int argc, char** argv) {
  InitIntrinsic(argv[0], argc, argv);
  QCHECK_OK(intrinsic::MainImpl());
  return 0;
}
