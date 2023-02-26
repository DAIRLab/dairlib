#include "systems/controllers/footstep_planning/alip_multiqp.h"
#include "systems/controllers/footstep_planning/alip_miqp.h"
#include "common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include "solvers/osqp_solver_options.h"
#include <iostream>

namespace dairlib::systems::controllers {

using geometry::ConvexFoothold;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

int do_main(int argc, char* argv[]) {
  std::vector<ConvexFoothold> footholds;

  Vector3d p0 = Vector3d::Zero();
  std::vector<Vector3d> origins = {
      p0,
      Vector3d(0.4, 0, 0),
      Vector3d(1, 1, 1),
      Vector3d(0.5, 0.1, 0),
      Vector3d(0.5, 0.1, 0),
      Vector3d(0.75, -0.1, 0),
      Vector3d(0.3, 0.1, 1.3),
      Vector3d(0.75, -0.1, 0)
  };
  for (auto& o: origins) {
    auto foothold = ConvexFoothold();
    foothold.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero());
    for (auto& i : {-1.0, 1.0}){
      for (auto& j: {-1.0, 1.0}) {
        foothold.AddFace(Vector3d(i, j, 0), o + Vector3d(0.1 * i, 0.1 * j, 0));
      }
    }
    footholds.push_back(foothold);
  }
  auto trajopt_miqp = AlipMIQP(32, 0.85, 10, alip_utils::ResetDiscretization::kFOH, 3);
  auto trajopt_multiqp = AlipMultiQP(32, 0.85, 10, alip_utils::ResetDiscretization::kFOH, 3);
  std::vector<AlipMPC*> trajopts = {&trajopt_miqp, &trajopt_multiqp};
  trajopt_miqp.SetDoubleSupportTime(0.1);
  trajopt_multiqp.SetDoubleSupportTime(0.1);
  auto xd = trajopt_miqp.MakeXdesTrajForVdes(Vector2d::UnitX(), 0.3, 0.3, 10);
  for (const auto trajopt: trajopts) {
    trajopt->AddFootholds(footholds);
    trajopt->AddTrackingCost(xd, Matrix4d::Identity(), Matrix4d::Identity());
    trajopt->UpdateNominalStanceTime(0.3, 0.3);
    trajopt->SetInputLimit(1);
    trajopt->AddInputCost(10);
    trajopt->Build();
  }

  auto start = std::chrono::high_resolution_clock::now();
  trajopt_miqp.CalcOptimalFootstepPlan(xd.front().head<4>(), p0);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "MIQP solve time: " << elapsed.count() << std::endl;

  start = std::chrono::high_resolution_clock::now();
  trajopt_multiqp.CalcOptimalFootstepPlan(xd.front().head<4>(), p0);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  std::cout << "MultiQP solve time: " << elapsed.count() << std::endl;
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::do_main(argc, argv);
}