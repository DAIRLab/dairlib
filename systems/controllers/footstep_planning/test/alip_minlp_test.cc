#include "systems/controllers/footstep_planning/alip_multiqp.h"
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
      Vector3d(0.5, 0.1, 0),
//      Vector3d(0.75, -0.1, 0)
  };
  for (auto& o: origins) {
    auto foothold = ConvexFoothold();
    foothold.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero());
    for (auto& i : {-1.0, 1.0}){
      for (auto& j: {-1.0, 1.0}) {
        foothold.AddFace(Vector3d(i, j, 0), o + Vector3d(0.5 * i, 0.5 * j, 0));
      }
    }
    footholds.push_back(foothold);
  }
  auto trajopt = AlipMultiQP(32, 0.85, 10, alip_utils::ResetDiscretization::kFOH, 3);
  trajopt.AddFootholds(footholds);
  auto xd = trajopt.MakeXdesTrajForVdes(Vector2d::UnitX(), 0.1, 0.35, 10);
  trajopt.AddTrackingCost(xd, Matrix4d::Identity(), Matrix4d::Identity());
  trajopt.UpdateNominalStanceTime(0.35, 0.35);
  trajopt.SetInputLimit(1);
  trajopt.AddInputCost(10);

  const auto& planner_solver_options =
      drake::yaml::LoadYamlFile<solvers::DairOsqpSolverOptions>(
          FindResourceOrThrow(
              "examples/perceptive_locomotion/gains/osqp_options_planner.yaml"
          ));

  trajopt.Build(planner_solver_options.osqp_options);

  auto start = std::chrono::high_resolution_clock::now();
  trajopt.CalcOptimalFootstepPlan(xd.front().head<4>(), p0);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;

  start = std::chrono::high_resolution_clock::now();
  trajopt.CalcOptimalFootstepPlan(xd.front().head<4>(), p0, true);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::do_main(argc, argv);
}