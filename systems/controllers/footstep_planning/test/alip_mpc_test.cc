#include "systems/controllers/footstep_planning/alip_multiqp.h"
#include "systems/controllers/footstep_planning/alip_miqp.h"
#include "systems/controllers/footstep_planning/alip_s2s_mpfc.h"
#include "common/find_resource.h"
#include "drake/common/yaml/yaml_io.h"
#include <iostream>

namespace dairlib::systems::controllers {

using geometry::ConvexPolygon;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix4d;
using Eigen::MatrixXd;

struct mpc_profiling_data {
  double multiqp_runtime;
  double multiqp_solve_time;
  double miqp_runtime;
  double miqp_solve_time;
};

std::vector<ConvexPolygon> GetRandomFootholds(int n, double r) {
  std::vector<ConvexPolygon> footholds;
  for (int i = 0; i < n; i++) {
    Vector3d origin = Vector3d::Random();
    Vector3d normal = Vector3d::Random();
    normal(2) = 1;
    normal(0) *= std::min(1.0,  0.2 / abs(normal(0)));
    normal(1) *= std::min(1.0,  0.2 / abs(normal(1)));
    origin(2) *= std::min(1.0,  0.5 / abs(origin(2)));
    normal.normalize();
    auto foothold = ConvexPolygon();
    foothold.SetPlane(normal, origin);
    for (auto& j : {-1.0, 1.0}){
      for (auto& k: {-1.0, 1.0}) {
        foothold.AddFace(Vector3d(j, k, 0),
                         origin + Vector3d(r * j, r * k, 0));
      }
    }
    footholds.push_back(foothold);
  }
  return footholds;
}

mpc_profiling_data TestRandomFootholds(int n, double r) {
  auto trajopt_miqp = AlipMIQP(32, 0.85, 10, alip_utils::ResetDiscretization::kFOH, 3);
  auto trajopt_multiqp = AlipMultiQP(32, 0.85, 10, alip_utils::ResetDiscretization::kFOH, 3);


  auto test_gait = alip_utils::AlipGaitParams {
    0.85,
    32.0,
    0.3,
    0.1,
    0.3,
    Vector2d::UnitX(),
    alip_utils::Stance::kLeft,
    alip_utils::ResetDiscretization::kZOH
  };

  alip_s2s_mpfc_params params;
  params.gait_params = test_gait;
  params.nmodes = 3;
  params.tmin = 0.25;
  params.tmax = 0.35;
  params.soft_constraint_cost = 1000;
  params.com_pos_bound = Eigen::Vector2d::Ones();
  params.com_vel_bound = 2.0 * Eigen::Vector2d::Ones();
  params.Q = Eigen::Matrix4d::Identity();
  params.R = Eigen::Matrix3d::Identity();
  params.Qf = Eigen::Matrix4d::Identity();
  params.solver_options.SetOption(
      drake::solvers::GurobiSolver::id(),
      "Presolve",
      0
  );

  auto trajopt_s2s = AlipS2SMPFC(params);

  std::vector<AlipMPC*> trajopts = {&trajopt_miqp, &trajopt_multiqp};
  trajopt_miqp.SetDoubleSupportTime(0.1);
  trajopt_multiqp.SetDoubleSupportTime(0.1);
  auto xd = trajopt_miqp.MakeXdesTrajForVdes(Vector2d::UnitX(), 0.3, 0.3, 10);
  auto footholds = GetRandomFootholds(n, r);
  for (const auto trajopt: trajopts) {
    trajopt->AddFootholds(footholds);
    trajopt->UpdateNominalStanceTime(0.3, 0.3);
    trajopt->AddTrackingCost(xd, Matrix4d::Identity(), Matrix4d::Identity());
    trajopt->SetInputLimit(1);
    trajopt->AddInputCost(10);
    trajopt->Build();
  }
  mpc_profiling_data times{0, 0, 0, 0};
  auto p0 = Vector3d::Zero();
//  trajopt_miqp.CalcOptimalFootstepPlan(xd.front().head<4>(), p0);
//  trajopt_multiqp.CalcOptimalFootstepPlan(xd.front().head<4>(), p0);

  auto sol = trajopt_s2s.Solve(xd.front().head<4>(), p0, 0.3, test_gait.desired_velocity, test_gait.initial_stance_foot, footholds);

  times.miqp_runtime = sol.total_time;
  times.miqp_solve_time = sol.optimizer_time;
  return times;
}

std::vector<mpc_profiling_data> TestRandomFootholds(int n, double r, int trials) {
  std::vector<mpc_profiling_data> data(trials, {0,0,0,0});
  for (int i = 0; i < trials; i++) {
    data.at(i) = TestRandomFootholds(n, r);
  }
  return data;
}

int do_main(int argc, char* argv[]) {
  std::map<int, std::vector<mpc_profiling_data>> profile_data{};

  const int max_n = 10;
  for (int i = 1; i < max_n; i++) {
    profile_data[i] = TestRandomFootholds(i, 0.5, 20);
    std::cout << "\nTesting " << i + 1 << " footholds\n";
  }

  for (int i = 1; i < max_n; i++) {
    for (const auto& data : profile_data.at(i)) {
      std::cout << i << ", "
                << data.miqp_solve_time << ", "
                << data.miqp_runtime << std::endl;
    }
  }
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::do_main(argc, argv);
}