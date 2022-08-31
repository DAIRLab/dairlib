#include "systems/controllers/footstep_planning/alip_minlp.h"

namespace dairlib::systems::controllers {

using geometry::ConvexFoothold;

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
      Vector3d(0.75, -0.1, 0)
  };
  for (auto& o: origins) {
    auto foothold = ConvexFoothold();
    foothold.SetContactPlane(Vector3d::UnitZ(), Vector3d::Zero());
    for (auto& i : {-1.0, 1.0}){
      for (auto& j: {-1.0, 1.0}) {
        foothold.AddFace(Vector3d(i, j, 0), o + Vector3d(0.05 * i, 0.05 * j, 0));
      }
    }
    footholds.push_back(foothold);
  }
  auto trajopt = AlipMINLP(32, 0.85);
  trajopt.AddFootholds(footholds);
  trajopt.AddMode(10);
  trajopt.AddMode(10);
  trajopt.AddMode(10);
  auto xd = trajopt.MakeXdesTrajForVdes(1.0, 0.1, 0.35, 10);
  trajopt.AddTrackingCost(xd, Matrix4d::Identity());
  trajopt.AddInputCost(10);
  trajopt.Build();
  auto start = std::chrono::high_resolution_clock::now();
  trajopt.CalcOptimalFootstepPlan(xd.front().front(), p0);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::do_main(argc, argv);
}