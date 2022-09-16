#include "systems/controllers/footstep_planning/alip_minlp.h"

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
  auto trajopt = AlipMINLP(32, 0.85);
  trajopt.AddFootholds(footholds);
  trajopt.AddMode(10);
  trajopt.AddMode(10);
  trajopt.AddMode(10);
  auto xd = trajopt.MakeXdesTrajForVdes(Vector2d::UnitX(), 0.1, 0.35, 10);
  trajopt.AddTrackingCost(xd, Matrix4d::Identity(), Matrix4d::Identity());
  trajopt.UpdateNominalStanceTime(0.35, 0.35);
  trajopt.AddInputCost(10);
  trajopt.Build();

  auto start = std::chrono::high_resolution_clock::now();
  trajopt.CalcOptimalFootstepPlan(xd.front().front(), p0);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;

  start = std::chrono::high_resolution_clock::now();
  trajopt.CalcOptimalFootstepPlan(xd.front().front(), p0, true);
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;




  // Check Copy Constuctor
  AlipMINLP trajopt2(trajopt);

  // Check Copy Assignemnt
  AlipMINLP trajopt3(0, 0);
  trajopt3 = trajopt;
  std::cout << "check accessing the solution: " << trajopt3.GetStateSolution().at(0).at(0).transpose() << "\n";
  trajopt2.CalcOptimalFootstepPlan(xd.front().front(), p0);
  trajopt3.CalcOptimalFootstepPlan(xd.front().front(), p0);

  int i = 1; int j = 3;
  std::cout << "random x check: \n" <<
      trajopt.GetStateSolution().at(i).at(j).transpose() << "\n" <<
      trajopt2.GetStateSolution().at(i).at(j).transpose() << "\n" <<
      trajopt3.GetStateSolution().at(i).at(j).transpose() << std::endl;

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::systems::controllers::do_main(argc, argv);
}