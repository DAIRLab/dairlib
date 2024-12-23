#include <iostream>
#include "systems/controllers/id_mpc/constrained_dynamics_info.h"
#include "common/eigen_utils.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::Vector3d;

int DoMain() {
  std::string urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  ConstrainedDynamicsInfo dynamics_info(urdf);

  dynamics_info.AddContactPoint(
      "toe_left_front",
      "toe_left",
      Vector3d(-0.0457, 0.112, 0),
      {0, 1, 2},
      0.8
  );

  dynamics_info.AddDistanceConstraint(
      "thigh_left",
      Vector3d(0.0, 0.0, 0.045),
      "heel_spring_left",
      Vector3d(.11877, -.01, 0.0),
      0.5012
  );

  auto context_double = dynamics_info.MakeContext<double>();
  auto context_ad = dynamics_info.MakeContext<AutoDiffXd>();

  VectorXd q = VectorXd::Zero(dynamics_info.nq());

  q << 1, VectorXd::Zero(6), -0.084017,  -0.00120735, 0.366012, -0.6305,
      0.00205363, 0.838878, 0.205351, 0.084017,  0.00120735, 0.366012,
      .6305, 0.00205363,  0.838878, 0.205351;
  VectorXd v = VectorXd::Zero(dynamics_info.nv());

  VectorXd inputs = VectorXd::Zero(
      dynamics_info.nu() + dynamics_info.nh() + dynamics_info.nc()
  );
  VectorXd x = stack<double>({q, v});
  dynamics_info.SetPlantStateIfNew<double>(x, context_double.get());

  auto results_double = dynamics_info.Evaluate<double>(
      *context_double,
      inputs.segment(0, dynamics_info.nu()),
      inputs.segment(dynamics_info.nu(), dynamics_info.nh()),
      inputs.segment(dynamics_info.nu() + dynamics_info.nh(), dynamics_info.nc()), {"toe_left_front"});

  std::cout << results_double;

  VectorX<AutoDiffXd> vars_ad = drake::math::InitializeAutoDiff(
      stack<double>({q, v, inputs}));

  int nx = dynamics_info.nq() + dynamics_info.nv();

  dynamics_info.SetPlantStateIfNew<AutoDiffXd>(
      vars_ad.head(nx), context_ad.get());

  auto results_ad = dynamics_info.Evaluate<AutoDiffXd>(
      *context_ad,
      vars_ad.segment(nx, dynamics_info.nu()),
      vars_ad.segment(nx + dynamics_info.nu(), dynamics_info.nh()),
      vars_ad.segment(nx + dynamics_info.nu() + dynamics_info.nh(),
                      dynamics_info.nc()), {"toe_left_front"});

  std::cout << "\n" << results_ad;

  return 0;
}

}

int main(int argc, char**argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}