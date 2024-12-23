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

  const auto& plant = dynamics_info.get_plant();
  const auto& plant_ad = dynamics_info.get_plant_ad();

  VectorXd q = VectorXd::Zero(plant.num_positions());

  q << 1, VectorXd::Zero(6), -0.084017,  -0.00120735, 0.366012, -0.6305,
      0.00205363, 0.838878, 0.205351, 0.084017,  0.00120735, 0.366012,
      .6305, 0.00205363,  0.838878, 0.205351;

  VectorXd v = VectorXd::Zero(plant.num_velocities());
  VectorXd inputs = VectorXd::Zero(
      dynamics_info.nu() + dynamics_info.nh() + dynamics_info.nc()
  );

  plant.SetPositions(context_double.get(), q);
  plant.SetVelocities(context_double.get(), v);

  auto results_double = dynamics_info.Evaluate<double>(
      *context_double,
      inputs.segment(0, dynamics_info.nu()),
      inputs.segment(dynamics_info.nu(), dynamics_info.nh()),
      inputs.segment(dynamics_info.nu() + dynamics_info.nh(), dynamics_info.nc()), {"toe_left_front"});

  std::cout << results_double;

  VectorX<AutoDiffXd> vars_ad = drake::math::InitializeAutoDiff(
      stack<double>({q, v, inputs}));

  plant_ad.SetPositions(context_ad.get(), vars_ad.head(dynamics_info.nq()));
  plant_ad.SetVelocities(
      context_ad.get(),
      vars_ad.segment(dynamics_info.nq(), dynamics_info.nv()));

  int nx = dynamics_info.nq() + dynamics_info.nv();
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