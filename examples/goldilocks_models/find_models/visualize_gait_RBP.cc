#include <memory>
#include <chrono>

#include <gflags/gflags.h>

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"

#include "drake/common/trajectories/piecewise_polynomial.h"

#include "systems/goldilocks_models/file_utils.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
using drake::MatrixX;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;

namespace dairlib {

void visualizeGait() {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/goldilocks_models/PlanarWalkerWithTorso.urdf",
      drake::multibody::joints::kFixed, &tree);

  // int n = tree.get_num_positions();
  // int nu = tree.get_num_actuators();


  // Create a testing piecewise polynomial
  std::vector<double> T_breakpoint = {0, 2};
  std::vector<MatrixXd> Y(T_breakpoint.size(), MatrixXd::Zero(14, 1));

  MatrixXd x0 = MatrixXd::Zero(14, 1);
  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  MatrixXd x1 = MatrixXd::Zero(14, 1);
  x1 << 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Y[0] = x0;
  Y[1] = x1;

  PiecewisePolynomial<double> pp_traj =
      PiecewisePolynomial<double>::FirstOrderHold(T_breakpoint, Y);

  // visualizer
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  auto state_source = builder.AddSystem<drake::systems::TrajectorySource>(
                        pp_traj);
  auto publisher = builder.AddSystem<drake::systems::DrakeVisualizer>(tree,
                   &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_traj.end_time());
  }

  return;
}
} // dairlib


int main() {

  dairlib::visualizeGait();

  return 0;
}

