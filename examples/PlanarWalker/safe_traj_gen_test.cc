#include "examples/PlanarWalker/safe_traj_gen.h"
#include "matplotlibcpp.h"

#include <math.h>
#include <string>

#include "drake/solvers/solve.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/primitives/discrete_time_delay.h"

#include "attic/multibody/rigidbody_utils.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"

#include "examples/PlanarWalker/safe_traj_gen.h"
#include "examples/PlanarWalker/state_based_fsm.h"
#include "systems/controllers/osc/operational_space_control.h"

namespace plt = matplotlibcpp;

namespace dairlib {

int DoMain() {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "examples/PlanarWalker/PlanarWalkerWithTorsoAndFeet.urdf",
      drake::multibody::joints::kFixed, &tree);

  const double terrain_size = 100;
  const double terrain_depth = 0.20;
  drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);
  LIPMSwingLeg<double> lipm_model(9.81, 0.75, 5.5, 0.05);
  LoadLyapunovPolynomial polynomial_loader("examples/PlanarWalker/csv/V_M.csv",
                                           "examples/PlanarWalker/csv/V_p.csv");

  int torso_idx = multibody::GetBodyIndexFromName(tree, "torso_mass");
  int left_foot_idx = multibody::GetBodyIndexFromName(tree, "left_foot");
  int right_foot_idx = multibody::GetBodyIndexFromName(tree, "right_foot");
  DRAKE_DEMAND(torso_idx != -1 && left_foot_idx != -1 && right_foot_idx != -1);

  Eigen::Vector3d pt_on_left_foot = Eigen::Vector3d::Zero();
  Eigen::Vector3d pt_on_right_foot = Eigen::Vector3d::Zero();

  double mid_foot_height = 0.1 + 0.05;
  double desired_final_foot_height = 0.005; // 0.05
  double desired_final_vertical_foot_velocity = 0;
  SafeTrajGenerator safe_traj_gen(
      tree, lipm_model, polynomial_loader, left_foot_idx, pt_on_left_foot,
      right_foot_idx, pt_on_right_foot, mid_foot_height,
      desired_final_foot_height, desired_final_vertical_foot_velocity, false);

  Eigen::Vector3d reduced_order_state, x_dot;
  reduced_order_state << 0.065, 0, 0;

  std::vector<double> time_hist_;
  std::vector<double> CoM_hist_x_;

  double current_time = 0.0;
  double dt = 0.01;
  double prev_stepping_time = 0.0;
  double x_stance = 0;
  while(current_time <= 1.0) {
    x_dot = safe_traj_gen.solveQP(reduced_order_state);
    reduced_order_state += x_dot * dt;
    // std::cout << x_dot.transpose() << std::endl;
    std::cout << reduced_order_state.transpose() << std::endl;
    std::cout << "______________________" << std::endl;

    time_hist_.push_back(current_time);
    CoM_hist_x_.push_back(reduced_order_state(0));

    if (safe_traj_gen.should_step(current_time, prev_stepping_time,
                                  reduced_order_state) ==
        SteppingResults::step) {
      std::cout << "Stepping!" << std::endl;
      x_stance += reduced_order_state(2);
      reduced_order_state = lipm_model.reset(current_time, reduced_order_state,
                                             Eigen::Vector2d::Zero());
      prev_stepping_time = current_time;
    }

    if(reduced_order_state.transpose() * reduced_order_state > 2) {
      std::cout << "Failed to balance!" << std::endl;
      break;
    } else if(reduced_order_state.transpose() * reduced_order_state < 0.001){
      std::cout << reduced_order_state << std::endl;
      std::cout << "Could balance!" << std::endl;
      break;
    }

    plt::figure(1);
    plt::named_plot("Center of Mass x", time_hist_, CoM_hist_x_);
    plt::legend();
    plt::pause(0.01);

    current_time += dt;
  }
  plt::pause(10);

  return 0;
}
}  // namespace dairlib

int main() {
  dairlib::DoMain();
  return 0;
}
