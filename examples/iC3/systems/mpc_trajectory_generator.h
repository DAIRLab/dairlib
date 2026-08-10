#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "c3/core/lcs.h"
#include "c3/systems/c3_controller_options.h"
#include "c3/multibody/lcs_factory_options.h"
#include "c3/multibody/geom_geom_collider.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/scene_graph.h"
#include "examples/iC3/iC3_options.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Outputs a lcmt_timestamped_saved_traj
class MPCTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  // example idx 0: plate
  // example idx 1: trifinger 180
  explicit MPCTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, drake::systems::Context<double>* plant_context,
    c3::multibody::LCSFactory lcs_factory, iC3Options ic3_options, c3::LCSFactoryOptions lcs_factory_options, 
    bool track_dynamically_feasible, int example_idx,
    MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u);

  const drake::systems::InputPort<double>& get_input_port_solution() const {
    return this->get_input_port(solution_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_x_lcs() const {
    return this->get_input_port(x_lcs_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor_trajectory()
      const {
    return this->get_output_port(actor_trajectory_port_);
  }

  void SetPublishEndEffectorOrientation(bool publish_end_effector_orientation) {
    publish_end_effector_orientation_ = publish_end_effector_orientation;
  }

 private:
  void OutputActorTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  // Returns xhat, uhat
  std::tuple<MatrixXd, MatrixXd> SimulateLCS(VectorXd x0, MatrixXd u_hat, 
      const drake::systems::Context<double>& context) const;

  c3::LCS CreatePlaceholderLCS() const;

  drake::systems::InputPortIndex solution_port_;
  drake::systems::InputPortIndex nominal_position_port_;
  drake::systems::InputPortIndex x_lcs_port_;

  drake::systems::OutputPortIndex actor_trajectory_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* plant_context_;

  c3::LCSFactoryOptions lcs_factory_options_;
  iC3Options ic3_options_;

  mutable c3::multibody::LCSFactory lcs_factory_;

  bool publish_end_effector_orientation_ = false;
  bool track_dynamically_feasible_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  mutable MatrixXd A_x_;
  mutable VectorXd lb_x_;
  mutable VectorXd ub_x_;
  mutable MatrixXd A_u_;
  mutable VectorXd lb_u_;
  mutable VectorXd ub_u_;

  int example_idx_;
  int N_;
};

}  // namespace dairlib
