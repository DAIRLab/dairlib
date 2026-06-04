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
#include "drake/systems/framework/leaf_system.h"
#include "examples/iC3/iC3_options.h"
#include "systems/framework/timestamped_vector.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Outputs a lcmt_timestamped_saved_traj
class LqrTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  // example idx 0: plate
  // example idx 1: trifinger 180
  explicit LqrTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant, c3::multibody::LCSFactory lcs_factory, iC3Options ic3_options, int example_idx,
        MatrixXd A_x, VectorXd lb_x, VectorXd ub_x, MatrixXd A_u, VectorXd lb_u, VectorXd ub_u);

  const drake::systems::InputPort<double>& get_input_port_actor_input() const {
    return this->get_input_port(actor_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_x_curr() const {
    return this->get_input_port(x_curr_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_tracking_target() const {
    return this->get_input_port(tracking_target_port_);
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

  drake::systems::InputPortIndex actor_input_port_;
  drake::systems::InputPortIndex nominal_position_port_;
  drake::systems::InputPortIndex tracking_target_port_;
  drake::systems::InputPortIndex x_curr_port_;

  drake::systems::OutputPortIndex actor_trajectory_port_;
  drake::systems::OutputPortIndex object_trajectory_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  iC3Options ic3_options_;

  mutable c3::multibody::LCSFactory lcs_factory_;

  bool publish_end_effector_orientation_ = false;

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
};

}  // namespace dairlib
