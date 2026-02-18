#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

using solvers::LCS;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/// Outputs a lcmt_timestamped_saved_traj
class C3TrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit C3TrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      C3Options c3_options, bool track_dynamically_feasible);

  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_lcs() const {
    return this->get_input_port(lcs_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor_trajectory()
      const {
    return this->get_output_port(actor_trajectory_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_object_trajectory()
      const {
    return this->get_output_port(object_trajectory_port_);
  }

  void SetPublishEndEffectorOrientation(bool publish_end_effector_orientation) {
    publish_end_effector_orientation_ = publish_end_effector_orientation;
  }

 private:
  void OutputActorTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  void OutputObjectTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  MatrixXd SimulateLCS(VectorXd x0, MatrixXd u_hat, LCS lcs) const;

  LCS CreatePlaceholderLCS() const;

  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::InputPortIndex nominal_position_port_;
  drake::systems::InputPortIndex lcs_port_;
  drake::systems::OutputPortIndex actor_trajectory_port_;
  drake::systems::OutputPortIndex object_trajectory_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  C3Options c3_options_;

  bool publish_end_effector_orientation_ = false;
  bool track_dynamically_feasible_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  int N_;
};

}  // namespace dairlib
