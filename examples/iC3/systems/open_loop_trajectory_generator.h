#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>

#include "c3/core/lcs.h"
#include "c3/multibody/lcs_factory.h"
#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "examples/iC3/iC3_options.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::LcmTrajectory;
using dairlib::lcmt_timestamped_saved_traj;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace dairlib {

// Extracts final iteration from raw iC3 trajectories -> ee position/orientation/force
class OpenLoopTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit OpenLoopTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant, iC3Options ic3_options, 
      c3::multibody::LCSFactory lcs_factory, bool track_dynamically_feasible, int example_idx);

  const drake::systems::InputPort<double>& get_input_port_x_curr() const {
    return this->get_input_port(x_curr_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_x() const {
    return this->get_input_port(ic3_x_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_ic3_u() const {
    return this->get_input_port(ic3_u_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_timestep() const {
    return this->get_input_port(timestep_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_nominal_position() const {
    return this->get_input_port(nominal_position_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor_trajectory()
      const {
    return this->get_output_port(actor_trajectory_port_);
  }

 private:
  void OutputActorTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  std::tuple<MatrixXd, MatrixXd> SimulateLCS(VectorXd x0, MatrixXd u_hat) const;

  drake::systems::InputPortIndex x_curr_port_;
  drake::systems::InputPortIndex ic3_x_port_;
  drake::systems::InputPortIndex ic3_u_port_;
  drake::systems::InputPortIndex timestep_port_;
  drake::systems::InputPortIndex nominal_position_port_;

  drake::systems::OutputPortIndex actor_trajectory_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  mutable c3::multibody::LCSFactory lcs_factory_;

  iC3Options ic3_options_;

  int iter_to_use_;
  int N_;
  double dt_;
  int example_idx_;

  mutable drake::math::RigidTransform<double> X_delta_;
 
  bool track_dynamically_feasible_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_u_;

};

}  // namespace dairlib
