#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "examples/cube_flip/parameter_headers/iC3_options.h"
#include "drake/systems/framework/leaf_system.h"

using dairlib::LcmTrajectory;
using dairlib::lcmt_timestamped_saved_traj;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace dairlib {

// Extracts final iteration from raw iC3 trajectories -> ee position/orientation/force
class iC3TrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit iC3TrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      iC3Options ic3_options);

  const drake::systems::InputPort<double>& get_input_port_nominal_trajectory() const {
    return this->get_input_port(nominal_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_iC3_x_trajectory() const {
    return this->get_input_port(ic3_x_trajectory_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_iC3_u_trajectory() const {
    return this->get_input_port(ic3_u_trajectory_port_);
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

  drake::systems::EventStatus SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex ic3_x_trajectory_port_;
  drake::systems::InputPortIndex ic3_u_trajectory_port_;
  drake::systems::InputPortIndex nominal_input_port_;

  drake::systems::OutputPortIndex actor_trajectory_port_;
  drake::systems::OutputPortIndex object_trajectory_port_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  iC3Options ic3_options_;

  bool publish_end_effector_orientation_ = true;

  int num_iters_;
  int N_;
  double dt_;

  double t0_idx_;
  mutable bool called_;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_u_;

};

}  // namespace dairlib
