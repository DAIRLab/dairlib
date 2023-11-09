#pragma once

#include <string>
#include <vector>

#include <drake/common/yaml/yaml_io.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3_options.h"
#include "solvers/lcs.h"
#include "solvers/solver_options_io.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class C3TrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  explicit C3TrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, C3Options c3_options);

  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor_trajectory()
      const {
    return this->get_output_port(actor_trajectory_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_object_trajectory()
      const {
    return this->get_output_port(object_trajectory_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_force_trajectory()
      const {
    return this->get_output_port(force_trajectory_port_);
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

  void OutputForceTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex c3_solution_port_;
  drake::systems::OutputPortIndex actor_trajectory_port_;
  drake::systems::OutputPortIndex object_trajectory_port_;
  drake::systems::OutputPortIndex force_trajectory_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  C3Options c3_options_;

  bool publish_end_effector_orientation_ = false;

  // convenience for variable sizes
  int n_q_;
  int n_v_;
  int n_x_;
  int n_lambda_;
  int n_u_;

  drake::systems::DiscreteStateIndex plan_start_time_index_;
  int N_;
};

}  // namespace systems
}  // namespace dairlib
