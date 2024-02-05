#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/jacktoy/systems/franka_kinematics_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class FrankaSimControls : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaSimControls(
      const drake::multibody::MultibodyPlant<double>& sim_plant,
      drake::systems::Context<double>* sim_context,
      drake::multibody::ModelInstanceIndex franka_index,
      drake::multibody::ModelInstanceIndex plate_index,
      drake::multibody::ModelInstanceIndex box_index,
      Eigen::VectorXd& q_franka_default, Eigen::VectorXd& q_plate_default,
      Eigen::VectorXd& q_box_default);

  const drake::systems::InputPort<double>& get_input_port_radio() const {
    return this->get_input_port(radio_port_);
  }

 private:
  void ResetSim(const drake::systems::Context<double>& context) const;

  drake::systems::EventStatus UpdateSimState(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex radio_port_;

  const drake::multibody::MultibodyPlant<double>& sim_plant_;
  drake::systems::Context<double>* sim_context_;
  drake::multibody::ModelInstanceIndex franka_index_;
  drake::multibody::ModelInstanceIndex plate_index_;
  drake::multibody::ModelInstanceIndex box_index_;

  Eigen::VectorXd q_franka_default_;
  Eigen::VectorXd q_plate_default_;
  Eigen::VectorXd q_box_default_;
};

}  // namespace systems
}  // namespace dairlib
