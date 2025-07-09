#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_estimated_joint_friction_trifinger.hpp"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {
using drake::multibody::MultibodyPlant;
class JointFrictionCompensator : public drake::systems::LeafSystem<double> {
 public:
  JointFrictionCompensator(const MultibodyPlant<double>& plant,
                           drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_uncompensated_torque_input_port()
      const {
    return this->get_input_port(uncompensated_torque_input_port_);
  }
  const drake::systems::OutputPort<double>& get_compensated_torque_output_port()
      const {
    return this->get_output_port(compensated_torque_output_port_);
  }

  const drake::systems::OutputPort<double>& get_estimated_friction_torque_port()
      const {
    return this->get_output_port(estimated_friction_torque_port_);
  }

 private:
  void CopyOutCompensatedTorque(
      const drake::systems::Context<double>& context,
      systems::TimestampedVector<double>* output_torque) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CopyOutEstimatedFriction(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_estimated_joint_friction_trifinger* friction_torque) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  int n_q_;
  int n_v_;
  int n_u_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex uncompensated_torque_input_port_;
  drake::systems::OutputPortIndex compensated_torque_output_port_;
  drake::systems::OutputPortIndex estimated_friction_torque_port_;

  Eigen::VectorXd estimated_friction_torque_;
  drake::systems::DiscreteStateIndex estimated_friction_torque_idx_;
  drake::systems::DiscreteStateIndex compensated_torque_idx_;
  drake::systems::DiscreteStateIndex prev_timestamp_idx_;
};
}  // namespace dairlib::systems