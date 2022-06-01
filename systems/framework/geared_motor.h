#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Class to model the relationship between motor speed and torque. This uses
/// the basic torque speed curve and is implemented identically to the motor
/// model in cassie-mujoco-sim provided by Agility Robotics
class GearedMotor final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GearedMotor)

  explicit GearedMotor(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::unordered_map<std::string, double>& max_motor_speeds);

  const drake::systems::InputPort<double>& get_input_port_command() const {
    return drake::systems::LeafSystem<double>::get_input_port(
        command_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return drake::systems::LeafSystem<double>::get_input_port(
        state_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port() const {
    return drake::systems::LeafSystem<double>::get_output_port(0);
  }

 protected:
  void CalcTorqueOutput(const drake::systems::Context<double>& context,
                        systems::BasicVector<double>* output) const;

 private:
  bool is_abstract() const { return false; }

  const int n_q;
  const int n_v;
  const int n_u;
  const Eigen::MatrixXd B_;
  int command_input_port_;
  int state_input_port_;

  std::vector<double> actuator_gear_ratios;
  std::vector<double> actuator_ranges;
  std::vector<double> actuator_max_speeds;
};

}  // namespace systems
}  // namespace dairlib
