#pragma once
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib {
namespace systems {

/*!
 * Provides a state-based radio command (horizontal and yaw position) for
 * repeatable simulation experiments.
 */
class CassieRadioOperator : public drake::systems::LeafSystem<double> {
 public:
  CassieRadioOperator(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::systems::Context<double>* context);

  [[nodiscard]] const drake::systems::InputPort<double>& get_input_port_state()
  const {
    return get_input_port(input_port_state_);
  }

  [[nodiscard]] const drake::systems::InputPort<double>&
  get_input_port_target_xy() const {
    return get_input_port(input_port_target_xy_);
  }

  [[nodiscard]] const drake::systems::OutputPort<double>&
  get_output_port_radio() const {
    return get_output_port(output_port_radio_);
  }

  [[nodiscard]] const drake::systems::OutputPort<double>&
  get_output_port_horizontal_velocity() const {
    return get_output_port(output_port_horizontal_velocity_);
  }

 private:

  void CopyRadio(const drake::systems::Context<double>& context,
                 drake::systems::BasicVector<double>* radio) const;

  void CopyVdes(const drake::systems::Context<double>& context,
                drake::systems::BasicVector<double>* vdes_xy) const;

  drake::systems::EventStatus DiscreteUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_values) const;

  drake::systems::InputPortIndex input_port_state_;
  drake::systems::InputPortIndex input_port_target_xy_;
  drake::systems::OutputPortIndex output_port_radio_;
  drake::systems::OutputPortIndex output_port_horizontal_velocity_;

  drake::systems::DiscreteStateIndex vdes_state_index_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
};

}
}