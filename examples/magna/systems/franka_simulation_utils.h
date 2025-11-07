#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/output_port.h>

#include "systems/framework/output_vector.h"

#include "drake/systems/lcm/lcm_interface_system.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

class ShunkCommandToTrajectory : public drake::systems::LeafSystem<double> {
 public:
  ShunkCommandToTrajectory();

  const drake::systems::InputPort<double>& get_input_port_schunk_command()
      const {
    return this->get_input_port(input_schunk_command_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_position_trajectory() const {
    return this->get_output_port(position_trajectory_port_);
  }

 private:
  void ToTrajectory(const drake::systems::Context<double>& context,
                    drake::trajectories::Trajectory<double>* output) const;
  drake::systems::InputPortIndex input_schunk_command_port_;
  drake::systems::OutputPortIndex position_trajectory_port_;
};

class StatusToRobotOutput : public drake::systems::LeafSystem<double> {
 public:
  StatusToRobotOutput(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::multibody::ModelInstanceIndex model_instance);

  const drake::systems::InputPort<double>& get_input_port_position() const {
    return this->get_input_port(input_position_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_velocity() const {
    return this->get_input_port(input_velocity_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_effort() const {
    return this->get_input_port(input_effort_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_robot_output()
      const {
    return this->get_output_port(output_robot_output_port_);
  }

 private:
  void ToRobotOutput(const drake::systems::Context<double>& context,
                     dairlib::systems::OutputVector<double>* output) const;

  drake::systems::InputPortIndex input_position_port_;
  drake::systems::InputPortIndex input_velocity_port_;
  drake::systems::InputPortIndex input_effort_port_;
  drake::systems::OutputPortIndex output_robot_output_port_;
};

class GravityCompensator : public drake::systems::LeafSystem<double> {
 public:
  GravityCompensator(const drake::multibody::MultibodyPlant<double>& plant,
                     drake::systems::Context<double>& context);
  const drake::systems::InputPort<double>& get_input_port_actuation() const {
    return this->get_input_port(actuation_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::OutputPort<double>&
  get_output_port_compensated_actuation() const {
    return this->get_output_port(compensated_actuation_port_);
  }

 private:
  void AddGravityCompensation(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;
  // constructor variables
  drake::systems::InputPortIndex actuation_port_;
  drake::systems::InputPortIndex state_port_;
  drake::systems::OutputPortIndex compensated_actuation_port_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>& context_;
  int num_actuators_;
};

const drake::systems::OutputPort<double>& SimulatePandaHand(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& hand_mbplant,
    drake::systems::Context<double>* hand_mbplant_context,
    drake::systems::lcm::LcmInterfaceSystem* lcm,
    const drake::systems::OutputPort<double>& gripper_state_input_port,
    std::string osc_qp_settings_file);

}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib