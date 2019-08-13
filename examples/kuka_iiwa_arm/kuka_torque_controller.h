#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/state_feedback_controller_interface.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace systems {

/**
 * Controller that take emulates the kuka_iiwa_arm when operated in torque
 * control mode. The controller specifies a stiffness and damping ratio at each
 * of the joints. Because the critical damping constant is a function of the
 * configuration the damping is non-linear. See
 * https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/kuka-driver/sunrise_1.11/DrakeFRITorqueDriver.java
 *
 * for details on the low-level controller. Note that the
 * input_port_desired_state() method takes a full state for convenient wiring
 * with other Systems, but ignores the velocity component.
 */

using drake::VectorX;
using drake::multibody::MultibodyPlant;

template <typename T>
class KukaTorqueController
    : public drake::systems::Diagram<T>,
      public drake::systems::controllers::StateFeedbackControllerInterface<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(KukaTorqueController)
  KukaTorqueController(std::unique_ptr<drake::multibody::MultibodyPlant<T>> plant,
                       const VectorX<double>& stiffness,
                       const VectorX<double>& damping);

  const drake::systems::InputPort<T>& get_input_port_commanded_torque() const {
    return drake::systems::Diagram<T>::get_input_port(
        input_port_index_commanded_torque_);
  }

  const drake::systems::InputPort<T>& get_input_port_estimated_state() const override {
    return drake::systems::Diagram<T>::get_input_port(
        input_port_index_estimated_state_);
  }

  const drake::systems::InputPort<T>& get_input_port_desired_state() const override {
    return drake::systems::Diagram<T>::get_input_port(input_port_index_desired_state_);
  }

  const drake::systems::OutputPort<T>& get_output_port_control() const override {
    return drake::systems::Diagram<T>::get_output_port(output_port_index_control_);
  }

 private:
  void SetUp(const VectorX<double>& stiffness,
             const VectorX<double>& damping_ratio);
  std::unique_ptr<MultibodyPlant<T>> robot_for_control_{nullptr};
  int input_port_index_estimated_state_{-1};
  int input_port_index_desired_state_{-1};
  int input_port_index_commanded_torque_{-1};
  int output_port_index_control_{-1};
};

}  // namespace systems
}  // namespace dair
