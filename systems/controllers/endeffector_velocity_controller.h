#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"

#include <math.h>


namespace dairlib{
namespace systems{

// KukaIiwaVelocityController extends LeafSystem, which is basically
// a 'block' like you might see in simulink with inputs and outputs.
// In this case, KukaIiwaVelocityController will take in desired velocities,
// q, q_dot, and output an appropriate torque
// \Tau = jacobian.transpose x K x desired_accelerations
 class EndEffectorVelocityController : public drake::systems::LeafSystem<double> {
  public:
    // Constructor
    EndEffectorVelocityController(const drake::multibody::MultibodyPlant<double>& plant,
                                  std::string ee_frame_name,
                                  Eigen::Vector3d ee_contact_frame,
                                  double k_d, double k_r);

    // Getter methods for each of the individual input/output ports.
    const drake::systems::InputPort<double>& get_joint_pos_input_port() const {
      return this->get_input_port(joint_position_measured_port_);
    }
    const drake::systems::InputPort<double>& get_joint_vel_input_port() const {
      return this->get_input_port(joint_velocity_measured_port_);
    }
    const drake::systems::InputPort<double>& get_endpoint_twist_input_port() const {
      return this->get_input_port(endpoint_twist_commanded_port_);
    }
    const drake::systems::OutputPort<double>& get_endpoint_torque_output_port() const{
      return this->get_output_port(endpoint_torque_output_port_);
    }

  private:
    // The callback called when declaring the output port of the system.
    // The 'output' vector is set in place and then passed out.
    // Think a simulink system.
    void CalcOutputTorques(const drake::systems::Context<double>& context,
                         drake::systems::BasicVector<double>* output) const;

    const drake::multibody::MultibodyPlant<double>& plant_;
    int num_joints_;
    const drake::multibody::Frame<double>& ee_joint_frame_;
    Eigen::Vector3d ee_contact_frame_;
    int joint_position_measured_port_;
    int joint_velocity_measured_port_;
    int endpoint_twist_commanded_port_;
    int endpoint_torque_output_port_;
    double k_d_;
    double k_r_;
};


} // namespace systems
} // namespace dairlib
