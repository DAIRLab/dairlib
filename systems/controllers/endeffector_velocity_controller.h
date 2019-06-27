#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;

namespace dairlib{
namespace systems{

// KukaIiwaVelocityController extends LeafSystem, which is basically
// a 'block' like you might see in simulink with inputs and outputs.
// In this case, KukaIiwaVelocityController will take in desired velocities,
// q, q_dot, and output an appropriate torque
// \Tau = jacobian.transpose x K x desired_accelerations
class EndEffectorVelocityController : public LeafSystem<double> {
  public:
    // Constructor
    EndEffectorVelocityController(const MultibodyPlant<double>& plant,
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
    void CalcOutputTorques(const Context<double>& context,
                         BasicVector<double>* output) const;

    const MultibodyPlant<double>& plant_;
    int num_joints_;
    const Frame<double>& ee_joint_frame_;
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
