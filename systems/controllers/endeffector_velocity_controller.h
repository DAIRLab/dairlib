#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/manipulation/util/sim_diagram_builder.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

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
    EndEffectorVelocityController(std::unique_ptr<RigidBodyTree<double>>& tree,
                                  Eigen::Isometry3d eeCFIsometry,
                                  int num_joints, int k_d, int k_r);

    // Getter methods for each of the individual input/output ports.
    const drake::systems::InputPort<double>& get_joint_pos_input_port() const {
      return this->get_input_port(joint_position_measured_port);
    }
    const drake::systems::InputPort<double>& get_joint_vel_input_port() const {
      return this->get_input_port(joint_velocity_measured_port);
    }
    const drake::systems::InputPort<double>& get_endpoint_twist_input_port() const {
      return this->get_input_port(endpoint_twist_commanded_port);
    }
    const drake::systems::OutputPort<double>& get_endpoint_torque_output_port() const{
      return this->get_output_port(endpoint_torque_output_port);
    }

  private:
    // The callback called when declaring the output port of the system.
    // The 'output' vector is set in place and then passed out.
    // Think a simulink system.
    void CalcOutputTorques(const Context<double>& context,
                         BasicVector<double>* output) const;

    Eigen::Vector3d eeContactFrame;
    Eigen::Translation3d eeContactFrameTranslation;
    Eigen::Isometry3d eeCFIsometry;

    std::unique_ptr<RigidBodyTree<double>>& tree;
    int joint_position_measured_port;
    int joint_velocity_measured_port;
    int endpoint_twist_commanded_port;
    int endpoint_torque_output_port;
    int num_joints;
    int k_d;
    int k_r;
};


} // namespace systems
} // namespace dairlib
