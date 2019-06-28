#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/manipulation/util/sim_diagram_builder.h"

using Eigen::VectorXd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems{

class SafeVelocityController : public LeafSystem<double> {
 public:
   SafeVelocityController(double max_velocity, int num_joints);

   const drake::systems::InputPort<double>& get_joint_torques_input_port() const {
     return this->get_input_port(joint_torques_input_port_);
   }
   const drake::systems::InputPort<double>& get_joint_velocities_input_port() const {
     return this->get_input_port(joint_velocities_input_port_);
   }
   const drake::systems::OutputPort<double>& get_joint_torques_output_port() const {
     return this->get_output_port(joint_torques_output_port_);
   }

 private:
   void CalcOutputTorques(const Context<double> &context,
                          BasicVector<double>* output) const;

   drake::systems::EventStatus CheckTerminate(
           const Context<double> &context,
           drake::systems::DiscreteValues<double>* next_state) const;


   int joint_torques_input_port_;
   int joint_velocities_input_port_;
   int joint_torques_output_port_;

   double max_velocity_;
};
}
}
