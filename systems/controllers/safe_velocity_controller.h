#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/discrete_values.h"

namespace dairlib{
namespace systems{

 class SafeVelocityController : public drake::systems::LeafSystem<double> {
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
   void CalcOutputTorques(const drake::systems::Context<double> &context,
                          drake::systems::BasicVector<double>* output) const;

   drake::systems::EventStatus CheckTerminate(
           const drake::systems::Context<double> &context,
           drake::systems::DiscreteValues<double>* next_state) const;


   int joint_torques_input_port_;
   int joint_velocities_input_port_;
   int joint_torques_output_port_;

   double max_velocity_;
};
}
}
