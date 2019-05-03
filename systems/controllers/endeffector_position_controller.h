#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/manipulation/util/sim_diagram_builder.h"

#include <iostream>
#include <fstream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using drake::systems::LeafSystem;
using drake::systems::Context;

namespace dairlib{
namespace systems{

// PD Controller for end effector position.
// Takes in desired end effector position and orientation,
// orientation as quaternion, position as 3-vector.
// outputs appropriate velocity (twist) as 6-vector: first three indices
// are desired angular velocity, next three are desired linear velocity.

class EndEffectorPositionController : public LeafSystem<double> {
	public:
		 // Constructor
		 EndEffectorPositionController(const RigidBodyTree<double>& tree, int ee_frame_id,
                             Eigen::Vector3d ee_contact_frame, int num_joints,
                             int k_p, int k_omega);

   // Getter methods for each of the individual input/output ports.
   const drake::systems::InputPort<double>& get_joint_pos_input_port() const {
     return this->get_input_port(joint_position_measured_port);
   }
   const drake::systems::InputPort<double>& get_endpoint_pos_input_port() const {
     return this->get_input_port(endpoint_position_commanded_port);
   }
   const drake::systems::InputPort<double>& get_endpoint_orient_input_port() const {
     return this->get_input_port(endpoint_orientation_commanded_port);
   }
   const drake::systems::OutputPort<double>& get_endpoint_cmd_output_port() const {
     return this->get_output_port(endpoint_position_cmd_output_port);
   }

	private:
 		// Callback method called when declaring output port of the system.
   // Twist combines linear and angular velocities.
   void CalcOutputTwist(const Context<double> &context,
                        BasicVector<double>* output) const;

 		Eigen::Vector3d ee_contact_frame;
 		int joint_position_measured_port;
 		int endpoint_position_commanded_port;
   int endpoint_orientation_commanded_port;
   int endpoint_position_cmd_output_port;
 		const RigidBodyTree<double>& tree_local;
   int ee_frame_id;
   int k_p;
   int k_omega;

};

}
}
