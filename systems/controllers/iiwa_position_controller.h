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

class KukaIiwaPositionController : public LeafSystem<double> {
	public:
		// Constructor
		KukaIiwaPositionController();

	private:
		// Callback method called when declaring output port of the system.
		void CalcOutputVelocity(const Context<double> &context,
                          BasicVector<double>* output) const;

		void CalcOutputAngularVelocity(const Context<double> &context,
                                 BasicVector<double>* output) const;

		Eigen::Vector3d eeContactFrame;
		int joint_position_measured_port;
		int endpoint_position_commanded_port;
		std::unique_ptr<RigidBodyTree<double>> tree;

};

}
}
