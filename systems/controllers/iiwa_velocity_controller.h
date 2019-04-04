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

// Declaring the class KukaIiwaVelocityController even though the
// file is called "iiwa_velocity_controller"
// KukaIiwaVelocityController extends LeafSystem, which is basically
// a 'block' like you might see in simulink with inputs and outputs.
// In this case, KukaIiwaVelocityController will take in desired velocities,
// q, q_dot, and output an appropriate torque.
class KukaIiwaVelocityController : public LeafSystem<double> {
  public:
    // Constructor
    KukaIiwaVelocityController();

  private:
    // The callback called when declaring the output port of the system.
    // The 'output' vector is set in place and then passed out.
    // 'Hur hur hur we're TRI and we don't use return values, we use
    // input and output ports instead'
    void CalcOutputTorques(const Context<double>& context,
                       BasicVector<double>* output) const;

    Eigen::Vector3d eeContactFrame;
    Eigen::Translation3d eeContactFrameTranslation;
    Eigen::Isometry3d eeCFIsometry;

    std::unique_ptr<RigidBodyTree<double>> tree;
    int joint_position_measured_port;
    int joint_velocity_measured_port;
    int endpoint_velocity_commanded_port;
    int endpoint_angular_velocity_commanded_port;
    double k;


};


} //systems
} //dairlib
