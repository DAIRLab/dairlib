#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "systems/controllers/operational_space_control/operational_space_control.h"

namespace dairlib {
namespace systems {
namespace controllers {

//TODO(yminchen): You can actually set the names of systems and connect them

// Requiremnt:
// The name of traj source block and its output port must have the same name as
// `TrackingData.name_`
void ConnectPortsForNonConstTraj(OperationalSpaceControl* osc,
                                 drake::systems::DiagramBuilder<double>& builder);

void AssignConstTrajToInputPorts(OperationalSpaceControl* osc,
                                 drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* diagram_context);


// Convert rotational matrix to Eular angle (roll pitch yaw)
void RotationalMatrixToRollPitchYaw(
  const Eigen::MatrixXd& mat, double* roll, double* pitch, double* yaw);

// Convert Quaternion to Eular angle (roll pitch yaw)
void QuaternionToRollPitchYaw(
  const Eigen::Quaterniond& q, double* roll, double* pitch, double* yaw);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
