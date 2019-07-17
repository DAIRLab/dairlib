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

/// Users can call ConnectPortsForNonConstTraj() if they don't want to connect
/// the input/output ports by themselves. However, there is one requirement:
/// the name of traj source block and its output port must have the same name as
/// `TrackingData.name_` (so that ConnectPortsForNonConstTraj() knows which
/// block and port should be connected to.)
void ConnectPortsForNonConstTraj(OperationalSpaceControl* osc,
                                 drake::systems::DiagramBuilder<double>& builder);

/// AssignConstTrajToInputPorts() assigns fixed values to the input ports which
/// corresponds to the constant desired trajectories.
void AssignConstTrajToInputPorts(OperationalSpaceControl* osc,
                                 drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* diagram_context);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
