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

/// AssignConstTrajToInputPorts() assigns fixed values to the input ports which
/// corresponds to the constant desired trajectories.
void AssignConstTrajToInputPorts(OperationalSpaceControl* osc,
                                 drake::systems::Diagram<double>* diagram,
                                 drake::systems::Context<double>* diagram_context);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
