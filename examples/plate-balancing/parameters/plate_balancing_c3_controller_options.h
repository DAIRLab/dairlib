#pragma once

#include <c3/systems/c3_controller_options.h>

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief C3 controller options for the plate balancing example.
 *
 * Extends C3ControllerOptions with additional actuator and workspace limits.
 */
struct PlateBalancingC3ControllerOptions
    : public c3::systems::C3ControllerOptions {
  std::vector<double>
      u_horizontal_limits;  ///< Limits for horizontal actuator inputs.
  std::vector<double>
      u_vertical_limits;  ///< Limits for vertical actuator inputs.
  std::vector<Eigen::VectorXd>
      workspace_limits;      ///< Workspace boundaries as vectors.
  double workspace_margins;  ///< Margins to be maintained within the workspace.

  template <typename Archive>
  void Serialize(Archive* a) {
    c3::systems::C3ControllerOptions::Serialize(a);
    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(workspace_limits));
    a->Visit(DRAKE_NVP(workspace_margins));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib