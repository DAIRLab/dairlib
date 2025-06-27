#pragma once

#include <c3/systems/c3_controller_options.h>

using namespace c3::systems;

namespace dairlib {
namespace examples {
namespace plate_balancing {
// Options for the C3 controller used in the plate balancing example.
struct PlateBalancingC3ControllerOptions : public C3ControllerOptions {
  std::vector<double> u_horizontal_limits;
  std::vector<double> u_vertical_limits;
  std::vector<Eigen::VectorXd> workspace_limits;
  double workspace_margins;

  template <typename Archive>
  void Serialize(Archive* a) {
    C3ControllerOptions::Serialize(a);
    a->Visit(DRAKE_NVP(workspace_limits));
    a->Visit(DRAKE_NVP(u_horizontal_limits));
    a->Visit(DRAKE_NVP(u_vertical_limits));
    a->Visit(DRAKE_NVP(workspace_margins));
  }
};
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib