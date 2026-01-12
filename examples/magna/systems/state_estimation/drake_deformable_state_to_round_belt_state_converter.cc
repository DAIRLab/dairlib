#include "examples/magna/systems/state_estimation/drake_deformable_state_to_round_belt_state_converter.h"

#include <iostream>

#include <drake/lcmt_viewer_link_data.hpp>

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

DrakeDeformableStateToRoundBeltStateConverter::
    DrakeDeformableStateToRoundBeltStateConverter() {
  deformable_state_input_port_ =
      this->DeclareAbstractInputPort(
              "deformable_state", drake::Value<drake::lcmt_viewer_link_data>{})
          .get_index();
  round_belt_state_output_port_ =
      this->DeclareAbstractOutputPort(
              "round_belt_state",
              &DrakeDeformableStateToRoundBeltStateConverter::
                  CopyRoundBeltStateToOutput)
          .get_index();
}

void DrakeDeformableStateToRoundBeltStateConverter::CopyRoundBeltStateToOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_round_belt_state* output) const {
  const auto& deformable_geometries =
      this->EvalInputValue<drake::lcmt_viewer_link_data>(
          context, deformable_state_input_port_);
  output->utime = context.get_time() * 1e6;
  output->frame_name = "robot";
  if (deformable_geometries->num_geom == 0) {
    return;
  }
  const auto& geom = deformable_geometries->geom[0];
  assert(geom.type == drake::lcmt_viewer_geometry_data::MESH);
  const int num_verts = static_cast<int>(geom.float_data[0]);
  const int v_start = 2;

  output->num_points = num_verts;
  output->point_positions.resize(output->num_points);
  for (int i = 0; i < output->num_points; i++) {
    output->point_positions[i].resize(3);
    output->point_positions[i][0] = geom.float_data[v_start + 3 * i + 0];
    output->point_positions[i][1] = geom.float_data[v_start + 3 * i + 1];
    output->point_positions[i][2] = geom.float_data[v_start + 3 * i + 2];
  }
  output->num_control_points = 0;
}

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
