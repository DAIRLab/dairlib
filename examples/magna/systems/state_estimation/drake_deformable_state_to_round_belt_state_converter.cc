#include "examples/magna/systems/state_estimation/drake_deformable_state_to_round_belt_state_converter.h"

#include <iostream>

#include <drake/lcmt_viewer_link_data.hpp>

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

DrakeDeformableStateToRoundBeltStateConverter::
    DrakeDeformableStateToRoundBeltStateConverter(
        std::vector<double> taskboard_position,
        std::vector<double> taskboard_orientation)
    : taskboard_position_(taskboard_position),
      taskboard_orientation_(taskboard_orientation) {
  taskboard_transform_ = drake::math::RigidTransform<double>(
      drake::math::RollPitchYaw<double>(taskboard_orientation_[0],
                                        taskboard_orientation_[1],
                                        taskboard_orientation_[2]),
      Eigen::Vector3d(taskboard_position_[0], taskboard_position_[1],
                      taskboard_position_[2]));
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
  output->frame_name = "taskboard";
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
    Eigen::Vector3d point_position_wrt_taskboard;
    point_position_wrt_taskboard << geom.float_data[v_start + 3 * i + 0],
        geom.float_data[v_start + 3 * i + 1],
        geom.float_data[v_start + 3 * i + 2];
    point_position_wrt_taskboard = taskboard_transform_.rotation().inverse() *
                                       point_position_wrt_taskboard -
                                   taskboard_transform_.rotation().inverse() *
                                       taskboard_transform_.translation();
    output->point_positions[i].resize(3);
    output->point_positions[i][0] = point_position_wrt_taskboard[0];
    output->point_positions[i][1] = point_position_wrt_taskboard[1];
    output->point_positions[i][2] = point_position_wrt_taskboard[2];
  }
  output->num_control_points = 0;
}

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
