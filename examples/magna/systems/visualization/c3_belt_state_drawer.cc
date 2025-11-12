#include "examples/magna/systems/visualization/c3_belt_state_drawer.h"

#include "dairlib/lcmt_c3_state.hpp"

#include "drake/common/eigen_types.h"
#include "drake/geometry/rgba.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace visualization {

using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::math::RollPitchYawd;
using drake::math::RotationMatrixd;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::Vector3d;

// Color sets for normal and target state
C3BeltStateDrawer::ColorSet C3BeltStateDrawer::GetColorSet(
    bool is_target_state) {
  if (is_target_state) {
    return {
        Rgba(1.0, 0.0, 0.0, 0.3),  // Red (semi-transparent) for end-effector
        Rgba(0.0, 1.0, 0.0, 0.3)   // Green (semi-transparent) for keypoints
    };
  } else {
    return {
        Rgba(0.0, 0.0, 1.0, 0.7),  // Blue for end-effector
        Rgba(0.0, 0.8, 1.0, 0.7)   // Cyan for keypoints
    };
  }
}

C3BeltStateDrawer::C3BeltStateDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat, int num_keypoints,
    bool is_target_state, const int end_effector_state_size,
    const int keypoint_state_size, const std::string& c3_state_path)
    : meshcat_(meshcat),
      num_keypoints_(num_keypoints),
      is_target_state_(is_target_state),
      end_effector_state_size_(end_effector_state_size),
      keypoint_state_size_(keypoint_state_size),
      c3_state_path_(c3_state_path) {
  this->set_name(is_target_state_ ? "C3BeltStateDrawer (target)"
                                  : "C3BeltStateDrawer");
  std::string port_name =
      is_target_state_ ? "lcmt_c3_state: target" : "lcmt_c3_state";
  c3_state_input_port_ =
      this->DeclareAbstractInputPort(port_name,
                                     drake::Value<dairlib::lcmt_c3_state>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);

  meshcat_->SetProperty(c3_state_path_, "visible", true, 0);

  // Select color set based on whether this is target state or normal state
  ColorSet colors = GetColorSet(is_target_state_);

  // Set up visualization objects for end-effector coordinate frames
  auto x_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitY()),
                      0.5 * Vector3d{0.05, 0.0, 0.0});
  auto y_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitX()),
                      0.5 * Vector3d{0.0, 0.05, 0.0});
  auto z_axis_transform_ee =
      RigidTransformd(Eigen::AngleAxis(0.5 * M_PI, Vector3d::UnitZ()),
                      0.5 * Vector3d{0.0, 0.0, 0.05});

  // Helper lambda to set up end-effector visualization
  auto setup_ee_visualization = [&](const std::string& suffix,
                                    const Rgba& color) {
    std::string ee_path = c3_state_path_ + "/" + suffix;
    meshcat_->SetObject(ee_path + "/x-axis", cylinder_for_ee_,
                        {1, 0, 0, color.a()});
    meshcat_->SetObject(ee_path + "/y-axis", cylinder_for_ee_,
                        {0, 1, 0, color.a()});
    meshcat_->SetObject(ee_path + "/z-axis", cylinder_for_ee_,
                        {0, 0, 1, color.a()});
    meshcat_->SetTransform(ee_path + "/x-axis", x_axis_transform_ee);
    meshcat_->SetTransform(ee_path + "/y-axis", y_axis_transform_ee);
    meshcat_->SetTransform(ee_path + "/z-axis", z_axis_transform_ee);
  };

  // Set up end-effector visualization with appropriate colors
  std::string ee_suffix = is_target_state_ ? "c3_target_ee" : "c3_state_ee";
  setup_ee_visualization(ee_suffix, colors.ee_color);

  // Set up visualization objects for keypoints (spheres)
  std::string keypoint_prefix =
      is_target_state_ ? "c3_target_material" : "c3_state_material";
  for (int i = 0; i < num_keypoints_; ++i) {
    std::string point_suffix = "/point_" + std::to_string(i);
    meshcat_->SetObject(c3_state_path_ + "/" + keypoint_prefix + point_suffix,
                        sphere_for_keypoint_, colors.keypoint_color);
  }

  DeclarePerStepDiscreteUpdateEvent(&C3BeltStateDrawer::DrawC3State);
}

drake::systems::EventStatus C3BeltStateDrawer::DrawC3State(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // if (this->EvalInputValue<dairlib::lcmt_c3_state>(context,
  //                                                  c3_state_input_port_)
  //         ->utime < 1e-3) {
  //   return drake::systems::EventStatus::Succeeded();
  // }
  // if (discrete_state->get_value(last_update_time_index_)[0] >=
  //     context.get_time()) {
  //   // no need to update if simulation has not advanced
  //   return drake::systems::EventStatus::Succeeded();
  // }

  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      context.get_time();
  const auto& c3_state = this->EvalInputValue<dairlib::lcmt_c3_state>(
      context, c3_state_input_port_);

  // If no state is provided, return success immediately to avoid
  // accessing undefined memory.
  if (c3_state->num_states == 0) {
    return drake::systems::EventStatus::Succeeded();
  }

  // State structure:
  // [0-2]: end-effector xyz position
  // [3-5]: roll-pitch-yaw angles
  // [6-8], [9-11], ...: xyz positions of keypoints (3 per point)

  // Determine paths based on whether this is target state or normal state
  std::string ee_suffix = is_target_state_ ? "c3_target_ee" : "c3_state_ee";
  std::string keypoint_prefix =
      is_target_state_ ? "c3_target_material" : "c3_state_material";

  // Draw end-effector with orientation
  auto draw_ee = [&](const std::string& suffix,
                     const dairlib::lcmt_c3_state& state, double time = 0) {
    Vector3d ee_pos(state.state[0], state.state[1], state.state[2]);
    RollPitchYawd rpy(state.state[3], state.state[4], state.state[5]);
    std::string ee_path = c3_state_path_ + "/" + suffix;
    if (time > 0) {
      meshcat_->SetTransform(
          ee_path, RigidTransformd(rpy.ToRotationMatrix(), ee_pos), time);
    } else {
      meshcat_->SetTransform(ee_path,
                             RigidTransformd(rpy.ToRotationMatrix(), ee_pos));
    }
  };
  draw_ee(ee_suffix, *c3_state, context.get_time());

  // Draw keypoints
  for (int i = 0; i < num_keypoints_; ++i) {
    int base_idx = end_effector_state_size_ + keypoint_state_size_ * i;
    std::string point_suffix = "/point_" + std::to_string(i);
    std::string path = c3_state_path_ + "/" + keypoint_prefix + point_suffix;
    Vector3d kp_pos(c3_state->state.at(base_idx),
                    c3_state->state.at(base_idx + 1),
                    c3_state->state.at(base_idx + 2));
    meshcat_->SetTransform(path, RigidTransformd(kp_pos), context.get_time());
  }

  return drake::systems::EventStatus::Succeeded();
}

}  // namespace visualization
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
