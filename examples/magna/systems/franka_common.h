#include "common/find_resource.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {

#define FRANKA_MODEL \
  "package://drake_models/franka_description/urdf/panda_arm.urdf"
#define FRANKA_HAND_MODEL \
  "package://drake_models/franka_description/urdf/panda_arm_hand.urdf"

/// This is the offset from the Panda's link7 frame to its flange where an end
/// effector can be attached.
inline const Eigen::Vector3d TOOL_ATTACHMENT_FRAME = {0, 0, 0.107};
inline const drake::math::RigidTransform<double> T_EE_L7 =
    drake::math::RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(M_PI, 0, -M_PI/4)),
        TOOL_ATTACHMENT_FRAME);

drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::optional<std::string> end_effector_model_path,
    bool with_hand = false) {
  drake::multibody::Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  drake::multibody::ModelInstanceIndex franka_index;
  if (with_hand) {
    franka_index = parser.AddModelsFromUrl(
        "package://drake_models/franka_description/urdf/panda_arm_hand.urdf")
                       [0];
  } else {
    franka_index = parser.AddModelsFromUrl(
        "package://drake_models/franka_description/urdf/panda_arm.urdf")[0];
  }
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);
  plant->AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
      "end_effector_frame", plant->GetBodyByName("panda_link7"), T_EE_L7));
  if (end_effector_model_path.has_value()) {
    parser.AddModels(FindResourceOrThrow(end_effector_model_path.value()));
    plant->WeldFrames(plant->GetFrameByName("end_effector_frame"),
                      plant->GetFrameByName("end_effector_flange"),
                      drake::math::RigidTransform<double>::Identity());
  }

  return franka_index;
}

drake::multibody::ModelInstanceIndex AddFrankaHandToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph) {
  drake::multibody::Parser parser(plant, scene_graph);
  auto hand_index = parser.AddModelsFromUrl(
      "package://drake_models/franka_description/urdf/panda_hand.urdf")[0];
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(),
                    plant->GetBodyByName("panda_hand").body_frame(), X_WI);
  return hand_index;
}

}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib