#include "sampling_c3_utils.h"
#include <iostream>
#include "common/find_resource.h"
#include "drake/common/drake_assert.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::BodyIndex;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;


ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("panda_link0"), X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransform<double> T_EE_W = RigidTransform<double>(
      drake::math::RotationMatrix<double>(
        drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
      kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
    }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransform<double> X_F_P = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToPlatformOffset);
    RigidTransform<double> X_F_G_franka = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("ground"), X_F_G_franka);
    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_walls) {
    AddWallsToPlant(plant, scene_graph);
  }

  return franka_index;
}

void AddWallsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const bool& include_back_wall) {
  Eigen::Vector3d side_wall_size(kWallLengthX, kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, side_wall_size, "left_wall");
  AddBoxToPlant(plant, scene_graph, side_wall_size, "right_wall");
  Eigen::Vector3d wall_size(kWallWidth, kWallLengthY+2*kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, wall_size, "front_wall");

  RigidTransform<double> X_G_LW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToLeftWallOffset);
  RigidTransform<double> X_G_RW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToRightWallOffset);
  RigidTransform<double> X_G_FW = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kGroundToFrontWallOffset);

  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("left_wall"), X_G_LW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("right_wall"), X_G_RW);
  plant->WeldFrames(plant->GetFrameByName("ground"),
                    plant->GetFrameByName("front_wall"), X_G_FW);

  if (include_back_wall) {
    AddBoxToPlant(plant, scene_graph, wall_size, "back_wall");
    RigidTransform<double> X_G_BW = RigidTransform<double>(
        drake::math::RotationMatrix<double>(), kGroundToBackWallOffset);
    plant->WeldFrames(plant->GetFrameByName("ground"),
                      plant->GetFrameByName("back_wall"), X_G_BW);
  }
}

void AddBoxToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const Eigen::Vector3d& box_size,
    const std::string& box_name) {
  ModelInstanceIndex model_instance_index = plant->AddModelInstance(box_name);
  const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(
    box_name,
    model_instance_index,
    drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
      1.0, box_size(0)/2, box_size(1)/2, box_size(2)/2));

  plant->RegisterVisualGeometry(
    body, RigidTransform<double>::Identity(),
    drake::geometry::Box(box_size(0), box_size(1), box_size(2)),
    box_name, kWallColor);
  plant->RegisterCollisionGeometry(
    body, RigidTransform<double>::Identity(),
    drake::geometry::Box(box_size(0), box_size(1), box_size(2)),
    box_name, kWallFriction);
}

ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

std::vector<ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models,
    const bool& orientation_is_quaternion) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  std::vector<ModelInstanceIndex> models;
  for (const auto& model : object_models) {
    models.push_back(parser.AddModels(FindResourceOrThrow(model))[0]);
    if (!orientation_is_quaternion) {
      // Get the name of the object in the model.
      ModelInstanceIndex obj_index = models.back();
      BodyIndex body_index = plant->GetBodyIndices(obj_index)[0];
      const std::string& obj_name = plant->get_body(body_index).name();

      // Weld the object's base link to the world.
      RigidTransform<double> X_WI = RigidTransform<double>::Identity();
      plant->WeldFrames(plant->world_frame(),
                        plant->GetFrameByName(obj_name + "_base"), X_WI);
    }
  }
  return models;
}

void AddLCSModelToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    const std::string& object_model,
    const bool& include_end_effector_orientation,
    const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_DEMAND(!include_end_effector_orientation);

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);
  parser_lcs.AddModels(object_model);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }
}


std::vector<ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant,
    SceneGraph<double>* scene_graph,
    std::vector<std::string> object_models,
    const bool& include_end_effector_orientation,
    const bool& include_walls,
    const bool& orientation_is_quaternion) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  std::vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("base_link"), X_WI);
  plant->WeldFrames(plant->world_frame(),
                    plant->GetFrameByName("ground"), X_W_G);

  for (const auto& model : object_models) {
    obj_models.push_back(parser_lcs.AddModels(FindResourceOrThrow(model))[0]);
    if (!orientation_is_quaternion) {
      // Get the name of the object in the model.
      ModelInstanceIndex obj_index = obj_models.back();
      BodyIndex body_index = plant->GetBodyIndices(obj_index)[0];
      const std::string& obj_name = plant->get_body(body_index).name();

      // Weld the object's base link to the world.
      RigidTransform<double> X_WI = RigidTransform<double>::Identity();
      plant->WeldFrames(plant->world_frame(),
                        plant->GetFrameByName(obj_name + "_base"), X_WI);
    }
  }

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }

  return obj_models;
}

/// State vector interpretation
Eigen::Quaterniond GetObjectQuaternionFromTheta(const double& theta) {
  Eigen::AngleAxisd aa(theta, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat(aa);
  return quat;
}

Eigen::Quaterniond GetObjectQuaternionFromObjectConfig(
    const Eigen::VectorXd& object_config,
    const bool& orientation_is_quaternion
) {
  if (orientation_is_quaternion) {
    DRAKE_DEMAND(object_config.size() == 7);
    Eigen::Vector4d quat_coeffs = object_config.head(4);
    quat_coeffs.normalize();
    return Eigen::Quaterniond(quat_coeffs(0), quat_coeffs(1),
                              quat_coeffs(2), quat_coeffs(3));
  }
  DRAKE_DEMAND(object_config.size() == 4);
  return GetObjectQuaternionFromTheta(object_config(3));
}

Eigen::Vector3d GetObjectPositionFromObjectConfig(
    const Eigen::VectorXd& object_config,
    const bool& orientation_is_quaternion
) {
  if (orientation_is_quaternion) {
    return object_config.tail(3);
  }
  return object_config.head(3);
}

Eigen::VectorXd GetObjectConfigFromPosQuat(
    const Eigen::Vector3d& object_xyz,
    const Eigen::Quaterniond& object_quat,
    const bool& orientation_is_quaternion
) {
  if (orientation_is_quaternion) {
    Eigen::VectorXd config = Eigen::VectorXd::Zero(7);
    config << object_quat.w(), object_quat.x(), object_quat.y(),
              object_quat.z(), object_xyz;
    return config;
  }
  // Use only the vertical rotation components of the quaternion.
  double w = object_quat.w();
  double z = object_quat.z();
  w = w / std::sqrt(w * w + z * z);
  z = z / std::sqrt(w * w + z * z);
  double theta = 2.0 * std::atan2(z, w);
  Eigen::VectorXd config = Eigen::VectorXd::Zero(4);
  config << object_xyz, theta;
  return config;
}

Eigen::VectorXd GetObjectVelocityFromLinRotVel(
    const Eigen::Vector3d& obj_lin_vel,
    const Eigen::Vector3d& obj_rot_vel,
    const bool& orientation_is_quaternion
) {
  if (orientation_is_quaternion) {
    Eigen::VectorXd vel = Eigen::VectorXd::Zero(6);
    vel << obj_rot_vel, obj_lin_vel;
    return vel;
  }
  // Use only the vertical rotation component of the angular velocity.
  Eigen::VectorXd vel = Eigen::VectorXd::Zero(4);
  vel << obj_lin_vel, obj_rot_vel(2);
  return vel;
}

Eigen::VectorXd GetObjectConfigFromLCSState(
    const Eigen::VectorXd& x_lcs,
    const int& object_index,
    const bool& orientation_is_quaternion
) {
  if (orientation_is_quaternion) {
    return x_lcs.segment<7>(3 + object_index*7);
  }
  return x_lcs.segment<4>(3 + object_index*4);
}

Eigen::Quaterniond GetObjectQuatFromLCSState(
    const Eigen::VectorXd& x_lcs,
    const int& object_index,
    const bool& orientation_is_quaternion
) {
  Eigen::VectorXd object_config = GetObjectConfigFromLCSState(
    x_lcs, object_index, orientation_is_quaternion);
  return GetObjectQuaternionFromObjectConfig(
    object_config, orientation_is_quaternion);
}

Eigen::Vector3d GetObjectPosFromLCSState(
    const Eigen::VectorXd& x_lcs,
    const int& object_index,
    const bool& orientation_is_quaternion
) {
  Eigen::VectorXd object_config = GetObjectConfigFromLCSState(
    x_lcs, object_index, orientation_is_quaternion);
  return GetObjectPositionFromObjectConfig(
    object_config, orientation_is_quaternion);
}


}   // namespace dairlib
