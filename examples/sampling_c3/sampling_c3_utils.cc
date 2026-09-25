#include "sampling_c3_utils.h"

#include <iostream>
#include <stdexcept>
#include <string>

#include "common/find_resource.h"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/prismatic_spring.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using std::vector;

ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_walls) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransform<double> T_Franka_EE = RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"),
                      T_Franka_EE);
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

void AddWallsToPlant(drake::multibody::MultibodyPlant<double>* plant,
                     drake::geometry::SceneGraph<double>* scene_graph,
                     const bool& include_back_wall) {
  Eigen::Vector3d side_wall_size(kWallLengthX, kWallWidth, kWallHeight);
  AddBoxToPlant(plant, scene_graph, side_wall_size, "left_wall");
  AddBoxToPlant(plant, scene_graph, side_wall_size, "right_wall");
  Eigen::Vector3d wall_size(kWallWidth, kWallLengthY + 2 * kWallWidth,
                            kWallHeight);
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

void AddBoxToPlant(drake::multibody::MultibodyPlant<double>* plant,
                   drake::geometry::SceneGraph<double>* scene_graph,
                   const Eigen::Vector3d& box_size,
                   const std::string& box_name) {
  ModelInstanceIndex model_instance_index = plant->AddModelInstance(box_name);
  const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(
      box_name, model_instance_index,
      drake::multibody::SpatialInertia<double>::SolidBoxWithMass(
          1.0, box_size(0) / 2, box_size(1) / 2, box_size(2) / 2));

  plant->RegisterVisualGeometry(
      body, RigidTransform<double>::Identity(),
      drake::geometry::Box(box_size(0), box_size(1), box_size(2)), box_name,
      kWallColor);
  plant->RegisterCollisionGeometry(
      body, RigidTransform<double>::Identity(),
      drake::geometry::Box(box_size(0), box_size(1), box_size(2)), box_name,
      kWallFriction);
}

ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    const std::string& object_model) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  return parser.AddModels(FindResourceOrThrow(object_model))[0];
}

vector<ModelInstanceIndex> AddObjectsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    vector<std::string> object_models) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  vector<ModelInstanceIndex> models;
  for (const auto& model : object_models) {
    models.push_back(parser.AddModels(FindResourceOrThrow(model))[0]);
  }
  return models;
}

void AddLCSModelToPlant(MultibodyPlant<double>* plant,
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
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }
}

vector<ModelInstanceIndex> AddLCSModelsToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    vector<std::string> object_models,
    const bool& include_end_effector_orientation, const bool& include_walls) {
  // Cannot currently handle end effector orientation (would just require new
  // EE simple model with orientation DOFs).
  DRAKE_ASSERT(!include_end_effector_orientation);

  vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  for (const auto& model : object_models) {
    obj_models.push_back(parser_lcs.AddModels(FindResourceOrThrow(model))[0]);
  }

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  RigidTransform<double> X_W_G = RigidTransform<double>(
      drake::math::RotationMatrix<double>(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_W_G);

  if (include_walls) {
    // TODO: may want to exclude the back wall for the LCS model.
    AddWallsToPlant(plant, scene_graph);  //, false);
  }

  return obj_models;
}

ModelInstanceIndex Add3DPrinterToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const bool& include_ee,
    const std::optional<FingerCompliance>& finger_compliance) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex printer_index =
      parser.AddModels(FindResourceOrThrow(k3DPrinterModel))[0];
  ModelInstanceIndex ramp_index =
      parser.AddModels(FindResourceOrThrow(k3DPrinterRampModel))[0];

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  RigidTransform<double> X_World_Ramp(k3dPrinterRampAttachmentRotationMatrix,
                                      k3dPrinterRampAttachmentFrame);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ramp_link"),
                    X_World_Ramp);

  // Disable gravity for the entire printer model instance.
  plant->set_gravity_enabled(printer_index, false);
  plant->set_gravity_enabled(ramp_index, false);

  plant->AddJointActuator("x_axis_actuator",
                          plant->GetJointByName("x_axis_joint", printer_index));
  plant->AddJointActuator("y_axis_actuator",
                          plant->GetJointByName("y_axis_joint", printer_index));
  plant->AddJointActuator("z_axis_actuator",
                          plant->GetJointByName("z_axis_joint", printer_index));

  if (plant->time_step() > 0.0) {
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("x_axis_actuator").index())
        .set_controller_gains(k3dPrinterXYAxesPdGains);
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("y_axis_actuator").index())
        .set_controller_gains(k3dPrinterXYAxesPdGains);
    plant
        ->get_mutable_joint_actuator(
            plant->GetJointActuatorByName("z_axis_actuator").index())
        .set_controller_gains(k3dPrinterZAxisPdGains);
  }

  if (include_ee) {
    ModelInstanceIndex ee_index = parser.AddModels(k3dEndEffectorModel)[0];
    RigidTransform<double> T_Printer_EE(k3dPrinterToolAttachmentFrame);
    if (!finger_compliance.has_value()) {
      plant->WeldFrames(plant->GetFrameByName("x_carriage"),
                        plant->GetFrameByName("end_effector_flange"),
                        T_Printer_EE);
    } else {
      // The whole end effector translates horizontally on two springs, x then
      // y, through a light intermediate body.  A translation rather than a
      // bend about the flange: what matters here is how far the tip can trail
      // the carriage it is reported as, not the tip's small rise as it bends.
      const RigidBody<double>& deflection_x_body = plant->AddRigidBody(
          "finger_deflection_x", ee_index,
          drake::multibody::SpatialInertia<double>::SolidSphereWithMass(
              0.01, 0.005));
      const auto& x_joint =
          plant->AddJoint<drake::multibody::PrismaticJoint>(
              "finger_deflection_x_joint",
              plant->GetBodyByName("x_carriage"), T_Printer_EE,
              deflection_x_body, RigidTransform<double>::Identity(),
              Eigen::Vector3d::UnitX(),
              -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              finger_compliance->damping);
      const auto& y_joint =
          plant->AddJoint<drake::multibody::PrismaticJoint>(
              "finger_deflection_y_joint", deflection_x_body,
              RigidTransform<double>::Identity(),
              plant->GetBodyByName("end_effector_flange", ee_index),
              RigidTransform<double>::Identity(), Eigen::Vector3d::UnitY(),
              -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(),
              finger_compliance->damping);
      plant->AddForceElement<drake::multibody::PrismaticSpring>(
          x_joint, 0.0, finger_compliance->stiffness);
      plant->AddForceElement<drake::multibody::PrismaticSpring>(
          y_joint, 0.0, finger_compliance->stiffness);
    }

    // Disable gravity for all end effector bodies.
    plant->set_gravity_enabled(ee_index, false);
  }

  return printer_index;
}

vector<ModelInstanceIndex> AddLCSModelsTo3DPrinterPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    vector<std::string> object_models) {
  vector<ModelInstanceIndex> obj_models;

  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(k3dEndEffectorSimpleModel);
  parser_lcs.AddModels(kBaseModel);
  parser_lcs.AddModels(FindResourceOrThrow(k3DPrinterRampModel));

  for (const auto& model : object_models) {
    obj_models.push_back(parser_lcs.AddModels(FindResourceOrThrow(model))[0]);
  }

  RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_ee_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_WI);

  RigidTransform<double> X_World_Ramp(k3dPrinterRampAttachmentRotationMatrix,
                                      k3dPrinterRampAttachmentFrame);

  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ramp_link"),
                    X_World_Ramp);

  return obj_models;
}

vector<ModelInstanceIndex> AddKeepOutModelsToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const vector<std::string>& keep_out_models) {
  DRAKE_DEMAND(scene_graph != nullptr);
  vector<ModelInstanceIndex> indices;
  indices.reserve(keep_out_models.size());

  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  for (const std::string& model_path : keep_out_models) {
    if (model_path.empty()) {
      indices.push_back(ModelInstanceIndex{});  // invalid: no regions this step
      continue;
    }
    ModelInstanceIndex model_instance =
        parser.AddModels(FindResourceOrThrow(model_path)).at(0);
    // The URDF root link must be named "keep_out_base"; each region link is
    // connected via a fixed joint.
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("keep_out_base", model_instance),
                      RigidTransform<double>::Identity());
    indices.push_back(model_instance);
  }
  return indices;
}

vector<vector<drake::SortedPair<drake::geometry::GeometryId>>>
BuildConeContactPairs(const MultibodyPlant<double>& plant_lcs,
                      const vector<std::string>& base_names) {
  using drake::SortedPair;
  using drake::geometry::GeometryId;

  vector<vector<SortedPair<GeometryId>>> contact_pairs;
  vector<SortedPair<GeometryId>> ee_ground_contact_pairs;
  vector<SortedPair<GeometryId>> ee_object_contact_pairs;
  vector<SortedPair<GeometryId>> ground_object_contact_pairs;
  vector<SortedPair<GeometryId>> ramp_object_contact_pairs;
  std::unordered_map<std::string, GeometryId> contact_geoms;

  GeometryId ee_contact_points = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  GeometryId ground_geom = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("ground"))[0];
  const vector<GeometryId>& ramp_geoms =
      plant_lcs.GetCollisionGeometriesForBody(
          plant_lcs.GetBodyByName("ramp_link"));

  contact_geoms["EE"] = ee_contact_points;
  contact_geoms["GROUND"] = ground_geom;

  // Add the EE-ground contact.  EE-ramp contact(s) also go here.
  ee_ground_contact_pairs.push_back(
      SortedPair(contact_geoms["EE"], contact_geoms["GROUND"]));

  // Build the demo-specific contact pairs: EE-object, object-ground,
  // object-ramp, and EE-ramp.
  for (int i = 0; i < base_names.size(); i++) {
    std::string body_name = base_names.at(i);
    const vector<GeometryId>& object_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName(body_name));

    contact_geoms["FULL_CONE"] = object_geoms[0];
    contact_geoms["CORNER_1_SPHERE"] = object_geoms[1];
    contact_geoms["CORNER_2_SPHERE"] = object_geoms[2];
    contact_geoms["CORNER_3_SPHERE"] = object_geoms[3];
    contact_geoms["CORNER_4_SPHERE"] = object_geoms[4];
    contact_geoms["CORNER_5_SPHERE"] = object_geoms[5];
    contact_geoms["CORNER_6_SPHERE"] = object_geoms[6];
    contact_geoms["CORNER_7_SPHERE"] = object_geoms[7];

    // Ground-object includes all corners with the ground.
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_1_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_2_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_3_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_4_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_5_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_6_SPHERE"], contact_geoms["GROUND"]));
    ground_object_contact_pairs.push_back(
        SortedPair(contact_geoms["CORNER_7_SPHERE"], contact_geoms["GROUND"]));

    // Ramp-object pairs every corner PLUS the full cone with every ramp piece.
    for (int j = 0; j < ramp_geoms.size(); j++) {
      for (int k = 0; k < object_geoms.size(); k++) {
        ramp_object_contact_pairs.push_back(
            SortedPair(ramp_geoms[j], object_geoms[k]));
      }

      // EE-ground contacts include EE-ground and EE-ramp contacts.
      ee_ground_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], ramp_geoms[j]));
    }

    // EE-object contact is just the EE with the full cone.
    ee_object_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], object_geoms[0]));
  }

  // Order:  EE-ground, EE-object, object-ground, object-ramp.
  contact_pairs.push_back(ee_ground_contact_pairs);
  contact_pairs.push_back(ee_object_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);
  contact_pairs.push_back(ramp_object_contact_pairs);
  return contact_pairs;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> GetWorkspaceBox(
    const std::vector<Eigen::VectorXd>& workspace_limits, double margin) {
  DRAKE_THROW_UNLESS(workspace_limits.size() == 3);
  Eigen::Vector3d lower;
  Eigen::Vector3d upper;
  for (int i = 0; i < 3; ++i) {
    DRAKE_THROW_UNLESS(workspace_limits[i].size() == 5);
    // Assume axis-aligned:  row i must be the unit normal along axis i.
    if (workspace_limits[i].segment(0, 3) != Eigen::Vector3d::Unit(i)) {
      throw std::runtime_error(
          "GetWorkspaceBox: workspace_limits row " + std::to_string(i) +
          " has normal [" + std::to_string(workspace_limits[i][0]) + ", " +
          std::to_string(workspace_limits[i][1]) + ", " +
          std::to_string(workspace_limits[i][2]) +
          "], but an axis-aligned box requires the unit normal along axis " +
          std::to_string(i) +
          ".  Non-axis-aligned workspace limits are honored by the C3 solve's "
          "linear constraints but not by any of the clamps that read them.");
    }
    lower(i) = workspace_limits[i][3] + margin;
    upper(i) = workspace_limits[i][4] - margin;
    DRAKE_THROW_UNLESS(lower(i) <= upper(i));
  }
  return {lower, upper};
}

}  // namespace dairlib
