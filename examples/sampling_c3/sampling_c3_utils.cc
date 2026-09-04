#include "sampling_c3_utils.h"

#include <iostream>

#include "common/find_resource.h"

#include "drake/common/drake_assert.h"
#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
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

ModelInstanceIndex Add3DPrinterToPlant(MultibodyPlant<double>* plant,
                                       SceneGraph<double>* scene_graph,
                                       const bool& include_ee) {
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
    plant->WeldFrames(plant->GetFrameByName("x_carriage"),
                      plant->GetFrameByName("end_effector_flange"),
                      T_Printer_EE);

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
  vector<SortedPair<GeometryId>> ee_contact_pairs;
  vector<SortedPair<GeometryId>> ground_object_contact_pairs;
  std::unordered_map<std::string, GeometryId> contact_geoms;

  // All demos include the end effector and ground.
  GeometryId ee_contact_points = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("end_effector_simple"))[0];
  GeometryId ground_geoms = plant_lcs.GetCollisionGeometriesForBody(
      plant_lcs.GetBodyByName("ground"))[0];

  contact_geoms["EE"] = ee_contact_points;
  contact_geoms["GROUND"] = ground_geoms;

  vector<SortedPair<GeometryId>> ee_ground_contact{
      SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};

  // For each pair of object-object or wall-object, we store the contact pairs
  // between their convex pieces
  vector<vector<SortedPair<GeometryId>>> object_object_contact_pairs;
  vector<vector<SortedPair<GeometryId>>> wall_object_contact_pairs;

  // Build the demo-specific contact pairs: EE-object, object-ground,
  // ramp-object, EE-ramp, and object-object.
  vector<vector<GeometryId>> all_object_geoms;
  for (int i = 0; i < base_names.size(); i++) {
    std::string body_name = base_names.at(i);
    const vector<GeometryId>& object_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName(body_name));
    const vector<GeometryId>& ramp_geoms =
        plant_lcs.GetCollisionGeometriesForBody(
            plant_lcs.GetBodyByName("ramp_link"));

    GeometryId corner_1_sphere_geoms = object_geoms[1];
    GeometryId corner_2_sphere_geoms = object_geoms[2];
    GeometryId corner_3_sphere_geoms = object_geoms[3];
    GeometryId corner_4_sphere_geoms = object_geoms[4];
    GeometryId corner_5_sphere_geoms = object_geoms[5];
    GeometryId corner_6_sphere_geoms = object_geoms[6];
    GeometryId corner_7_sphere_geoms = object_geoms[7];

    contact_geoms["CORNER_1_SPHERE"] = corner_1_sphere_geoms;
    contact_geoms["CORNER_2_SPHERE"] = corner_2_sphere_geoms;
    contact_geoms["CORNER_3_SPHERE"] = corner_3_sphere_geoms;
    contact_geoms["CORNER_4_SPHERE"] = corner_4_sphere_geoms;
    contact_geoms["CORNER_5_SPHERE"] = corner_5_sphere_geoms;
    contact_geoms["CORNER_6_SPHERE"] = corner_6_sphere_geoms;
    contact_geoms["CORNER_7_SPHERE"] = corner_7_sphere_geoms;

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

    for (int j = 0; j < ramp_geoms.size(); j++) {
      for (int k = 1; k < object_geoms.size(); k++) {
        ground_object_contact_pairs.push_back(
            SortedPair(ramp_geoms[j], object_geoms[k]));
      }
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], ramp_geoms[j]));
    }

    const vector<GeometryId> object_geoms_without_spheres =
        vector<GeometryId>(object_geoms.begin(), object_geoms.end() - 7);

    ee_contact_pairs.push_back(
        SortedPair(contact_geoms["EE"], object_geoms[0]));
    all_object_geoms.push_back(object_geoms_without_spheres);
  }

  // Object-object contact pairs (excluding end effector), each pair of
  // convex pieces for each pair of objects
  for (int i = 0; i + 1 < base_names.size(); i++) {
    for (int j = i + 1; j < base_names.size(); j++) {
      vector<SortedPair<GeometryId>> convex_piece_pairs;
      const vector<GeometryId>& object_1_geoms = all_object_geoms.at(i);
      const vector<GeometryId>& object_2_geoms = all_object_geoms.at(j);

      for (const auto& g1 : object_1_geoms) {
        for (const auto& g2 : object_2_geoms) {
          convex_piece_pairs.emplace_back(g1, g2);
        }
      }
      object_object_contact_pairs.push_back(std::move(convex_piece_pairs));
    }
  }
  // Order:  EE-ground, EE-object, object-ground, object-object, object-wall
  contact_pairs.push_back(ee_ground_contact);
  contact_pairs.push_back(ee_contact_pairs);
  contact_pairs.push_back(ground_object_contact_pairs);
  for (const auto& obj_obj_pair : object_object_contact_pairs) {
    contact_pairs.push_back(obj_obj_pair);
  }
  for (const auto& wall_obj_pair : wall_object_contact_pairs) {
    contact_pairs.push_back(wall_obj_pair);
  }
  return contact_pairs;
}

}  // namespace dairlib
