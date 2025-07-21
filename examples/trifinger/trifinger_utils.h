#pragma once

#include <string>
#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {

/// Constants for the Franka and end effector.
static constexpr const char* kTrifingerModel =
  "examples/trifinger/models/urdf/trifinger.urdf";
static constexpr const char* kGroundModel =
  "examples/sampling_c3/urdf/ground.urdf";
static constexpr const char* kTrifingerSimplifiedModel =
  "examples/trifinger/models/urdf/simplified_trifinger.urdf";
static constexpr const char* kFingertip0Name = "finger_tip_link_0";
static constexpr const char* kFingertip120Name = "finger_tip_link_120";
static constexpr const char* kFingertip240Name = "finger_tip_link_240";

/// Add the Trifinger to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @return the ModelInstanceIndex of the Trifinger in the plant
drake::multibody::ModelInstanceIndex AddTrifingerToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr
);

/// Add an object to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
/// @return the ModelInstanceIndex of the object in the plant
drake::multibody::ModelInstanceIndex AddObjectToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& object_model = "");

/// Add LCS models to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_model the model of the object to add to the plant
/// @return a pair of ModelInstanceIndex for the trifinger simplified model and the object model
std::pair<drake::multibody::ModelInstanceIndex, drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& object_model = "");
}   // namespace dairlib
