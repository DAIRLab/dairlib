#pragma once

#include <string>

#include <Eigen/Dense>

#include "examples/deform/parameter_headers/spring_damper_model_params.h"

#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/linear_spring_damper.h"

namespace dairlib {

using drake::multibody::CoulombFriction;

/// Constants.
/// Robot models.
static constexpr const char* kHandModel =
    "package://drake_models/allegro_hand_description/urdf/"
    "allegro_hand_description_right.urdf";
static constexpr const char* kGroundModel =
    "examples/deform/models/ground.urdf";
static constexpr const char* kPointModel =
    "examples/deform/models/small_xyz_point.urdf";
static constexpr const char* kEndEffector1DModel =
    "examples/deform/models/ee_1d_sphere.urdf";
static constexpr const char* kElastoPlastic1DModel =
    "examples/deform/models/elastoplastic_1d.urdf";
static constexpr const char* kRigid1DModel =
    "examples/deform/models/rigid_1d.urdf";

/// 1D demo specific parameters.
static const double k1DElastoPlasticFreeLength = 0.1;
static const double k1DElastoPlasticStiffness = 2e2;
static const double k1DElastoPlasticDamping = 2e0;
static const Eigen::Vector3d k1DRobotPosOffset = {0, 0, 0.05};

/// Initial poses.
static const Eigen::Vector3d kRobotPosOffset = {0, 0.2, 0.2};
static const Eigen::Quaterniond kRobotRotOffset = {0.5, -0.5, -0.5, -0.5};

/// FEM material properties.
static const double kYoungsModulus = 3e4;
static const double kPoissonsRatio = 0.4;
static const double kMassDensity = 1e3;
static const double kStiffnessDampingCoefficient = 0.01;
static const CoulombFriction<double> kSurfaceFriction(1.15, 1.15);
static const double kDissipation = 10.0;

/// Demos.
static const std::vector<std::string> kDemos = {"1d", "1d_rigid"};

/// Add the robot hand to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param include_ground whether to include the ground in the plant
/// @return the ModelInstanceIndex of the robot hand in the plant
drake::multibody::ModelInstanceIndex AddRobotHandToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ground = true);

/// Add a deformable elastic object to a given multibody plant and scene graph.
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param object_mesh_file the file path to the mesh of the deformable object
/// @param q_init_object the initial pose of the deformable object as a 7D
///        vector (qw, qx, qy, qz, x, y, z)
void AddElasticMeshToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& object_mesh_file = "", const double& mesh_scale = 1.0,
    const Eigen::VectorXd& q_init_object = {1, 0, 0, 0, 0, 0, 0});

void AddElasticSphereToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const double& radius = 0.1, const double& resolution_hint = 0.02,
    const Eigen::VectorXd& q_init_object = {1, 0, 0, 0, 0, 0, 0});

std::vector<drake::multibody::ModelInstanceIndex> AddSpringDamperModelToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const SpringDamperModelParams& spring_damper_params = {});

void SetDefaultSpringDamperPositions(
    drake::multibody::MultibodyPlant<double>* plant,
    const std::vector<drake::multibody::ModelInstanceIndex>& point_indices,
    const SpringDamperModelParams& spring_damper_params);

/// Returns the robot and object ModelInstanceIndexs added to the plant.
std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& demo = kDemos.at(0));

}  // namespace dairlib
