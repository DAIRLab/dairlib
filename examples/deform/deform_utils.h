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
/// Deformable example models.
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

/// Constants for the Franka, end effector, and environment.
static constexpr const char* kFrankaModel =
    "package://drake_models/franka_description/urdf/panda_arm.urdf";
static constexpr const char* kEndEffectorModel =
    "examples/deform/models/end_effector_full.urdf";
static constexpr const char* kEndEffectorName = "end_effector_tip";
static constexpr const char* kGroundFrankaModel =
    "examples/deform/models/ground_franka.urdf";
static constexpr const char* kPlatformModel =
    "examples/deform/models/platform.urdf";
static constexpr const char* kBoxModel = "examples/deform/models/box.urdf";
static constexpr const char* kEndEffectorSimpleModel =
    "examples/sampling_c3/urdf/end_effector_simple_model.urdf";
inline const Eigen::VectorXd kQInitFranka =
    (Eigen::VectorXd(7) << 2.19, 0.8, -1.7, -2.4, 0.95, 2.02, 0.08).finished();

/// Tool attachment frame is the offset from the Panda's link7 frame to its
/// flange where an end effector can be attached.
static const Eigen::Vector3d kToolAttachmentFrame = {0, 0, 0.107};
static const Eigen::Vector3d kFrankaToGroundOffset = {0, 0, -0.029};
static const Eigen::Vector3d kFrankaToPlatformOffset = {0, 0, -0.0145};
static const Eigen::Vector3d kWorldToFrankaOffset = {0, 0, 0};
static const Eigen::Vector3d kWorldToGroundOffset =
    kWorldToFrankaOffset + kFrankaToGroundOffset;
static const Eigen::Vector3d kWorldToBoxOffset = {0.4, 0.25, 0};

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

/// Add the Franka to a given multibody plant and scene graph.
drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ee = true,
    const bool& include_ground_and_platform = true,
    const bool& include_box = false);

/// Add the robot hand to a given multibody plant and scene graph.
drake::multibody::ModelInstanceIndex AddRobotHandToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const bool& include_ground = true);

/// Add a deformable elastic object to a given multibody plant and scene graph.
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

/// Add LCS models to the plant for the deformable graph network demo.  This
/// adds the floating spherical EE, deformable nodes, a ground, and optionally
/// a box.
std::vector<drake::multibody::ModelInstanceIndex>
AddLCSModelsForDeformableToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph, const int& n_nodes,
    const double& deformable_mass, const bool& include_box);

/// Add the deformable LCS model to the plant for the deformable graph network
/// demo.  This adds the deformable nodes and assigns the deformable mass
/// divided by the number of nodes to each node.
std::vector<drake::multibody::ModelInstanceIndex> AddDeformableLCSModelToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph, const int& n_nodes,
    const double& deformable_mass);

/// Returns the robot and object ModelInstanceIndexs added to the plant.
std::vector<drake::multibody::ModelInstanceIndex> AddLCSModelsToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    const std::string& demo = kDemos.at(0));

}  // namespace dairlib
