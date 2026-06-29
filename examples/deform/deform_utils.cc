#include "deform_utils.h"

#include <iostream>

#include "common/find_resource.h"

#include "drake/multibody/parsing/parser.h"

namespace dairlib {

using drake::geometry::AddContactMaterial;
using drake::geometry::GeometryInstance;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Shape;
using drake::geometry::Sphere;
using drake::math::RigidTransform;
using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;
using drake::multibody::DeformableModel;
using drake::multibody::LinearSpringDamper;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RigidBody;
using drake::multibody::fem::DeformableBodyConfig;
using Eigen::Vector3d;
using std::vector;

ModelInstanceIndex AddFrankaToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    const bool& include_ee,
                                    const bool& include_ground_and_platform,
                                    const bool& include_box) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  ModelInstanceIndex franka_index = parser.AddModelsFromUrl(kFrankaModel)[0];
  RigidTransformd X_WI = RigidTransformd::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);

  if (include_ee) {
    parser.AddModels(FindResourceOrThrow(kEndEffectorModel));
    RigidTransformd T_EE_W =
        RigidTransformd(drake::math::RotationMatrix<double>(
                            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
                        kToolAttachmentFrame);
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_W);
  }

  if (include_ground_and_platform) {
    parser.AddModels(FindResourceOrThrow(kGroundFrankaModel));
    parser.AddModels(FindResourceOrThrow(kPlatformModel));

    RigidTransformd X_F_P = RigidTransformd(kFrankaToPlatformOffset);
    RigidTransformd X_F_G_franka = RigidTransformd(kFrankaToGroundOffset);

    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("ground"), X_F_G_franka);
    plant->WeldFrames(plant->GetFrameByName("panda_link0"),
                      plant->GetFrameByName("platform"), X_F_P);
  }

  if (include_box) {
    parser.AddModels(FindResourceOrThrow(kBoxModel));
    RigidTransformd X_WB = RigidTransformd(kWorldToBoxOffset);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("box"), X_WB);
  }

  return franka_index;
}

ModelInstanceIndex AddRobotHandToPlant(MultibodyPlant<double>* plant,
                                       SceneGraph<double>* scene_graph,
                                       const bool& include_ground) {
  Parser parser(plant, scene_graph);
  ModelInstanceIndex hand_index = parser.AddModelsFromUrl(kHandModel)[0];
  RigidTransformd X_WH = RigidTransformd(kRobotRotOffset, kRobotPosOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("hand_root"),
                    X_WH);

  if (include_ground) {
    parser.AddModels(FindResourceOrThrow(kGroundModel));
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                      RigidTransformd::Identity());
  }

  return hand_index;
}

static void AddElasticObjectToPlant(MultibodyPlant<double>* plant,
                                    SceneGraph<double>* scene_graph,
                                    std::unique_ptr<Shape> shape,
                                    const double& resolution_hint,
                                    const Eigen::VectorXd& q_init_object) {
  // Set up a deformable object.
  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(kYoungsModulus);
  deformable_config.set_poissons_ratio(kPoissonsRatio);
  deformable_config.set_mass_density(kMassDensity);
  deformable_config.set_stiffness_damping_coefficient(
      kStiffnessDampingCoefficient);

  Eigen::Quaterniond quat_init{q_init_object(0), q_init_object(1),
                               q_init_object(2), q_init_object(3)};
  Vector3d pos_init = q_init_object.tail<3>();
  RigidTransformd X_WO = RigidTransformd(quat_init, pos_init);
  auto elastic_instance = std::make_unique<GeometryInstance>(
      X_WO, std::move(shape), "deformable_elastic");

  // Minimally required proximity properties for deformable bodies: A valid
  // Coulomb friction coefficient.
  ProximityProperties deformable_proximity_props;
  AddContactMaterial(kDissipation, {}, kSurfaceFriction,
                     &deformable_proximity_props);
  elastic_instance->set_proximity_properties(deformable_proximity_props);

  // Register the deformable body.
  DeformableModel<double>& deformable_model = plant->mutable_deformable_model();
  deformable_model.RegisterDeformableBody(std::move(elastic_instance),
                                          deformable_config, resolution_hint);

  // Ensure the plant uses SAP for discrete contact approximation.
  plant->set_discrete_contact_approximation(
      drake::multibody::DiscreteContactApproximation::kSap);
}

void AddElasticMeshToPlant(MultibodyPlant<double>* plant,
                           SceneGraph<double>* scene_graph,
                           const std::string& object_mesh_file,
                           const double& mesh_scale,
                           const Eigen::VectorXd& q_init_object) {
  auto elastic_mesh = std::make_unique<Mesh>(object_mesh_file, mesh_scale);

  /* Registration of all deformable geometries ostensibly requires a
  resolution hint parameter that dictates how the shape is tessellated. In the
  case of a `Mesh` shape, the resolution hint is unused because the shape is
  already tessellated. */
  const double unused_resolution_hint = 1.0;
  AddElasticObjectToPlant(plant, scene_graph, std::move(elastic_mesh),
                          unused_resolution_hint, q_init_object);
}

void AddElasticSphereToPlant(MultibodyPlant<double>* plant,
                             SceneGraph<double>* scene_graph,
                             const double& radius,
                             const double& resolution_hint,
                             const Eigen::VectorXd& q_init_object) {
  auto elastic_sphere = std::make_unique<Sphere>(radius);
  AddElasticObjectToPlant(plant, scene_graph, std::move(elastic_sphere),
                          resolution_hint, q_init_object);
}

vector<ModelInstanceIndex> AddSpringDamperModelToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const SpringDamperModelParams& spring_damper_params) {
  Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);
  vector<ModelInstanceIndex> point_indices;

  int n_vertices = spring_damper_params.vertex_positions.size();
  int n_springs = spring_damper_params.vertex_connections.size();

  // Instantiate vertex positions.
  for (size_t i = 0; i < n_vertices; ++i) {
    const Vector3d& vertex_pos = spring_damper_params.vertex_positions[i];
    ModelInstanceIndex point_index = parser.AddModels(kPointModel)[0];
    point_indices.push_back(point_index);
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base_link", point_index),
                      RigidTransformd());
  }

  // Instantiate springs and dampers between connected vertices.
  for (size_t i = 0; i < n_springs; ++i) {
    int vi_1 = spring_damper_params.vertex_connections[i][0];
    int vi_2 = spring_damper_params.vertex_connections[i][1];
    Eigen::Vector3d p1 = spring_damper_params.vertex_positions[vi_1];
    Eigen::Vector3d p2 = spring_damper_params.vertex_positions[vi_2];
    double free_length =
        (spring_damper_params.vertex_scaling * (p2 - p1).norm());
    ModelInstanceIndex body_i = point_indices[vi_1];
    ModelInstanceIndex body_j = point_indices[vi_2];
    plant->AddForceElement<LinearSpringDamper>(
        plant->GetBodyByName("pt", body_i), drake::Vector3<double>(0, 0, 0),
        plant->GetBodyByName("pt", body_j), drake::Vector3<double>(0, 0, 0),
        free_length, spring_damper_params.spring_stiffness,
        spring_damper_params.damping_coefficient);
  }
  return point_indices;
}

void SetDefaultSpringDamperPositions(
    MultibodyPlant<double>* plant,
    const vector<ModelInstanceIndex>& point_indices,
    const SpringDamperModelParams& spring_damper_params) {
  int n_vertices = spring_damper_params.vertex_positions.size();
  for (size_t i = 0; i < n_vertices; ++i) {
    const Vector3d& vertex_pos = spring_damper_params.vertex_scaling *
                                     spring_damper_params.vertex_positions[i] +
                                 spring_damper_params.initial_offset;
    plant->SetDefaultPositions(point_indices[i], vertex_pos);
  }
}

vector<ModelInstanceIndex> AddLCSModelsForDeformableToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const int& n_nodes, const bool& include_box) {
  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);
  parser_lcs.AddModels(kEndEffectorSimpleModel);
  parser_lcs.AddModels(kGroundModel);

  vector<ModelInstanceIndex> node_model_indices =
      AddDeformableLCSModelToPlant(plant, scene_graph, n_nodes);

  RigidTransformd X_WI = RigidTransformd::Identity();
  RigidTransformd X_W_G =
      RigidTransformd(RotationMatrixd(), kWorldToGroundOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base_link"),
                    X_WI);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                    X_W_G);

  if (include_box) {
    parser_lcs.AddModels(FindResourceOrThrow(kBoxModel));
    RigidTransformd X_WB = RigidTransformd(kWorldToBoxOffset);
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("box"), X_WB);
  }

  return node_model_indices;
}

vector<ModelInstanceIndex> AddDeformableLCSModelToPlant(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const int& n_nodes) {
  Parser parser_lcs(plant);
  parser_lcs.SetAutoRenaming(true);

  vector<ModelInstanceIndex> node_model_indices;
  for (int i = 0; i < n_nodes; ++i) {
    node_model_indices.push_back(parser_lcs.AddModels(kPointModel)[0]);
    plant->WeldFrames(
        plant->world_frame(),
        plant->GetFrameByName("origin_link", node_model_indices[i]),
        RigidTransformd());
  }

  return node_model_indices;
}

vector<ModelInstanceIndex> AddLCSModelsToPlant(MultibodyPlant<double>* plant,
                                               SceneGraph<double>* scene_graph,
                                               const std::string& demo) {
  Parser parser_lcs(plant, scene_graph);
  parser_lcs.SetAutoRenaming(true);
  vector<ModelInstanceIndex> model_indices;

  if (demo == "1d" || demo == "1d_rigid") {
    ModelInstanceIndex robot_idx = parser_lcs.AddModels(kEndEffector1DModel)[0];
    parser_lcs.AddModels(kGroundModel);
    ModelInstanceIndex obj_idx;
    if (demo == "1d") {
      obj_idx = parser_lcs.AddModels(kElastoPlastic1DModel)[0];
      // Add a spring-damper force element between the two object links.
      const RigidBody<double>& slider_body =
          plant->GetBodyByName("frictional_slider");
      const RigidBody<double>& springed_body = plant->GetBodyByName("springed");
      plant->AddForceElement<LinearSpringDamper>(
          slider_body, drake::Vector3<double>(0, 0, 0), springed_body,
          drake::Vector3<double>(0, 0, 0), k1DElastoPlasticFreeLength,
          k1DElastoPlasticStiffness, k1DElastoPlasticDamping);
    } else if (demo == "1d_rigid") {
      obj_idx = parser_lcs.AddModels(kRigid1DModel)[0];
    }

    RigidTransformd X_WI = RigidTransformd::Identity();
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("ground"),
                      X_WI);
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base_link", obj_idx), X_WI);

    RigidTransformd X_WEE = RigidTransformd(k1DRobotPosOffset);
    plant->WeldFrames(plant->world_frame(),
                      plant->GetFrameByName("base_link", robot_idx), X_WEE);

    model_indices.push_back(robot_idx);
    model_indices.push_back(obj_idx);
  } else {
    throw std::runtime_error("Demo " + demo + " not recognized.");
  }
  return model_indices;
}

}  // namespace dairlib
