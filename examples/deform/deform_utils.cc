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
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableModel;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::fem::DeformableBodyConfig;
using Eigen::Vector3d;

ModelInstanceIndex AddRobotHandToPlant(MultibodyPlant<double>* plant,
                                       SceneGraph<double>* scene_graph) {
  Parser parser(plant, scene_graph);
  ModelInstanceIndex hand_index = parser.AddModelsFromUrl(kHandModel)[0];
  RigidTransformd X_WH = RigidTransformd(kRobotRotOffset, kRobotPosOffset);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("hand_root"),
                    X_WH);

  return hand_index;
}

void AddElasticObjectToPlant(MultibodyPlant<double>* plant,
                             SceneGraph<double>* scene_graph,
                             const std::string& object_mesh_file,
                             const double& mesh_scale,
                             const Eigen::VectorXd& q_init_object) {
  // Set up a deformable object.
  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(3e4);
  deformable_config.set_poissons_ratio(0.4);
  deformable_config.set_mass_density(1e3);
  deformable_config.set_stiffness_damping_coefficient(0.01);

  auto elastic_mesh = std::make_unique<Mesh>(object_mesh_file, mesh_scale);
  Eigen::Quaterniond quat_init{q_init_object(0), q_init_object(1),
                               q_init_object(2), q_init_object(3)};
  Vector3d pos_init = q_init_object.tail<3>();
  RigidTransformd X_WO = RigidTransformd(quat_init, pos_init);
  auto elastic_instance = std::make_unique<GeometryInstance>(
      X_WO, std::move(elastic_mesh), "deformable_elastic");

  // Minimally required proximity properties for deformable bodies: A valid
  // Coulomb friction coefficient.
  ProximityProperties deformable_proximity_props;
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial(10.0, {}, surface_friction, &deformable_proximity_props);
  elastic_instance->set_proximity_properties(deformable_proximity_props);

  /* Registration of all deformable geometries ostensibly requires a
  resolution hint parameter that dictates how the shape is tessellated. In the
  case of a `Mesh` shape, the resolution hint is unused because the shape is
  already tessellated. */
  // TODO(xuchenhan-tri): Though unused, we still asserts the resolution hint
  // is positive. Remove the requirement of a resolution hint for meshed
  // shapes.
  const double unused_resolution_hint = 1.0;
  DeformableModel<double>& deformable_model = plant->mutable_deformable_model();
  deformable_model.RegisterDeformableBody(
      std::move(elastic_instance), deformable_config, unused_resolution_hint);
}

}  // namespace dairlib
