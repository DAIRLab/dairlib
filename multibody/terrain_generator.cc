#include "multibody/terrain_generator.h"
#include <random>
#include <ctime>

using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::Box;
using drake::geometry::Ellipsoid;
using drake::geometry::SceneGraph;
using drake::geometry::ProximityProperties;
using drake::geometry::AddCompliantHydroelasticPropertiesForHalfSpace;
using drake::geometry::AddContactMaterial;
using drake::multibody::MultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::math::RigidTransformd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using std::vector;

namespace dairlib::multibody {



void addFlatHydroelasticTerrain(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    double mu_static, double mu_kinetic, Eigen::Vector3d normal_W) {

  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  Eigen::Vector3d point_W(0, 0, 0);
  drake::multibody::CoulombFriction<double> friction(mu_static, mu_kinetic);

  // A half-space for the ground geometry.
  const drake::math::RigidTransformd X_WG(
      HalfSpace::MakePose(normal_W, point_W));

  drake::geometry::ProximityProperties ground_props;
  AddCompliantHydroelasticPropertiesForHalfSpace(1.0, 5e7, &ground_props);
  AddContactMaterial(1.25, {}, friction, &ground_props);

  plant->RegisterCollisionGeometry(
      plant->world_body(), X_WG, HalfSpace(),
      "collision", std::move(ground_props));

  // Add visual for the ground.
  plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                "visual");
}

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                      TerrainConfig terrain_config) {
  addFlatHydroelasticTerrain(
      plant, scene_graph, terrain_config.mu_flat,
      terrain_config.mu_flat, terrain_config.normal);

  int nboxes = 20;
  std::vector<RigidTransformd> cube_poses = GenerateRandomPoses(
      nboxes, terrain_config.clearing_radius, terrain_config.rpy_bounds);
  std::vector<Ellipsoid> ellipsoids = GenerateRandomEllipsoids(
      nboxes, terrain_config.min_cube_size, terrain_config.max_cube_size);
  std::vector<CoulombFriction<double>> frictions = GenerateRandomFrictions(
      nboxes, terrain_config.mu_cube_min, terrain_config.mu_cube_max);

  for (int i = 0; i < nboxes; i++) {
    plant->RegisterCollisionGeometry(
        plant->world_body(), cube_poses.at(i),
        ellipsoids.at(i), "box_collision_"+std::to_string(i),
        frictions.at(i));
    plant->RegisterVisualGeometry(
        plant->world_body(), cube_poses.at(i),
        ellipsoids.at(i), "box_visual_"+std::to_string(i),
        Vector4d(0.7, 0.1, 0.1, 1.0));
  }
}

std::vector<RigidTransformd> GenerateRandomPoses(
    int n, double clearing_radius, Vector3d rpy_bounds) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(-1.0, 1.0);
  std::default_random_engine r(time(NULL));

  std::vector<RigidTransformd> poses;
  for (int i = 0 ; i < n; i++) {
    drake::math::RollPitchYawd rpy(rpy_bounds(0)*d(r),
                                   rpy_bounds(1)*d(r),
                                   rpy_bounds(2)*d(r));

    Vector3d trans(10*d(r), d(r), 0);
    while (trans.norm() < clearing_radius) {
      trans*=2.0;
    }
    poses.emplace_back(RigidTransformd(rpy, trans));
  }
  return poses;
}

std::vector<Box> GenerateRandomBoxes(int n, double s_min, double s_max) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(s_min, s_max);
  std::default_random_engine r(time(NULL));;

  std::vector<Box> boxes;
  for (int i = 0; i < n; i++) {
    double s = d(r);
    boxes.emplace_back(Box(s, s, s));
  }
  return boxes;
}

std::vector<Ellipsoid> GenerateRandomEllipsoids(int n, double s_min, double s_max) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(s_min, s_max);
  std::default_random_engine r(time(NULL));;

  std::vector<Ellipsoid> ellip;
  for (int i = 0; i < n; i++) {
    ellip.emplace_back(Ellipsoid(d(r), d(r), d(r)));
  }
  return ellip;
}

std::vector<CoulombFriction<double>> GenerateRandomFrictions(
    int n, double mu_min, double mu_max) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(mu_min, mu_max);
  std::default_random_engine r(time(NULL));;

  std::vector<CoulombFriction<double>> friction;
  for (int i = 0; i < n; i++) {
    double s = d(r);
    friction.emplace_back(CoulombFriction<double>(s, s));
  }
  return friction;
}

}