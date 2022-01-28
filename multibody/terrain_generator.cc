#include "multibody/terrain_generator.h"
#include <random>
#include <ctime>

using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::Box;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::math::RigidTransformd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using std::vector;

namespace dairlib::multibody {

void addRandomTerrain(drake::multibody::MultibodyPlant<double> *plant,
                drake::geometry::SceneGraph<double> *scene_graph,
                double mu_static, double mu_kinetic,
                Eigen::Vector3d normal_W) {
  addFlatTerrain<double>(plant, scene_graph, mu_static, mu_kinetic, normal_W);

  int nboxes = 10;
  std::vector<RigidTransformd> cube_poses = GenerateRandomPoses(nboxes);
  std::vector<Box> boxes = GenerateRandomBoxes(nboxes);
  std::vector<CoulombFriction<double>> frictions = GenerateRandomFrictions(nboxes);
  for (int i = 0; i < nboxes; i++) {
    plant->RegisterCollisionGeometry(
        plant->world_body(), cube_poses.at(i),
        boxes.at(i), "box_collision_"+std::to_string(i),
        frictions.at(i));
    plant->RegisterVisualGeometry(
        plant->world_body(), cube_poses.at(i),
        boxes.at(i), "box_visual_"+std::to_string(i),
        Vector4d(0.7, 0.1, 0.1, 1.0));
  }
}

std::vector<RigidTransformd> GenerateRandomPoses(int n) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(0.0, 1.0);
  std::default_random_engine r(time(NULL));

  std::vector<RigidTransformd> poses;
  for (int i = 0 ; i < n; i++) {
    drake::math::RollPitchYawd rpy(d(r), d(r), d(r));
    Vector3d trans(d(r), d(r), d(r));
    poses.emplace_back(RigidTransformd(rpy, trans));
  }
  return poses;
}

std::vector<Box> GenerateRandomBoxes(int n) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(0.0, 1.0);
  std::default_random_engine r(time(NULL));;

  std::vector<Box> boxes;
  for (int i = 0; i < n; i++) {
    double s = d(r);
    boxes.emplace_back(Box(s, s, s));
  }
  return boxes;
}

std::vector<CoulombFriction<double>> GenerateRandomFrictions(int n) {
  DRAKE_ASSERT(n >= 0);
  std::uniform_real_distribution<double> d(0.0, 1.0);
  std::default_random_engine r(time(NULL));;

  std::vector<CoulombFriction<double>> friction;
  for (int i = 0; i < n; i++) {
    double s = d(r);
    friction.emplace_back(CoulombFriction<double>(s, s));
  }
  return friction;
}

}