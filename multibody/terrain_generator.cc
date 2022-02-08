#include "multibody/terrain_generator.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include <random>
#include <ctime>

using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::Box;
using drake::geometry::Ellipsoid;
using drake::geometry::SceneGraph;
using drake::geometry::ProximityProperties;
using drake::geometry::SurfaceTriangle;
using drake::geometry::AddCompliantHydroelasticPropertiesForHalfSpace;
using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddContactMaterial;
using drake::multibody::MultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::math::RigidTransformd;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Vector4d;
using std::vector;

namespace dairlib::multibody {

drake::geometry::TriangleSurfaceMesh<double>
makeRandomHeightMap(int nx, int ny,double x_resolution, double y_resolution,
                    Vector3d normal) {
  Vector4d freq_x = Vector4d::Random();
  Vector4d freq_y = Vector4d::Random();

  VectorXd xvals = VectorXd::LinSpaced(nx,-nx*x_resolution, nx*x_resolution);
  VectorXd yvals = VectorXd::LinSpaced(ny,-ny*y_resolution, ny*y_resolution);
  MatrixXd height = MatrixXd::Zero(ny, nx);

  for (int i = 0; i < freq_x.size(); i++) {
    MatrixXd freq = (freq_y(0) * yvals.array().sin()).matrix() *
        (freq_x(0) * xvals.array().sin().transpose()).matrix();
    height += freq;
  }
  height.rowwise() += (normal(0) / normal(2)) * xvals.transpose();
  height.colwise() += (normal(1) / normal(2)) * yvals;

  std::vector<Vector3d> v;
  std::vector<SurfaceTriangle> t;
  for (int i = 0; i < nx; i++) {
    for(int j = 0; j < ny; j++) {
      v.emplace_back(Vector3d(xvals(i), yvals(j), height(j,i)));
    }
  }
  for (int i = 0; i < nx-1; i++) {
    for (int j = 0; j < ny-1; j++) {
      int ul = j*nx + i;
      int bl = ul+nx;
      int ur = ul+1;
      int br = ul+nx+1;
      t.emplace_back(SurfaceTriangle(ul, bl, ur));
      t.emplace_back(SurfaceTriangle(bl, br, ur));
    }
  }

  drake::geometry::TriangleSurfaceMesh<double>
      mesh(std::move(t), std::move(v));
  return mesh;
}

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

  ProximityProperties ground_props;
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

  auto mesh = makeRandomHeightMap(terrain_config.xbound,
                                            terrain_config.ybound,
                                            terrain_config.mesh_res,
                                            terrain_config.mesh_res,
                                            terrain_config.normal);
  addFlatHydroelasticTerrain(plant, scene_graph,
                             terrain_config.mu_flat,
                             terrain_config.mu_flat,
                             terrain_config.normal);

  ProximityProperties obstacle_props;
  AddCompliantHydroelasticProperties(
      0.1, 5e7, &obstacle_props);
  CoulombFriction<double> friction(terrain_config.mu_flat, terrain_config.mu_flat);
  AddContactMaterial(1.25, {}, friction, &obstacle_props);


//  plant->RegisterCollisionGeometry(
//        plant->world_body(), RigidTransformd::Identity(),
//        mesh, "ground_mesh",
//        std::move(obstacle_props));
//  plant->RegisterVisualGeometry(
//        plant->world_body(),  RigidTransformd::Identity(),
//        mesh, "ground_mesh",
//        Vector4d(1, 1, 1, 1.0));
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