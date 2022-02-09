#include "multibody/terrain_generator.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "common/find_resource.h"
#include <random>
#include <ctime>
#include <fstream>

using drake::VectorX;
using drake::geometry::HalfSpace;
using drake::geometry::Box;
using drake::geometry::Ellipsoid;
using drake::geometry::SceneGraph;
using drake::geometry::ProximityProperties;
using drake::geometry::SurfaceTriangle;
using drake::geometry::TriangleSurfaceMesh;
using drake::geometry::AddCompliantHydroelasticPropertiesForHalfSpace;
using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::AddRigidHydroelasticProperties;
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


void writeTriangleMeshToObj(const TriangleSurfaceMesh<double> &mesh,
                            std::string filename) {
  std::ofstream ofs;
  ofs.open(filename, std::ios::out);
  ofs << "# Random Terrain Generated for Cassie Simulation\n# Vertices:\n";
  for (auto& v : mesh.vertices()) {
    ofs << "v " << v(0) << " " << v(1) << " " << v(2) << "\n";
  }
  ofs << "# Faces:\n";
  for (auto& t : mesh.triangles()) {
    ofs << "f " << t.vertex(0)+1 << " " << t.vertex(1)+1 << " " << t.vertex(2)+1 << "\n";
  }
  ofs.close();
}

std::string makeRandomHeightMap(int nx, int ny,double x_resolution,
                                double y_resolution, Vector4d freq_scales,
                                Vector3d normal) {
  srand((unsigned int) time(0));
  Vector4d x_amplitudes = (Vector4d::Random().array() * freq_scales.array()).matrix();
  Vector4d y_amplitudes = (Vector4d::Random().array() * freq_scales.array()).matrix();

  VectorXd xvals = VectorXd::LinSpaced(nx,-nx*x_resolution, nx*x_resolution);
  VectorXd yvals = VectorXd::LinSpaced(ny,-ny*y_resolution, ny*y_resolution);
  MatrixXd height = MatrixXd::Zero(ny, nx);

  double omega_x = 2*M_2_PI / (nx * x_resolution);
  double omega_y = 2*M_2_PI / (ny * y_resolution);

  for (int i = 0; i < x_amplitudes.size(); i++) {
    MatrixXd freq = y_amplitudes(i) * Eigen::sin((i+1)*omega_y*yvals.array()).matrix() *
        (x_amplitudes(i) * Eigen::sin((i+1)*omega_x*xvals.array()).matrix().transpose());
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
      int ul = j*ny + i;
      int bl = ul+ny;
      int ur = ul+1;
      int br = bl+1;
      t.emplace_back(SurfaceTriangle(ul, bl, ur));
      t.emplace_back(SurfaceTriangle(bl, br, ur));
    }
  }

  TriangleSurfaceMesh<double>mesh(std::move(t), std::move(v));
//  mesh.ReverseFaceWinding();
  std::string filename = "examples/Cassie/terrains/tmp/terrain_mesh.obj";
  writeTriangleMeshToObj(mesh, filename);
  return filename;
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
                      const TerrainConfig& terrain_config) {

  if (!plant->geometry_source_is_registered()) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
  }

  auto mesh_file = makeRandomHeightMap(terrain_config.xbound,
                                            terrain_config.ybound,
                                            terrain_config.mesh_res,
                                            terrain_config.mesh_res,
                                            terrain_config.freq_scales,
                                            terrain_config.normal);

  ProximityProperties obstacle_props;
  AddRigidHydroelasticProperties(0.02, &obstacle_props);
  CoulombFriction<double> friction(terrain_config.mu_flat, terrain_config.mu_flat);
  AddContactMaterial(1.5, 9100, friction, &obstacle_props);

  drake::geometry::Mesh mesh(FindResourceOrThrow(mesh_file));
  plant->RegisterCollisionGeometry(
        plant->world_body(), RigidTransformd::Identity(),
        mesh, "ground_mesh",
        std::move(obstacle_props));
  plant->RegisterVisualGeometry(
        plant->world_body(),  RigidTransformd::Identity(),
        mesh, "ground_mesh",
        Vector4d(1, 1, 1, 1.0));

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