#include "multibody/terrain_generator.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "common/find_resource.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/proximity_properties.h"
#include <random>
#include <ctime>
#include <fstream>
#include <filesystem>

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

std::string makeRandomHeightMapAsMesh(int nx, int ny, double x_resolution,
                                      double y_resolution, int n_freq,
                                      Eigen::VectorXd frequencies,
                                      Eigen::VectorXd freq_scales,
                                      Eigen::Vector3d normal) {
  srand((unsigned int) time(0));

  DRAKE_DEMAND(frequencies.size() == n_freq);
  DRAKE_DEMAND(freq_scales.size() == n_freq);

  VectorXd xvals = VectorXd::LinSpaced(nx,-nx*x_resolution, nx*x_resolution);
  VectorXd yvals = VectorXd::LinSpaced(ny,-ny*y_resolution, ny*y_resolution);
  MatrixXd height = MatrixXd::Zero(ny, nx);

  for (int i = 0; i < n_freq; i++) {
    double alpha_x = x_resolution / (x_resolution + 0.16/frequencies(i));
    double alpha_y = y_resolution / (y_resolution + 0.16/frequencies(i));
    MatrixXd freq =
        freq_scales(i) * MatrixXd ::Random(ny, nx);
    MatrixXd freq_filtered = MatrixXd::Zero(ny, nx);
    for (int j = 0; j < ny; j++) {
      Eigen::RowVectorXd filt_row = Eigen::RowVectorXd::Zero(nx);
      filt_row(0) = freq(j, 0);
      for (int k = 1; k < nx; k++) {
        filt_row(k) = alpha_x * freq(j, k) + (1-alpha_x) * filt_row(k-1);
      }
      freq_filtered.row(j) = (j == 0) ? filt_row :
          alpha_y * filt_row + (1-alpha_y) * freq_filtered.row(j-1);
    }
    height += freq_filtered;
  }
  height.rowwise() += (normal(0) / normal(2)) * xvals.transpose();
  height.colwise() += (normal(1) / normal(2)) * yvals;
  height -= MatrixXd::Constant(ny, nx, height((ny+1)/2, (nx+1)/2));
  std::vector<Vector3d> v;
  std::vector<SurfaceTriangle> t;
  for (int i = 0; i < nx; i++) {
    for(int j = 0; j < ny; j++) {
      v.push_back(Vector3d(xvals(i), yvals(j), height(j,i)));
    }
  }
  for (int j = 0; j < nx-1; j++) {
    for (int i = 0; i < ny-1; i++) {
      int ul = j*ny + i;
      int bl = ul+ny;
      int ur = ul+1;
      int br = bl+1;
      t.push_back(SurfaceTriangle(ul, bl, ur));
      t.push_back(SurfaceTriangle(bl, br, ur));
    }
  }

  TriangleSurfaceMesh<double>mesh(std::move(t), std::move(v));
//  mesh.ReverseFaceWinding();

  std::filesystem::remove_all("examples/Cassie/terrains/tmp");
  std::filesystem::create_directory("examples/Cassie/terrains/tmp");
  std::string filename = "examples/Cassie/terrains/tmp/terrain_mesh" +
      std::to_string(time(0)) + ".obj";
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

  auto mesh_file = makeRandomHeightMapAsMesh(terrain_config.x_length,
                                             terrain_config.y_length,
                                             terrain_config.x_resolution,
                                             terrain_config.y_resolution,
                                             terrain_config.n_freq_components,
                                             terrain_config.freqs,
                                             terrain_config.freq_amps,
                                             terrain_config.normal);

  ProximityProperties obstacle_props;
  AddRigidHydroelasticProperties(0.02, &obstacle_props);
  CoulombFriction<double> friction(terrain_config.mu, terrain_config.mu);
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
}