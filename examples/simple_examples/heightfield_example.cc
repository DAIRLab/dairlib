#include <iostream>

#include <gflags/gflags.h>
#include <filesystem>
#include <fstream>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/framework/diagram_builder.h"
#include "common/find_resource.h"

namespace dairlib {

DEFINE_double(realtime_rate, 1.0, "target realtime rate for simulator");

using drake::systems::DiagramBuilder;
using drake::multibody::MultibodyPlant;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::math::RigidTransform;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::HeightField;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SurfaceTriangle;
using drake::geometry::TriangleSurfaceMesh;
using drake::multibody::CoulombFriction;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

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

std::string SaveHeightFieldToMesh(HeightField& h) {
  std::vector<Vector3d> v;
  std::vector<SurfaceTriangle> t;

  for (int i = 0; i < h.nx(); i++) {
    for(int j = 0; j < h.ny(); j++) {
      v.emplace_back(Vector3d(h.x_grid()(i), h.y_grid()(j), h.heights()(i,j)));
    }
  }
  for (int i = 0; i < h.nx()-1; i++) {
    for (int j = 0; j < h.ny()-1; j++) {
      int ul = i*h.nx() + j;
      int bl = ul+h.nx();
      int ur = ul+1;
      int br = bl+1;
      t.emplace_back(SurfaceTriangle(ul, bl, ur));
      t.emplace_back(SurfaceTriangle(bl, br, ur));
    }
  }
  TriangleSurfaceMesh<double> mesh(std::move(t), std::move(v));
  std::filesystem::remove_all("examples/simple_examples/tmp");
  std::filesystem::create_directory("examples/simple_examples/tmp");

  std::string filename = "examples/simple_examples/tmp/terrain_mesh" +
      std::to_string(time(0)) + ".obj";
  writeTriangleMeshToObj(mesh, filename);
  return filename;
}

int do_main(int argc, char* argv[]) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = 0.0001;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  plant.AddRigidBody("sphere", SpatialInertia<double>(
      1.0, Vector3d::Zero(), UnitInertia<double>(1.0, 1.0, 1.0)));
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  builder.Connect(plant.get_geometry_poses_output_port(),
                  scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  double radius = 0.05;
  plant.RegisterCollisionGeometry(
      plant.GetBodyByName("sphere"), RigidTransform<double>::Identity(),
      Sphere(0.1), "sphere_collide", CoulombFriction<double>(0.8, 0.8));
  plant.RegisterVisualGeometry(
      plant.GetBodyByName("sphere"), RigidTransform<double>::Identity(),
      Sphere(0.05), "sphere_visual", Vector4d(1.0, 0.0, 0.0, 1.0));

  int n = 10;
  MatrixXd h = MatrixXd::Zero(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      auto nd = static_cast<double>(n);
      h(i, j) =  ((i - nd/2)*(i - nd/2) +  (j - nd/2)*(j-nd/2)) / (nd*nd);
    }
  }
  HeightField hfield(h, 5.0, 5.0, 10.0);
  plant.RegisterCollisionGeometry(
      plant.world_body(), RigidTransform<double>::Identity(), hfield,
      "hfield_collide", CoulombFriction<double>(0.8, 0.8));

  auto mesh_file = SaveHeightFieldToMesh(hfield);
  drake::geometry::Mesh mesh(FindResourceOrThrow(mesh_file));

  plant.RegisterVisualGeometry(
      plant.world_body(), RigidTransform<double>::Identity(),
      mesh, "hfield_visual", Vector4d(0.0, 0.5, 1.0, 1.0));
  plant.Finalize();

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  DrakeVisualizer<double>::DispatchLoadMessage(scene_graph, lcm);
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, diagram_context.get());


  VectorXd q_init = VectorXd::Zero(7);
  q_init << 1, 0, 0, 0, -1.0, -1.0, 5.0;
  std::cout << "here4\n";

  plant.SetPositions(&plant_context,q_init);
  drake::systems::Simulator<double>
      simulator(*diagram, std::move(diagram_context));
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(10.0);

  return 0;

}
}

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}