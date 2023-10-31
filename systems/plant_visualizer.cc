#include "plant_visualizer.h"

// dairlib includes
#include "common/find_resource.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/framework/output_vector.h"

// drake includes
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"


namespace dairlib {
namespace systems {

using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

PlantVisualizer::PlantVisualizer(const std::string& urdf) : plant_(0.0) {

  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
  scene_graph.set_name("plant_visualizer_scene_graph");

  Parser parser(&plant_, &scene_graph);
  parser.AddModels(FindResourceOrThrow(urdf));
  plant_.Finalize();

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      OutputVector<double>(plant_).size(), 0, plant_.num_positions()
  );
  auto to_pose = builder.AddSystem<MultibodyPositionToGeometryPose<double>>(
      plant_
  );

  builder.Connect(*passthrough, *to_pose);
  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_.get_source_id().value())
  );

  meshcat_ = std::make_shared<Meshcat>();
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat_);

  builder.ExportInput(passthrough->get_input_port(), "x, u, t");
  builder.BuildInto(this);
}

}
}