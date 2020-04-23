#include "multibody/multipose_visualizer.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"

using std::string;
using Eigen::MatrixXd;
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;

namespace dairlib {
namespace multibody {

MultiposeVisualizer::MultiposeVisualizer(string model_file, int num_poses) : 
    num_poses_(num_poses) {
  DiagramBuilder<double> builder;

  SceneGraph<double>* scene_graph{};
  std::tie(plant_, scene_graph) =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  Parser parser(plant_, scene_graph);

  // Add num_poses copies of the plant, giving each a unique name
  for (int i = 0; i < num_poses_; i++) {
    auto index = 
        parser.AddModelFromFile(model_file, "model[" + std::to_string(i) + "]");
    model_indices_.push_back(index);
  }

  plant_->Finalize();
  drake::geometry::ConnectDrakeVisualizer(&builder, *scene_graph, lcm);
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  drake::geometry::DispatchLoadMessage(*scene_graph, lcm);
}

void MultiposeVisualizer::DrawPoses(MatrixXd poses) {

  // Set positions for individual instances
  auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_,
      diagram_context_.get());
  for (int i = 0; i < num_poses_; i++) {
    plant_->SetPositions(&plant_context, model_indices_.at(i),
        poses.block(0, i, plant_->num_positions(model_indices_.at(i)), 1));
  }

  // Publish diagram
  diagram_->Publish(*diagram_context_);
}

}  // namespace multibody
}  // namespace dairlib
