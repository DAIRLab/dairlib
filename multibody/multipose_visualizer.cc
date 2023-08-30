#include "multibody/multipose_visualizer.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/meshcat_visualizer_params.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"

using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using Eigen::MatrixXd;
using std::string;

namespace dairlib {
namespace multibody {

MultiposeVisualizer::MultiposeVisualizer(string model_file, int num_poses,
                                         string weld_frame_to_world)
    : MultiposeVisualizer(model_file, num_poses,
                          Eigen::VectorXd::Constant(num_poses, 1.0)) {}

MultiposeVisualizer::MultiposeVisualizer(string model_file, int num_poses,
                                         double alpha_scale,
                                         string weld_frame_to_world)
    : MultiposeVisualizer(model_file, num_poses,
                          Eigen::VectorXd::Constant(num_poses, alpha_scale)) {}

MultiposeVisualizer::MultiposeVisualizer(string model_file, int num_poses,
                                         const Eigen::VectorXd& alpha_scale,
                                         string weld_frame_to_world)
    : num_poses_(num_poses) {
  DRAKE_DEMAND(num_poses == alpha_scale.size());
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
    if (!weld_frame_to_world.empty()) {
      plant_->WeldFrames(
          plant_->world_frame(),
          plant_->GetFrameByName(weld_frame_to_world, model_indices_.at(i)),
          drake::math::RigidTransform<double>(Eigen::Vector3d::Zero()));
    }
  }

  plant_->Finalize();

  // Adjust transparency alpha values
  const auto& inspector = scene_graph->model_inspector();

  // Model instances 0 and 1 are reserved, plants are therefore 2+
  for (int i = 2; i < plant_->num_model_instances(); i++) {
    auto body_indices =
        plant_->GetBodyIndices(drake::multibody::ModelInstanceIndex(i));
    for (const auto& body_index : body_indices) {
      const auto& geometry_ids =
          plant_->GetVisualGeometriesForBody(plant_->get_body(body_index));
      for (const auto& geometry_id : geometry_ids) {
        const auto& prop = inspector.GetIllustrationProperties(geometry_id);
        if (prop && prop->HasProperty("phong", "diffuse")) {
          drake::geometry::IllustrationProperties new_props(*prop);
          auto phong =
              prop->GetProperty<drake::geometry::Rgba>("phong", "diffuse");

          // Scale alpha value, threshold to [0,1]
          double new_alpha = alpha_scale(i - 2) * phong.a();
          new_alpha = std::max(new_alpha, 0.0);
          new_alpha = std::min(new_alpha, 1.0);
          phong.set(phong.r(), phong.g(), phong.b(), new_alpha);

          new_props.UpdateProperty("phong", "diffuse", phong);
          scene_graph->AssignRole(plant_->get_source_id().value(), geometry_id,
                                  new_props,
                                  drake::geometry::RoleAssign::kReplace);
        }
      }
    }
  }

  drake::geometry::MeshcatVisualizerParams params;
  params.publish_period = 1.0/60.0;
  meshcat_ = std::make_shared<drake::geometry::Meshcat>();
  meshcat_visualizer_ = &drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
      &builder, *scene_graph, meshcat_, std::move(params));

  DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph, lcm);
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  DrakeVisualizer<double>::DispatchLoadMessage(*scene_graph, lcm);
}

void MultiposeVisualizer::DrawPoses(MatrixXd poses) {
  // Set positions for individual instances
  auto& plant_context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
  for (int i = 0; i < num_poses_; i++) {
    plant_->SetPositions(
        &plant_context, model_indices_.at(i),
        poses.block(0, i, plant_->num_positions(model_indices_.at(i)), 1));
  }

  // Publish diagram
  diagram_->ForcedPublish(*diagram_context_);
}

}  // namespace multibody
}  // namespace dairlib
