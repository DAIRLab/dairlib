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
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"


namespace dairlib {
namespace systems {

using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::math::RigidTransformd;
using drake::math::RotationMatrixd;

using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;

using drake::geometry::SceneGraph;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;

using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::DiscreteValues;
using drake::systems::ConstantVectorSource;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

MeshcatCameraManager::MeshcatCameraManager(const drake::multibody::MultibodyPlant<
    double> &plant, const drake::multibody::Frame<double> &camera_track_frame,
    std::shared_ptr<drake::geometry::Meshcat> meshcat) :
    plant_(plant),
    meshcat_(meshcat),
    camera_track_frame_(camera_track_frame) {

  plant_context_ = plant_.CreateDefaultContext();

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)).get_index();

  input_port_cam_pos_ = DeclareVectorInputPort("cam_pos", 3).get_index();

  DeclarePerStepDiscreteUpdateEvent(&MeshcatCameraManager::UpdateMeshcat);

  meshcat_->Delete("/Background");

}

drake::systems::EventStatus MeshcatCameraManager::UpdateMeshcat(
    const Context<double> &context,
    DiscreteValues<double> *discrete_state) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));
  Vector3d cam_pos_local =
      EvalVectorInput(context, input_port_cam_pos_)->get_value();

  VectorXd q = robot_output->GetPositions();
  plant_.SetPositions(plant_context_.get(), q);
  Vector3d body_pos_in_world = camera_track_frame_.CalcPoseInWorld(
      *plant_context_).translation() - 0.4 * Vector3d::UnitZ();

  meshcat_->SetCameraPose(body_pos_in_world + cam_pos_local, body_pos_in_world);

  // Set the lighting positions
  meshcat_->SetTransform(
      "/Lights/SpotLight/<object>",
      RigidTransformd(
          RotationMatrixd{}, body_pos_in_world + Vector3d(0.0, -5.0, 1.0)));
  meshcat_->SetTransform(
      "/Lights/PointLightPositiveX/<object>",
      RigidTransformd(
          RotationMatrixd{}, body_pos_in_world + Vector3d(2.0, 0.0, 2.0)));
  meshcat_->SetTransform(
      "/Lights/PointLightNegativeX/<object>",
      RigidTransformd(
          RotationMatrixd{}, body_pos_in_world + Vector3d(-2.0, 0.0, 2.0)));

  meshcat_->SetProperty("/Lights/PointLightPositiveX/<object>", "castShadow", true);
  meshcat_->SetProperty("/Lights/SpotLight/<object>", "castShadow", true);
  meshcat_->SetProperty("/Lights/PointLightPositiveX/<object>", "intensity", 100.0);
  meshcat_->SetProperty("/Lights/SpotLight/<object>", "intensity", 40.0);

  return EventStatus::Succeeded();
}


PlantVisualizer::PlantVisualizer(
    const std::string& urdf, const std::string& body_to_track) : plant_(0.0) {

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

  const drake::multibody::Frame<double>& frame = body_to_track.empty() ?
      plant_.world_frame() : plant_.GetBodyByName(body_to_track).body_frame();

  if (not body_to_track.empty()) {
    auto cam_manager = builder.AddSystem<MeshcatCameraManager>(plant_, frame, meshcat_);
    auto cam_position = builder.AddSystem<ConstantVectorSource<double>>(
        Vector3d(0, -2.5, 0.1)
    );


    builder.ConnectInput("x, u, t", cam_manager->get_input_port_state());
    builder.Connect(
        cam_position->get_output_port(),
        cam_manager->get_input_port_cam_pos()
    );
  }


  builder.BuildInto(this);
}

}
}