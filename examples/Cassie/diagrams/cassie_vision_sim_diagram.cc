#include "cassie_vision_sim_diagram.h"

#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/zero_order_hold.h>
#include <drake/common/yaml/yaml_io.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/curriculum_terrain_config.h"
#include "multibody/boxy_height_map.h"
#include "multibody/cube_height_map.h"
#include "systems/framework/geared_motor.h"
#include "systems/primitives/radio_parser.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/cameras/camera_utils.h"

#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/common/yaml/yaml_io.h"
#include "multibody/curriculum_height_map.h"

namespace dairlib {
namespace examples {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::math::RotationMatrix;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

CassieVisionSimDiagram::CassieVisionSimDiagram(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
    const std::string& urdf, bool visualize, double mu, double map_yaw,
    const Eigen::Vector3d& normal, const std::string& map_config_fname) {

  cam_transform_ =
      drake::math::RigidTransform<double>(
          camera::MakeXZAlignedCameraRotation(-0.85*M_PI/2),
          Eigen::Vector3d(0.175, 0, 0.15));

  DiagramBuilder<double> builder;
  scene_graph_ = builder.AddSystem<SceneGraph>();
  scene_graph_->set_name("scene_graph");

  const std::string renderer_name = "multibody_renderer_vtk";
  scene_graph_->AddRenderer(renderer_name,
      drake::geometry::MakeRenderEngineVtk(
          drake::geometry::RenderEngineVtkParams()));

  plant_ = builder.AddSystem(std::move(plant));
  AddCassieMultibody(plant_, scene_graph_, true, urdf, true, true);
  auto curriculum_info = drake::yaml::LoadYamlFile<multibody::CurriculumTerrainInfo>(map_config_fname);

  hmap_ = multibody::CurriculumHeightMap(normal, 5, 0.5, map_yaw, mu, curriculum_info);
//  hmap_ = multibody::BoxyHeightMap::MakeRandomMap(normal, map_yaw, mu, map_height);
//  multibody::CubeHeightMap hmap =
//      multibody::CubeHeightMap::MakeRandomMap(normal, map_yaw, mu);
  hmap_.AddHeightMapToPlant(plant_, scene_graph_);
  plant_->RegisterVisualGeometry(plant_->GetBodyByName("pelvis"),
                                 cam_transform_,
                                 drake::geometry::Box(0.15, 0.025, 0.025),
                                 "realsense_box");
  plant_->Finalize();

  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(*plant_);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant_->get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          actuator_update_rate, actuator_delay / actuator_update_rate,
          plant_->num_actuators() + 1);
  auto state_sender =
      builder.AddSystem<systems::RobotOutputSender>(*plant_, true);

  auto constant_source =
      builder.AddSystem<drake::systems::ConstantVectorSource>(
          VectorXd::Zero(10));
  auto input_zero_order_hold = builder.AddSystem<drake::systems::ZeroOrderHold>(
      0.001, plant_->num_actuators() + 1);
  sensor_aggregator_ = &AddImuAndAggregator(&builder, *plant_,
                                            constant_source->get_output_port());

  const auto& cassie_motor = AddMotorModel(&builder, *plant_);
  auto radio_parser = builder.AddSystem<systems::RadioParser>();

  // Add camera model
  const auto& [color_camera, depth_camera] =
  camera::MakeDairD455CameraModel(renderer_name, camera_type_);
  const std::optional<drake::geometry::FrameId> parent_body_id =
      plant_->GetBodyFrameIdIfExists(
          plant_->GetFrameByName("pelvis").body().index());


  auto camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
      parent_body_id.value(), cam_transform_, color_camera, depth_camera);


  // connect leaf systems
  builder.Connect(input_receiver->get_output_port(),
                  input_zero_order_hold->get_input_port());
  builder.Connect(input_zero_order_hold->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  cassie_motor.get_input_port_command());
  builder.Connect(cassie_motor.get_output_port(),
                  plant_->get_actuation_input_port());
  builder.Connect(plant_->get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(plant_->get_state_output_port(),
                  cassie_motor.get_input_port_state());
  builder.Connect(cassie_motor.get_output_port(),
                  state_sender->get_input_port_effort());
  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());
  builder.Connect(radio_parser->get_output_port(),
                  sensor_aggregator_->get_input_port_radio());
  builder.Connect(scene_graph_->get_query_output_port(),
                  camera->query_object_input_port());

  builder.ExportInput(input_receiver->get_input_port(), "lcmt_robot_input");
  builder.ExportInput(radio_parser->get_input_port(), "raw_radio");
  builder.ExportOutput(state_sender->get_output_port(0), "lcmt_robot_output");
  builder.ExportOutput(sensor_aggregator_->get_output_port(0),
                       "lcmt_cassie_out");
  builder.ExportOutput(camera->depth_image_32F_output_port(), "camera_output");

  if (visualize) {
    DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph_);
  }
  builder.BuildInto(this);
  this->set_name("cassie_sim_diagram");
//  DrawAndSaveDiagramGraph(*this);
}

double CassieVisionSimDiagram::get_height_at(double x, double y) {
  return hmap_.GetHeightInWorld({x, y});
}

}  // namespace examples
}  // namespace dairlib