#include <cmath>

#include "hiking_sim_diagram.h"

#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/cameras/camera_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/primitives/radio_parser.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render_vtk/factory.h"


namespace dairlib::perceptive_locomotion {

using systems::SubvectorPassThrough;
using systems::RobotOutputSender;
using systems::RobotInputReceiver;

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::sensors::PixelType;
using drake::perception::pc_flags::BaseField::kXYZs;
using drake::perception::pc_flags::BaseField::kRGBs;
using drake::perception::DepthImageToPointCloud;

HikingSimDiagram::HikingSimDiagram(const std::string& terrain_yaml,
                                   const std::string& camera_pose_yaml) {

  // magic numbers:
  static constexpr double sim_dt = 5e-4;
  static constexpr double terrain_friction = 0.8;
  static constexpr double actuator_delay = 6e-3;
  static constexpr double actuator_update_period = 2e-3;

  // other constants
  auto contact_solver = drake::multibody::DiscreteContactSolver::kSap;
  const std::string urdf = "examples/Cassie/urdf/cassie_v2_shells.urdf";
  const std::string renderer_name = "hiking_sim_renderer";

  DiagramBuilder<double> builder;

  // Add the plant
  scene_graph_ = builder.AddSystem<SceneGraph<double>>();
  plant_ = builder.AddSystem<MultibodyPlant<double>>(sim_dt);
  plant_->set_discrete_contact_solver(contact_solver);

  multibody::AddSteppingStonesToSimFromYaml(
      plant_, scene_graph_, terrain_yaml, terrain_friction
  );
  AddCassieMultibody(plant_, scene_graph_, true, urdf, true, true);
  plant_->Finalize();

  scene_graph_->AddRenderer(renderer_name,
                            drake::geometry::MakeRenderEngineVtk(
                            drake::geometry::RenderEngineVtkParams()));


  // extra sensor and motor models
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      plant_->get_actuation_input_port().size() + 1, 0,
      plant_->get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          actuator_update_period,
          std::round(actuator_delay / actuator_update_period),
          plant_->num_actuators() + 1
      );
  auto state_sender = builder.AddSystem<RobotOutputSender>(*plant_, true);

  const auto& cassie_motor = AddMotorModel(&builder, *plant_);

  const auto& sensor_aggregator = AddImuAndAggregator(
      &builder, *plant_, cassie_motor.get_output_port()
  );

  // camera model
  const auto cam_transform = camera::ReadCameraPoseFromYaml(camera_pose_yaml);
  const auto& [color_camera, depth_camera] = camera::MakeDairD455CameraModel(
      renderer_name, camera::D455ImageSize::k424x240
  );
  const auto parent_body_id = plant_->GetBodyFrameIdIfExists(
      plant_->GetFrameByName("pelvis").body().index()
  );
  const auto camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
          parent_body_id.value(), cam_transform, color_camera, depth_camera
  );

  // Add radio just in case
  const auto radio_parser = builder.AddSystem<systems::RadioParser>();

  // wire it up
  builder.Connect(
      discrete_time_delay->get_output_port(), passthrough->get_input_port()
  );
  builder.Connect(
      passthrough->get_output_port(), cassie_motor.get_input_port_command()
  );
  builder.Connect(
      cassie_motor.get_output_port(), plant_->get_actuation_input_port()
  );
  builder.Connect(
      plant_->get_state_output_port(), state_sender->get_input_port_state()
  );
  builder.Connect(
      plant_->get_state_output_port(), cassie_motor.get_input_port_state()
  );
  builder.Connect(
      cassie_motor.get_output_port(), state_sender->get_input_port_effort()
  );
  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value())
  );
  builder.Connect(
      scene_graph_->get_query_output_port(),
      plant_->get_geometry_query_input_port()
  );
  builder.Connect(
      scene_graph_->get_query_output_port(), camera->query_object_input_port()
  );
  builder.Connect(
      radio_parser->get_output_port(), sensor_aggregator.get_input_port_radio()
  );

  // export the output ports
  input_port_control_ = builder.ExportInput(
      discrete_time_delay->get_input_port(), "u, t"
  );
  input_port_radio_ = builder.ExportInput(
      radio_parser->get_input_port(), "radio channels"
  );
  output_port_state_ = builder.ExportOutput(
      state_sender->get_output_port(), "x, u, t"
  );
  output_port_cassie_out_ = builder.ExportOutput(
      sensor_aggregator.get_output_port(), "lcmt_cassie_out"
  );
  output_port_lcm_radio_ = builder.ExportOutput(
      radio_parser->get_output_port(), "lcmt_radio_out"
  );

  builder.BuildInto(this);
  this->set_name("hiking_sim_diagram");
  DrawAndSaveDiagramGraph(*this, "../hiking_sim_diagram");


}


}
