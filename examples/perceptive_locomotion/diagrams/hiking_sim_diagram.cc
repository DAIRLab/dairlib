#include <cmath>

#include "hiking_sim_diagram.h"

#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/stepping_stone_utils.h"
#include "systems/system_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/perception/camera_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/primitives/radio_parser.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render_vtk/factory.h"


namespace dairlib::perceptive_locomotion {

using Eigen::VectorXd;
using Eigen::Vector3d;

using multibody::SquareSteppingStoneList;
using systems::SubvectorPassThrough;
using systems::RobotOutputSender;
using systems::RobotOutputReceiver;
using systems::RobotInputReceiver;

using drake::geometry::SceneGraph;
using drake::geometry::DrakeVisualizer;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::sensors::PixelType;
using drake::perception::pc_flags::BaseField::kXYZs;
using drake::perception::pc_flags::BaseField::kRGBs;
using drake::perception::DepthImageToPointCloud;

HikingSimDiagram::HikingSimDiagram(
    const std::variant<std::string, SquareSteppingStoneList>& terrain,
    const std::string& camera_pose_yaml)
    : urdf_("examples/Cassie/urdf/cassie_v2_self_collision.urdf") {


  // magic numbers:
  static constexpr double sim_dt = 5e-4;
  static constexpr double terrain_friction = 0.8;
  static constexpr double actuator_delay = 6e-3;
  static constexpr double actuator_update_period = 2e-3;

  // other constants
  auto contact_solver = drake::multibody::DiscreteContactSolver::kSap;
  const std::string renderer_name = "hiking_sim_renderer";

  DiagramBuilder<double> builder;

  // Add the plant
  scene_graph_ = builder.AddSystem<SceneGraph<double>>();
  plant_ = builder.AddSystem<MultibodyPlant<double>>(sim_dt);
  plant_->set_discrete_contact_solver(contact_solver);

  multibody::AddSteppingStonesToSim(
      plant_, scene_graph_, terrain, terrain_friction
  );
  AddCassieMultibody(plant_, scene_graph_, true, urdf_, true, true);
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
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(*plant_);

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
  camera->set_name("pelvis_depth");

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
  builder.Connect(*state_sender, *state_receiver);

  // export the output ports
  input_port_control_ = builder.ExportInput(
      discrete_time_delay->get_input_port(), "u, t"
  );
  input_port_radio_ = builder.ExportInput(
      radio_parser->get_input_port(), "radio channels"
  );
  output_port_state_ = builder.ExportOutput(
      state_receiver->get_output_port(), "x, u, t"
  );
  output_port_state_lcm_ = builder.ExportOutput(
      state_sender->get_output_port(), "lcmt_robot_output"
  );
  output_port_cassie_out_ = builder.ExportOutput(
      sensor_aggregator.get_output_port(), "lcmt_cassie_out"
  );
  output_port_lcm_radio_ = builder.ExportOutput(
      radio_parser->get_output_port(), "lcmt_radio_out"
  );
  output_port_scene_graph_query_  = builder.ExportOutput(
      scene_graph_->get_query_output_port(), "Scene_graph_query_port"
  );
  output_port_depth_image_ = builder.ExportOutput(
      camera->depth_image_32F_output_port(), "depth_image_32F"
  );

  builder.BuildInto(this);
  this->set_name("hiking_sim_diagram");
}

std::pair<VectorXd, VectorXd>  HikingSimDiagram::SetPlantInitialConditionFromIK(
    const drake::systems::Diagram<double>* parent_diagram,
    Context<double>* parent_context, const Vector3d& pelvis_vel,
    double foot_spread, double height) const {

  auto [q, v] = GetInitialCassieState(urdf_, true, pelvis_vel, foot_spread, height);
  SetPlantInitialCondition(parent_diagram, parent_context, q, v);

  return {q, v};
}

void HikingSimDiagram::SetPlantInitialCondition(
    const drake::systems::Diagram<double>* parent_diagram,
    Context<double>* parent_context, const VectorXd& q, const VectorXd& v)
    const {

  auto& plant_context = parent_diagram->GetMutableSubsystemContext(
      *plant_, parent_context
  );
  plant_->SetPositions(&plant_context, q);
  plant_->SetVelocities(&plant_context, v);
}

const DrakeVisualizer<double>& HikingSimDiagram::AddDrakeVisualizer(
    DiagramBuilder<double>* builder) const {
  auto& visualizer = *builder->AddSystem<DrakeVisualizer<double>>();
  builder->Connect(
      get_output_port_scene_graph_query(),
      visualizer.query_object_input_port()
  );
  return visualizer;
}

}
