#include <gflags/gflags.h>
#include "geometry/convex_polygon_set.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/cassie_mpfc_diagram.h"
#include "examples/perceptive_locomotion/systems/cassie_radio_operator.h"
#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualizer.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/plant_visualizer.h"
#include "lcm/lcm_log_sink.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_alip_s2s_mpfc_debug.hpp"

#include "drake/common/yaml/yaml_io.h"



namespace dairlib {
namespace perceptive_locomotion {

using geometry::ConvexPolygonSet;

using drake::systems::ConstantVectorSource;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;

using systems::CassieRadioOperator;
using systems::controllers::Alips2sMPFCSystem;

DEFINE_string(terrain, "examples/perceptive_locomotion/terrains/stones.yaml",
              "yaml file to load terrain from");

DEFINE_string(savefile, "../standalone_sim_log", "lcm log file to save");

int DoMain(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto instance =
      AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();

  auto plant_context = plant.CreateDefaultContext();

  std::string terrain_yaml = FindResourceOrThrow(FLAGS_terrain);
  std::string gains_file =
      "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string gains_mpc_file =
      "examples/perceptive_locomotion/gains/alip_s2s_mpfc_gains.yaml";
  std::string osqp_options =
      "solvers/fcc_qp_options_default.yaml";
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";
  std::string elevation_mapping_params_yaml =
      "examples/perceptive_locomotion/camera_calib/"
      "elevation_mapping_params_simulation.yaml";

  const auto sim_options =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          FindResourceOrThrow(
              "examples/perceptive_locomotion/standalone_sim_params.yaml"));

  Eigen::Vector2d goal_location = Eigen::Vector2d::Map(
      sim_options.at("goal_location").data());

  auto builder = drake::systems::DiagramBuilder<double>();

  auto mpfc = builder.AddSystem<CassieMPFCDiagram<Alips2sMPFCSystem>>(plant, gains_mpc_file, -1);

  std::vector<ConvexPolygon> footholds =
      multibody::LoadSteppingStonesFromYaml(terrain_yaml).footholds;

  auto foothold_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<ConvexPolygonSet>(footholds));

  auto radio_operator = builder.AddSystem<CassieRadioOperator>(
      plant, plant_context.get());

  auto osc_diagram = builder.AddSystem<MpfcOscDiagram>(
      plant, gains_file, gains_mpc_file, osqp_options,
      MpfcOscDiagramInputType::kLcmtAlipMpcOutput
  );
  auto sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml
  );

  lcm::LcmLogSink lcm_log_sink{};
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto osc_debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_osc_output>(
          "OSC_DEBUG_WALKING",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto input_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_input>(
          "OSC_WALKING",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto mpc_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_alip_s2s_mpfc_debug>(
          "ALIP_S2S_MPFC_DEBUG",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.01)
  );

  std::map<std::string, drake::systems::sensors::CameraInfo> sensor_info;
  for (const auto& sensor_name : {"pelvis_depth"}) {
    sensor_info.insert(
        {sensor_name, sim_diagram->get_depth_camera_info(sensor_name)}
    );
  }
  auto plant_visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
  auto mpfc_visualizer = builder.AddSystem<AlipMPFCMeshcatVisualizer>(
      plant_visualizer->get_meshcat(), plant_visualizer->get_plant());
  multibody::AddSteppingStonesToMeshcatFromYaml(
      plant_visualizer->get_meshcat(), terrain_yaml
  );
  auto goal_position = builder.AddSystem<ConstantVectorSource<double>>(
    goal_location
  );

  builder.Connect(
      goal_position->get_output_port(),
      radio_operator->get_input_port_target_xy()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      radio_operator->get_input_port_state()
  );
  builder.Connect(
      radio_operator->get_output_port_horizontal_velocity(),
      mpfc->get_input_port_vdes()
  );
  builder.Connect(
      sim_diagram->get_output_port_state_lcm(),
      osc_diagram->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_lcm_radio(),
      osc_diagram->get_input_port_radio()
  );
  builder.Connect(
      sim_diagram->get_output_port_state_lcm(),
      mpfc->get_input_port_state()
  );
  builder.Connect(
      mpfc->get_output_port_mpc_output(),
      osc_diagram->get_input_port_mpc_output()
  );
  builder.Connect(
      osc_diagram->get_output_port_actuation(),
      sim_diagram->get_input_port_actuation()
  );
  builder.Connect(
      foothold_source->get_output_port(),
      mpfc->get_input_port_footholds()
  );
  builder.Connect(
      radio_operator->get_output_port_radio(),
      sim_diagram->get_input_port_radio()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      plant_visualizer->get_input_port()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      mpfc_visualizer->get_input_port_state()
  );
  builder.Connect(
      mpfc->get_output_port_mpfc_debug(),
      mpfc_visualizer->get_input_port_mpc()
  );

  builder.Connect(osc_diagram->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(osc_diagram->get_output_port_u_lcm(),
                  input_pub->get_input_port());
  builder.Connect(sim_diagram->get_output_port_state_lcm(),
                  state_pub->get_input_port());
  builder.Connect(mpfc->get_output_port_mpfc_debug(),
                  mpc_pub->get_input_port());

  auto diagram = builder.Build();
  diagram->set_name("mpfc_osc_with_sim");
  DrawAndSaveDiagramGraph(*diagram);

  auto context = diagram->CreateDefaultContext();

  auto [q, v] = sim_diagram->SetPlantInitialConditionFromIK(
      diagram.get(),
      context.get() ,
      Vector3d::Zero(),
      0.1,
      0.95
  );
  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  if (sim_options.at("realtime_rate").front() > 0) {
    simulator.set_target_realtime_rate(sim_options.at("realtime_rate").front());
  }
  double end_time = sim_options.at("time").front();
  simulator.AdvanceTo(end_time);

  lcm_log_sink.WriteLog(FLAGS_savefile);
  lcm_log_sink.clear();

  return 0;
}
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain(argc, argv);
}