#include "geometry/convex_polygon_set.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/alip_mpfc_diagram.h"

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


namespace dairlib {
namespace perceptive_locomotion {

using geometry::ConvexPolygonSet;

using drake::systems::ConstantVectorSource;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;

int DoMain() {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  auto instance =
      AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();


  std::string gains_file =
      "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string gains_mpc_file =
      "examples/perceptive_locomotion/gains/alip_s2s_mpfc_gains.yaml";
  std::string osqp_options =
      "solvers/fcc_qp_options_default.yaml";
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";
  std::string terrain_yaml =
      "examples/perceptive_locomotion/terrains/stones.yaml";
  std::string elevation_mapping_params_yaml =
      "examples/perceptive_locomotion/camera_calib/"
      "elevation_mapping_params_simulation.yaml";


  auto builder = drake::systems::DiagramBuilder<double>();

  auto mpfc = builder.AddSystem<AlipMPFCDiagram>(plant, gains_mpc_file, -1);

  std::vector<ConvexPolygon> footholds =
      multibody::LoadSteppingStonesFromYaml(terrain_yaml).footholds;

  auto foothold_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<ConvexPolygonSet>(footholds));

  auto vdes = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::Vector2d::Zero());

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
  auto radio_source = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(18)
  );
  auto visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);

  builder.Connect(
      vdes->get_output_port(), mpfc->get_input_port_vdes()
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
      osc_diagram->get_input_port_alip_mpc_output()
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
      radio_source->get_output_port(),
      sim_diagram->get_input_port_radio()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      visualizer->get_input_port()
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
  simulator.set_target_realtime_rate(1.0);
//  simulator.Initialize();
  simulator.AdvanceTo(10.0);

  lcm_log_sink.WriteLog("../lcm_log_sink_test_log");
  lcm_log_sink.clear();

  return 0;
}
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain();
}