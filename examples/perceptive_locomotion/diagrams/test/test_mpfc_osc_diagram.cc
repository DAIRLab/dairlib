#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/perception_module_diagram.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/plant_visualizer.h"
#include "systems/perception/grid_map_visualizer.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace dairlib {
namespace perceptive_locomotion {

using drake::systems::ConstantVectorSource;

int DoMain() {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  drake::multibody::MultibodyPlant<double> plant(0.0);
  [[maybe_unused]] auto instance =
      AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();

  std::string gains_file =
      "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string gains_mpc_file =
      "examples/perceptive_locomotion/gains/alip_mpfc_gains_simulation.yaml";
  std::string osqp_options =
      "examples/perceptive_locomotion/gains/osqp_options_osc.yaml";
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";
  std::string terrain_yaml =
      "examples/perceptive_locomotion/terrains/stones.yaml";
  std::string elevation_mapping_params_yaml =
      "examples/perceptive_locomotion/camera_calib/"
      "elevation_mapping_params_simulation.yaml";

  auto builder = drake::systems::DiagramBuilder<double>();

  auto osc_diagram = builder.AddSystem<MpfcOscDiagram>(
      plant, gains_file, gains_mpc_file, osqp_options
  );
  auto sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml
  );
  std::map<std::string, drake::systems::sensors::CameraInfo> sensor_info;
  for (const auto& sensor_name : {"pelvis_depth"}) {
    sensor_info.insert(
        {sensor_name, sim_diagram->get_depth_camera_info(sensor_name)}
    );
  }
  auto perception_module = builder.AddSystem(
      PerceptionModuleDiagram::Make(elevation_mapping_params_yaml, sensor_info)
  );
  auto foothold_source = builder.AddSystem<ConstantVectorSource<double>>(
      -0.3 * Eigen::Vector3d::UnitY()
  );
  auto radio_source = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(18)
  );
  auto visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
  std::vector<std::string> layers_to_visualize = {"elevation"};
  auto grid_map_visualizer = builder.AddSystem<perception::GridMapVisualizer>(
      visualizer->get_meshcat(), 30.0, layers_to_visualize
  );

  builder.Connect(
      sim_diagram->get_output_port_cassie_out(),
      perception_module->get_input_port_cassie_out()
  );
  builder.Connect(
      perception_module->get_output_port_robot_output(),
      osc_diagram->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_depth_image(),
      perception_module->get_input_port_depth_image("pelvis_depth")
  );
  builder.Connect(
      sim_diagram->get_output_port_lcm_radio(),
      osc_diagram->get_input_port_radio()
  );
  builder.Connect(
      osc_diagram->get_output_port_actuation(),
      sim_diagram->get_input_port_actuation()
  );
  builder.Connect(
      foothold_source->get_output_port(),
      osc_diagram->get_input_port_footstep_command()
  );
  builder.Connect(
      radio_source->get_output_port(),
      sim_diagram->get_input_port_radio()
  );
  builder.Connect(
      perception_module->get_output_port_state(), visualizer->get_input_port()
  );
  builder.Connect(
      perception_module->get_output_port_elevation_map(),
      grid_map_visualizer->get_input_port()
  );

  auto diagram = builder.Build();
  diagram->set_name("mpfc_osc_with_sim");
  DrawAndSaveDiagramGraph(*diagram);

  auto context = diagram->CreateDefaultContext();

  auto [q, v] = sim_diagram->SetPlantInitialConditionFromIK(
      diagram.get(),
      context.get() ,
      Vector3d::Zero(),
      0.3,
      0.95
  );

  perception_module->InitializeEkf(context.get(), q, v);

  drake::systems::Simulator<double> simulator(*diagram, std::move(context));

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(0.05);
  simulator.Initialize();
  simulator.AdvanceTo(0.5);
  return 0;
}
}
}

int main(int argc, char **argv) {
  return dairlib::perceptive_locomotion::DoMain();
}