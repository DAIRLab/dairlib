#include "perceptive_full_sim_diagram.h"

#include "geometry/convex_polygon_set.h"
#include "examples/perceptive_locomotion/systems/cassie_radio_operator.h"
#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualizer.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/perception/grid_map_visualizer.h"
#include "systems/perception/grid_map_lcm_systems.h"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/plant_visualizer.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::perceptive_locomotion {

using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::TriggerType;
using drake::systems::ConstantVectorSource;
using drake::systems::lcm::LcmPublisherSystem;

using geometry::ConvexPolygonSet;
using systems::CassieRadioOperator;
using perception::GridMapSender;
using perception::GridMapVisualizer;
using systems::controllers::Alips2sMPFCSystem;

PerceptiveFullSimDiagram::PerceptiveFullSimDiagram(const std::string& mpc_gains_yaml,
                                                   const std::string& solver_options_yaml,
                                                   const std::string &terrain_yaml,
                                                   const std::string &sim_params_yaml) {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2_self_collision.urdf";
  [[maybe_unused]] auto instance = AddCassieMultibody(
      &plant, nullptr, true, urdf, true, false);
  plant.Finalize();

  plant_context = plant.CreateDefaultContext();

  std::string gains_file =
      "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string osqp_options =
      "solvers/fcc_qp_options_default.yaml";
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";
  std::string elevation_mapping_params_yaml =
      "bindings/pydairlib/perceptive_locomotion/params/elevation_mapping_params_sim.yaml";

  const auto sim_options =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          sim_params_yaml);

  Eigen::Vector2d goal_location = Eigen::Vector2d::Map(
      sim_options.at("goal_location").data());

  auto builder = drake::systems::DiagramBuilder<double>();

  auto mpfc = builder.AddSystem<CassieMPFCDiagram<Alips2sMPFCSystem>>(plant,
      mpc_gains_yaml, solver_options_yaml, -1);

  auto radio_operator = builder.AddSystem<CassieRadioOperator>(
      plant, plant_context.get());

  auto osc_diagram = builder.AddSystem<MpfcOscDiagram>(
      plant, gains_file, mpc_gains_yaml, osqp_options
  );
  sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml
  );

  std::map<std::string, drake::systems::sensors::CameraInfo> sensor_info;
  for (const auto& sensor_name : {"pelvis_depth"}) {
    sensor_info.insert(
        {sensor_name, sim_diagram->get_depth_camera_info(sensor_name)}
    );
  }

  perception = builder.AddSystem(PerceptionModuleDiagram::Make(
      elevation_mapping_params_yaml, sensor_info));

  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto state_pub_dispatcher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          "NETWORK_CASSIE_STATE_DISPATCHER",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.005)
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
      LcmPublisherSystem::Make<lcmt_alip_mpfc_debug_complete>(
          "ALIP_S2S_MPFC_DEBUG",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.01)
  );
  auto grid_map_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_grid_map>(
          "CASSIE_ELEVATION_MAP",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          1.0 / 30.0)
  );

  auto grid_map_sender = builder.AddSystem<GridMapSender>();

  auto plant_visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
  auto mpfc_visualizer = builder.AddSystem<AlipMPFCMeshcatVisualizer>(
      plant_visualizer->get_meshcat(), plant_visualizer->get_plant());
  multibody::AddSteppingStonesToMeshcatFromYaml(
      plant_visualizer->get_meshcat(), terrain_yaml
  );

  meshcat_ = plant_visualizer->get_meshcat();

  // need to help template deduction out by declaring the type of this vector
  std::vector<std::string> layers = {"elevation", "segmented_elevation", "elevation_inpainted"};
  auto grid_map_visualizer = builder.AddSystem<GridMapVisualizer>(
      meshcat_, (1.0 / 30.0), layers
  );

  auto goal_position = builder.AddSystem<ConstantVectorSource<double>>(
      goal_location
  );

  builder.Connect(
      sim_diagram->get_output_port_cassie_out(),
      perception->get_input_port_cassie_out()
  );
  builder.Connect(
      perception->get_output_port_robot_output(),
      osc_diagram->get_input_port_state()
  );
  builder.Connect(
      perception->get_output_port_robot_output(),
      state_pub_dispatcher->get_input_port()
  );
  builder.Connect(
      sim_diagram->get_output_port_depth_image(),
      perception->get_input_port_depth_image("pelvis_depth")
  );
  builder.Connect(
      goal_position->get_output_port(),
      radio_operator->get_input_port_target_xy()
  );
  builder.Connect(
      perception->get_output_port_state(),
      radio_operator->get_input_port_state()
  );
  builder.Connect(
      radio_operator->get_output_port_horizontal_velocity(),
      mpfc->get_input_port_vdes()
  );
  builder.Connect(
      sim_diagram->get_output_port_lcm_radio(),
      osc_diagram->get_input_port_radio()
  );
  builder.Connect(
      perception->get_output_port_robot_output(),
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
      radio_operator->get_output_port_radio(),
      sim_diagram->get_input_port_radio()
  );
  builder.Connect(
      perception->get_output_port_state(),
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
  builder.Connect(*grid_map_sender, *grid_map_pub);

  input_port_footholds_ = builder.ExportInput(
      mpfc->get_input_port_footholds(),
      "footholds"
  );
  input_port_grid_map_ = builder.ExportInput(
      mpfc->get_input_port_grid_map(),
      "elevation"
  );
  output_port_elevation_map_ = builder.ExportOutput(
      perception->get_output_port_elevation_map(),
      "grid_map"
  );
  builder.ConnectInput(
      input_port_grid_map_,
      grid_map_sender->get_input_port()
  );
  builder.ConnectInput(
      input_port_grid_map_,
      grid_map_visualizer->get_input_port()
  );
  builder.BuildInto(this);
}

void PerceptiveFullSimDiagram::SetPlantInitialConditions(
    Diagram<double> *diagram, Context<double> *context) {
  auto [q, v] = sim_diagram->SetPlantInitialConditionFromIK(
      diagram, context, Eigen::Vector3d::Zero(), 0.1, 0.95
  );
  perception->InitializeEkf(context, q, v);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(q.rows() + v.rows());
  x.head(q.rows()) = q;
  v.tail(v.rows()) = v;
  perception->InitializeElevationMap(x, context);
}

drake::math::RigidTransformd PerceptiveFullSimDiagram::GetCassiePelvisPoseInWorld(
    const Context<double>& context) const {
  const auto& sim_plant = sim_diagram->get_plant();
  const auto& plant_context = sim_plant.GetMyContextFromRoot(context);
  return sim_plant.GetBodyByName("pelvis").EvalPoseInWorld(plant_context);
}

void PerceptiveFullSimDiagram::SaveLcmLog(const std::string &fname) {
  lcm_log_sink.WriteLog(fname);
  lcm_log_sink.clear();
}

}