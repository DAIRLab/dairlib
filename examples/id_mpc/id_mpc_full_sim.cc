#include "id_mpc_full_sim.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_id_mpc_reference.hpp"
#include "dairlib/lcmt_id_mpc_walking_debug.hpp"

#include "examples/Cassie/cassie_utils.h"
#include "examples/id_mpc/cassie_mpc_utils.h"
#include "examples/id_mpc/systems/id_mpc_walking_debug_visualizer.h"
#include "examples/perceptive_locomotion/systems/cassie_radio_operator.h"

#include "systems/controllers/id_mpc/systems/joint_pd_controller.h"
#include "systems/controllers/id_mpc/systems/joint_controller_gains.h"
#include "systems/perception/ground_truth_elevation_mapping_system.h"
#include "systems/visualization/lcm_visualization_systems.h"
#include "systems/plant_visualizer.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::TriggerType;
using drake::systems::ConstantValueSource;
using drake::systems::ConstantVectorSource;
using drake::systems::lcm::LcmPublisherSystem;

using perceptive_locomotion::HikingSimDiagram;
using perception::GroundTruthElevationMappingSystem;

IDMPCFullSim::IDMPCFullSim(const std::string &terrain,
                           const std::string &sim_opts,
                           const std::string &mpc_gains_yaml,
                           const std::string &pd_gains_yaml,
                           const std::string &gait_yaml,
                           const std::string &solver_options_yaml) {
  // setup the plant for the sim diagram
  const std::string urdf = "examples/Cassie/urdf/cassie_fixed_spring_conservative"
                           ".urdf";
  [[maybe_unused]] auto instance = AddCassieMultibody(
      &plant, nullptr, true, urdf, false, false);
  plant.Finalize();
  plant_context = plant.CreateDefaultContext();

  /* Load yamls */
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";

  multibody::SquareSteppingStoneList stepping_stones =
      multibody::LoadSteppingStonesFromYaml(terrain);

  std::vector<ConvexPolygon> footholds = stepping_stones.footholds;

  geometry::ConvexPolygonSet polygons_for_map = geometry::ConvexPolygonSet(
      stepping_stones.GetConvexPolygonsForHeightmapSimulation(stepping_stones.stones));


  const auto sim_options =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          sim_opts);

  auto builder = drake::systems::DiagramBuilder<double>();

  auto foothold_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<geometry::ConvexPolygonSet>(footholds));

  Eigen::Vector2d goal_location = Eigen::Vector2d::Map(
      sim_options.at("goal_location").data());

  auto goal_position = builder.AddSystem<ConstantVectorSource<double>>(
      goal_location
  );

  double dt = 0.01;

  // Add the MPC and PD Controller
  auto dynamics = MakeCassieDynamics();
  IDMPCParams params = LoadIDMPCParamsFromYaml(mpc_gains_yaml);

  context_for_reference_system = dynamics->get_plant().CreateDefaultContext();
  auto gait_params = MakeCassieGaitParams(gait_yaml, params);

  ref_gen = builder.AddSystem<WalkingReferenceSystem>(
      *dynamics, context_for_reference_system.get(), gait_params);
  ref_gen->MakeDrivenByStandaloneSimulator(dt);

  auto mpc_system = builder.AddSystem<IDMPCWalkingSystem>(
      params, std::move(dynamics), gait_params, solver_options_yaml);
  mpc_system->MakeDrivenByStandaloneSimulator(dt);

  auto gains = drake::yaml::LoadYamlFile<JointPDGains>(pd_gains_yaml);

  auto pd_controller = builder.AddSystem<JointPDController>(
      plant, gains.kp, gains.kd);

  auto radio_operator = builder.AddSystem<CassieRadioOperator>(
      plant, plant_context.get(), 0.4);

  sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain, camera_yaml, false
  );

  auto map_server = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<multibody::BoxSet>(stepping_stones.cubes)
  );

  auto plant_visualizer = builder.AddSystem<PlantVisualizer>(urdf);

  meshcat_ = plant_visualizer->get_meshcat();

  auto mpc_visualizer = builder.AddSystem<LcmConfigurationDrawer>(
      plant_visualizer->get_meshcat(), urdf, "q", 5);
  auto debug_visualizer = builder.AddSystem<IDMPCWalkingDebugVisualizer>(
      plant_visualizer->get_meshcat());

  multibody::AddSteppingStonesToMeshcatFromYaml(
      plant_visualizer->get_meshcat(), terrain
  );

  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.001)
  );

  auto solution_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_timestamped_saved_traj>(
          "ID_MPC",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          dt)
  );
  auto debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_id_mpc_walking_debug>(
          "ID_MPC_DEBUG",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          dt)
  );

  builder.Connect(
      sim_diagram->get_output_port_state_lcm(),+
      state_pub->get_input_port()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_solution(),
      solution_pub->get_input_port()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_debug(),
      debug_pub->get_input_port()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      radio_operator->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      mpc_system->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      ref_gen->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      pd_controller->get_input_port_state()
  );
  builder.Connect(
      sim_diagram->get_output_port_state(),
      plant_visualizer->get_input_port()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_solution(),
      mpc_visualizer->get_input_port()
  );
  builder.Connect(
      goal_position->get_output_port(),
      radio_operator->get_input_port_target_xy()
  );
  builder.Connect(
      radio_operator->get_output_port_horizontal_velocity(),
      ref_gen->get_input_port_vdes()
  );
  builder.Connect(
      ref_gen->get_output_port(),
      mpc_system->get_input_port_reference()
  );
  builder.Connect(
      foothold_source->get_output_port(),
      mpc_system->get_input_port_footholds()
  );
  builder.Connect(
      map_server->get_output_port(),
      mpc_system->get_input_port_boxes()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_solution(),
      pd_controller->get_input_port_lcm_traj()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_debug(),
      debug_visualizer->get_input_port()
  );
  builder.Connect(
    pd_controller->get_output_port(),
    sim_diagram->get_input_port_actuation()
  );

  builder.BuildInto(this);
}

void IDMPCFullSim::SetPlantInitialConditions(
    Diagram<double> *diagram, Context<double> *context) {
  auto [q, v] = sim_diagram->SetPlantInitialConditionFromIK(
      diagram, context, Eigen::Vector3d::Zero(), 0.1, 0.95
  );

  auto& ref_context = ref_gen->GetMyMutableContextFromRoot(context);
  ref_gen->CalcForcedUnrestrictedUpdate(ref_context, &ref_context.get_mutable_state());
}

drake::math::RigidTransformd IDMPCFullSim::GetCassiePelvisPoseInWorld(
    const Context<double>& context) const {
  const auto& sim_plant = sim_diagram->get_plant();
  const auto& plant_context = sim_plant.GetMyContextFromRoot(context);
  return sim_plant.GetBodyByName("pelvis").EvalPoseInWorld(plant_context);
}

void IDMPCFullSim::SaveLcmLog(const std::string &fname) {
  lcm_log_sink.WriteLog(fname);
  lcm_log_sink.clear();
}

}