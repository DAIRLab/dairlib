#include "full_sim_diagram.h"

#include "geometry/convex_polygon_set.h"
#include "examples/perceptive_locomotion/systems/cassie_radio_operator.h"
#include "examples/perceptive_locomotion/systems/alip_mpfc_meshcat_visualizer.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/plant_visualizer.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::perceptive_locomotion {

using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::TriggerType;
using drake::systems::ConstantValueSource;
using drake::systems::ConstantVectorSource;
using drake::systems::lcm::LcmPublisherSystem;

using geometry::ConvexPolygonSet;
using systems::CassieRadioOperator;
using systems::controllers::Alips2sMPFCSystem;

FullSimDiagram::FullSimDiagram(const std::string& mpc_gains_yaml,
                               const std::string& solver_options_yaml,
                               const std::string &terrain_yaml,
                               const std::string &sim_params_yaml,
                               bool visualize) {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2_self_collision.urdf";
  [[maybe_unused]] auto instance = AddCassieMultibody(
      &plant, nullptr, true, urdf, false, false);
  plant.Finalize();

  plant_context = plant.CreateDefaultContext();

  std::string osc_gains_file =
      "examples/perceptive_locomotion/gains/osc_gains_simulation.yaml";
  std::string osqp_options =
      "solvers/fcc_qp_options_default.yaml";
  std::string camera_yaml =
      "examples/perceptive_locomotion/camera_calib/cassie_hardware.yaml";

  const auto sim_options =
      drake::yaml::LoadYamlFile<std::map<std::string, std::vector<double>>>(
          sim_params_yaml);

  Eigen::Vector2d goal_location = Eigen::Vector2d::Map(
      sim_options.at("goal_location").data());

  auto builder = drake::systems::DiagramBuilder<double>();

  auto mpfc = builder.AddSystem<CassieMPFCDiagram<Alips2sMPFCSystem>>(
      plant, mpc_gains_yaml, solver_options_yaml, -1
  );

  std::vector<ConvexPolygon> footholds =
      multibody::LoadSteppingStonesFromYaml(terrain_yaml).footholds;

  auto foothold_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<ConvexPolygonSet>(footholds));

  auto radio_operator = builder.AddSystem<CassieRadioOperator>(
      plant, plant_context.get());

  auto osc_diagram = builder.AddSystem<MpfcOscDiagram>(
      plant, osc_gains_file, mpc_gains_yaml, osqp_options
  );
  sim_diagram = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml
  );

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
      LcmPublisherSystem::Make<lcmt_alip_mpfc_debug_complete>(
          "ALIP_S2S_MPFC_DEBUG",
          &lcm_log_sink,
          {TriggerType::kPeriodic},
          0.01)
  );

  auto goal_position = builder.AddSystem<ConstantVectorSource<double>>(
      goal_location
  );

  builder.Connect(
      foothold_source->get_output_port(),
      mpfc->get_input_port_footholds()
  );
  builder.Connect(
      sim_diagram->get_output_port_state_lcm(),
      osc_diagram->get_input_port_state()
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
      radio_operator->get_output_port_radio(),
      sim_diagram->get_input_port_radio()
  );

  if (visualize) {
    auto plant_visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
    auto mpfc_visualizer = builder.AddSystem<AlipMPFCMeshcatVisualizer>(
        plant_visualizer->get_meshcat(), plant_visualizer->get_plant());
    multibody::AddSteppingStonesToMeshcatFromYaml(
        plant_visualizer->get_meshcat(), terrain_yaml
    );

    meshcat_ = plant_visualizer->get_meshcat();

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
  }



  builder.Connect(osc_diagram->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(osc_diagram->get_output_port_u_lcm(),
                  input_pub->get_input_port());
  builder.Connect(sim_diagram->get_output_port_state_lcm(),
                  state_pub->get_input_port());
  builder.Connect(mpfc->get_output_port_mpfc_debug(),
                  mpc_pub->get_input_port());

  builder.BuildInto(this);
}

void FullSimDiagram::SetPlantInitialConditions(
    Diagram<double> *diagram, Context<double> *context) {
  auto [q, v] = sim_diagram->SetPlantInitialConditionFromIK(
      diagram, context, Eigen::Vector3d::Zero(), 0.1, 0.95
  );
}

drake::math::RigidTransformd FullSimDiagram::GetCassiePelvisPoseInWorld(
    const Context<double>& context) const {
  const auto& sim_plant = sim_diagram->get_plant();
  const auto& plant_context = sim_plant.GetMyContextFromRoot(context);
  return sim_plant.GetBodyByName("pelvis").EvalPoseInWorld(plant_context);
}

void FullSimDiagram::SaveLcmLog(const std::string &fname) {
  lcm_log_sink.WriteLog(fname);
  lcm_log_sink.clear();
}

}
