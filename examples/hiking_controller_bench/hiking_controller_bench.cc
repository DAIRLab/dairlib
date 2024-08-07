#include "examples/hiking_controller_bench/hiking_controller_bench.h"

#include "geometry/convex_polygon_set.h"
#include "examples/perceptive_locomotion/diagrams/mpfc_osc_diagram.h"
#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/cassie_mpfc_diagram.h"
#include "examples/perceptive_locomotion/systems/cassie_radio_operator.h"
#include "systems/controllers/footstep_planning/alip_mpfc_s2s_system.h"
#include "systems/controllers/footstep_planning/cf_mpfc_system.h"

#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/plant_visualizer.h"

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/analysis/simulator.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

#include "drake/common/yaml/yaml_io.h"

namespace dairlib::perceptive_locomotion {

using drake::systems::ConstantVectorSource;
using drake::systems::ConstantValueSource;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::TriggerType;

using geometry::ConvexPolygonSet;
using systems::controllers::Alips2sMPFCSystem;

template <mpfc MPC, mpfc_osc_diagram OSC>
HikingControllerBench<MPC, OSC>::HikingControllerBench(
    const std::string &terrain_yaml,
    const std::string &osc_gains_yaml,
    const std::string &mpc_gains_yaml,
    const std::string &qp_solver_options_yaml,
    const std::string &camera_yaml,
    bool visualize) {

  const std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  [[maybe_unused]] auto instance = AddCassieMultibody(
      &plant_, nullptr, true, urdf, true, false);
  plant_.Finalize();

  auto builder = drake::systems::DiagramBuilder<double>();

  auto mpfc = builder.AddSystem<CassieMPFCDiagram<MPC>>(
      plant_, mpc_gains_yaml, -1);

  std::vector<ConvexPolygon> footholds =
      multibody::LoadSteppingStonesFromYaml(terrain_yaml).footholds;

  auto foothold_source = builder.AddSystem<ConstantValueSource<double>>(
      drake::Value<ConvexPolygonSet>(footholds));

  auto osc_diagram = builder.AddSystem<OSC>(
      plant_, osc_gains_yaml, mpc_gains_yaml, qp_solver_options_yaml);

  sim_diagram_ = builder.AddSystem<HikingSimDiagram>(
      terrain_yaml, camera_yaml);

  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION",
          &lcm_log_sink_,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto osc_debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_osc_output>(
          "OSC_DEBUG_WALKING",
          &lcm_log_sink_,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto input_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_robot_input>(
          "OSC_WALKING",
          &lcm_log_sink_,
          {TriggerType::kPeriodic},
          0.001)
  );
  auto mpc_pub = builder.AddSystem(
      LcmPublisherSystem::Make<decltype(MPC::empty_debug_message())>(
          "S2S_MPFC_DEBUG",
          &lcm_log_sink_,
          {TriggerType::kPeriodic},
          0.01)
  );

  auto radio_source = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(18));

  vdes_source_ = builder.AddSystem<ConstantVectorSource<double>>(
      Eigen::Vector2d::Zero());

  builder.Connect(
      vdes_source_->get_output_port(),
      mpfc->get_input_port_vdes()
  );
  builder.Connect(
      sim_diagram_->get_output_port_state_lcm(),
      osc_diagram->get_input_port_state()
  );
  builder.Connect(
      sim_diagram_->get_output_port_lcm_radio(),
      osc_diagram->get_input_port_radio()
  );
  builder.Connect(
      sim_diagram_->get_output_port_state_lcm(),
      mpfc->get_input_port_state()
  );
  builder.Connect(
      mpfc->get_output_port_mpc_output(),
      osc_diagram->get_input_port_mpc_output()
  );
  builder.Connect(
      osc_diagram->get_output_port_actuation(),
      sim_diagram_->get_input_port_actuation()
  );
  builder.Connect(
      foothold_source->get_output_port(),
      mpfc->get_input_port_footholds()
  );
  builder.Connect(
      radio_source->get_output_port(),
      sim_diagram_->get_input_port_radio()
  );

  builder.Connect(osc_diagram->get_output_port_osc_debug(),
                  osc_debug_pub->get_input_port());
  builder.Connect(osc_diagram->get_output_port_u_lcm(),
                  input_pub->get_input_port());
  builder.Connect(sim_diagram_->get_output_port_state_lcm(),
                  state_pub->get_input_port());
  builder.Connect(mpfc->get_output_port_mpfc_debug(),
                  mpc_pub->get_input_port());

  if (visualize) {
    auto plant_visualizer = builder.AddSystem<systems::PlantVisualizer>(urdf);
    multibody::AddSteppingStonesToMeshcatFromYaml(
        plant_visualizer->get_meshcat(), terrain_yaml);
    builder.Connect(
        sim_diagram_->get_output_port_state(),
        plant_visualizer->get_input_port());
  }

  diagram_ = builder.Build();
}

template <mpfc MPC, mpfc_osc_diagram OSC>
void HikingControllerBench<MPC, OSC>::Simulate(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    double realtime_rate, double end_time, std::string save_file) {

  lcm_log_sink_.clear();
  auto context = diagram_->CreateDefaultContext();
  sim_diagram_->SetPlantInitialCondition(
      diagram_.get(),
      context.get(),
      q, v
  );

  drake::systems::Simulator<double> simulator(*diagram_, std::move(context));
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);

  if (realtime_rate > 0) {
    simulator.set_target_realtime_rate(realtime_rate);
  }
  simulator.AdvanceTo(end_time);

  if (not save_file.empty()) {
    lcm_log_sink_.WriteLog(save_file);
  }
}

template class HikingControllerBench<Alips2sMPFCSystem, MpfcOscDiagram>;

}