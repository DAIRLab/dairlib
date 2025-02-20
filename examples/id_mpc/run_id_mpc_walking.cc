#include "cassie_mpc_utils.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/id_mpc/systems/id_mpc_walking_system.h"
#include "systems/controllers/id_mpc/systems/constant_reference_system.h"
#include "systems/robot_lcm_systems.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/solvers/osqp_solver.h"
#include "solvers/solver_options_io.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::trajectories::PiecewisePolynomial;

using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::ConstantVectorSource;
using drake::yaml::LoadYamlFile;

using solvers::SolverOptionsFromYaml;
const std::string gains_f = "examples/id_mpc/gains/mpc_gains_walking.yaml";
const std::string gait_f = "examples/id_mpc/gains/gait_params_walking.yaml";
const std::string solver_opts_in = "examples/id_mpc/gains/osqp_opts_ncqp.yaml";
const std::string solver_opts_p = "examples/id_mpc/gains/osqp_opts_tight.yaml";

int DoMain() {

  drake::systems::DiagramBuilder<double> builder;

  auto dynamics = MakeCassieDynamics();

  IDMPCParams params = LoadIDMPCParamsFromYaml(gains_f);

  auto plant_context = dynamics->get_plant().CreateDefaultContext();
  auto gait_params = MakeCassieGaitParams(gait_f, params);

  auto ref_gen = builder.AddSystem<WalkingReferenceSystem>(
      *dynamics, plant_context.get(), gait_params);

  const auto& plant = dynamics->get_plant();
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);

  auto mpc_system = builder.AddSystem<IDMPCWalkingSystem>(
      params, std::move(dynamics), gait_params,
      LoadYamlFile<SolverOptionsFromYaml>(
          solver_opts_in).GetAsSolverOptions(drake::solvers::OsqpSolver::id()),
      LoadYamlFile<SolverOptionsFromYaml>(
          solver_opts_p).GetAsSolverOptions(drake::solvers::OsqpSolver::id()));

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  auto solution_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_timestamped_saved_traj>(
          "ID_MPC", &lcm_local, TriggerTypeSet({TriggerType::kForced})
      ));
  auto debug_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_id_mpc_walking_debug>(
          "ID_MPC_DEBUG", &lcm_local, TriggerTypeSet({TriggerType::kForced})
      ));

  auto vdes = builder.AddSystem<ConstantVectorSource<double>>(
      1.2 * Eigen::Vector2d::UnitX());

  builder.Connect(
      state_receiver->get_output_port(),
      mpc_system->get_input_port_state()
  );
  builder.Connect(
      ref_gen->get_output_port(),
      mpc_system->get_input_port_reference()
  );
  builder.Connect(
      state_receiver->get_output_port(),
      ref_gen->get_input_port_state()
  );
  builder.Connect(
      vdes->get_output_port(),
      ref_gen->get_input_port_vdes()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_solution(),
      solution_pub->get_input_port()
  );
  builder.Connect(
      mpc_system->get_output_port_mpc_debug(),
      debug_pub->get_input_port()
  );

  auto diagram = builder.Build();

  LcmDrivenLoop<lcmt_robot_output> loop(
      &lcm_local, std::move(diagram), state_receiver, "CASSIE_STATE_SIMULATION",
      true, 50);

  loop.Simulate();

  return 0;

}

}

int main(int argc, char** argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}