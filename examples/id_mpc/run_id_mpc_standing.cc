#include "cassie_mpc_utils.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/id_mpc/systems/id_mpc_system.h"
#include "systems/controllers/id_mpc/systems/constant_reference_system.h"
#include "systems/robot_lcm_systems.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"


namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::trajectories::PiecewisePolynomial;

using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmPublisherSystem;


int DoMain() {

  drake::systems::DiagramBuilder<double> builder;

  auto dynamics = MakeCassieDynamics();

  // TODO (@Brian-Acosta) YAML-ize this
  IDMPCParams params;
  params.dt = 0.05;
  params.N = static_cast<int>(0.5 / params.dt);
  params.num_full_torque_knots = 2;

  params.Wq = 100 * MatrixXd::Identity(dynamics->nq(), dynamics->nq());
  params.Wv = 0.01 * MatrixXd::Identity(dynamics->nv(), dynamics->nv());
  params.Wu = 0.01 * MatrixXd::Identity(dynamics->nu(), dynamics->nu());
  params.Wlambda = 0.01 * MatrixXd::Identity(
      dynamics->nlambda(), dynamics->nlambda());


  VectorXd q = VectorXd::Zero(dynamics->nq());
  VectorXd v = VectorXd::Zero(dynamics->nv());
  VectorXd u = VectorXd::Zero(dynamics->nu());
  VectorXd lambda = VectorXd::Zero(dynamics->nlambda());

  q << 1, 0, 0, 0, 0, 0, 0.95,
      0.0730404, 0, 0.571375, -1.38058, 1.60491, -1.6692,
      -0.0730404, 0, 0.571375, -1.38058, 1.60491, -1.6692;

  u <<  -2.03951, 2.04169, 0.906345, -0.861539, -5.96077, -6.16527, 45.7984,
      45.6304, -3.48936, -3.52897;

  lambda << -395.296, -395.589,
      39.7438, -9.54163, 81.7462,
      -39.8489, 4.21296, 80.2822,
      39.8174, 9.30058, 81.8166,
      -39.7123, -3.97191, 79.936;

  MPCReference reference;
  reference.q_traj_ = PiecewisePolynomial<double>(q);
  reference.v_traj_ = PiecewisePolynomial<double>(v);
  reference.lambda_traj_ = PiecewisePolynomial<double>(lambda);
  reference.u_traj_ = PiecewisePolynomial<double>(u);
  for (int i = 0; i < params.N + 1; ++i) {
    reference.active_contacts_.push_back(dynamics->contacts());
    reference.knot_times_.push_back(params.dt * i);
  }

  const auto& plant = dynamics->get_plant();
  auto const_ref = builder.AddSystem<ConstantReferenceSystem>(reference);
  auto mpc_system = builder.AddSystem<IDMPCSystem>(params, std::move(dynamics));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(plant);

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  auto solution_pub = builder.AddSystem(
    LcmPublisherSystem::Make<lcmt_id_mpc_solution>(
        "ID_MPC", &lcm_local, TriggerTypeSet({TriggerType::kForced})
    ));


  builder.Connect(
      state_receiver->get_output_port(),
      mpc_system->get_input_port_state()
  );
  builder.Connect(
      const_ref->get_output_port(),
      mpc_system->get_input_port_reference()
  );
  builder.Connect(*mpc_system, *solution_pub);
  auto diagram = builder.Build();



  LcmDrivenLoop<lcmt_robot_output> loop(
      &lcm_local, std::move(diagram), state_receiver, "CASSIE_STATE_SIMULATION", true);

  loop.Simulate();

  return 0;

}

}

int main(int argc, char** argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}