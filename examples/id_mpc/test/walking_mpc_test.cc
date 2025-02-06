#include "examples/id_mpc/cassie_mpc_utils.h"

#include "systems/framework/lcm_driven_loop.h"
#include "systems/controllers/id_mpc/systems/id_mpc_system.h"
#include "systems/controllers/id_mpc/systems/walking_reference_system.h"
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
  IDMPCParams params =
      LoadIDMPCParamsFromYaml("examples/id_mpc/gains/mpc_gains_walking.yaml");
  VectorXd q = VectorXd::Zero(dynamics->nq());
  VectorXd v = VectorXd::Zero(dynamics->nv());
  VectorXd u = VectorXd::Zero(dynamics->nu());

  q << 1, 0, 0, 0, 0, 0, 0.75,
      0.0924283, 0, 0.839764, -1.91927, 2.14352, -1.9375,
      -0.0924283, 0, 0.839764, -1.91927, 2.14352, -1.9375;

  u <<  -2.03951, 2.04169, 0.906345, -0.861539, -5.96077, -6.16527, 45.7984,
      45.6304, -3.48936, -3.52897;


  OutputVector<double> state(q, v, u);

  auto plant_context = dynamics->get_plant().CreateDefaultContext();

  auto ref_gen = builder.AddSystem<WalkingReferenceSystem>(
      *dynamics, plant_context.get(), MakeCassieGaitParams(params));

  auto mpc_system = builder.AddSystem<IDMPCSystem>(params, std::move(dynamics));

  ref_gen->AddSwingFootTrajCostToMPC(
      mpc_system->get_mutable_trajopt_ptr(), Eigen::Matrix3d::Identity());

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");
  auto solution_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_timestamped_saved_traj>(
          "ID_MPC", &lcm_local, TriggerTypeSet({TriggerType::kForced})
      ));
  builder.Connect(
      ref_gen->get_output_port(),
      mpc_system->get_input_port_reference()
  );
  builder.Connect(*mpc_system, *solution_pub);
  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  const double dt = 0.025;
  double t = 0;

  while (t < 5.0) {
    state.set_timestamp(t);
    context->SetTime(t);

    auto& ref_subcontext = ref_gen->GetMyMutableContextFromRoot(context.get());
    ref_gen->get_input_port_state().FixValue(&ref_subcontext, state);
    ref_gen->get_input_port_vdes().FixValue(&ref_subcontext, 0.8 * Eigen::Vector2d::UnitX());

    auto& mpc_subcontext =
        mpc_system->GetMyMutableContextFromRoot(context.get());
    mpc_system->get_input_port_state().FixValue(&mpc_subcontext, state);


    diagram->CalcForcedUnrestrictedUpdate(
        *context, &context->get_mutable_state());
    diagram->ForcedPublish(*context);
    t += dt;
  }

  return 0;

}

}

int main(int argc, char** argv) {
  return dairlib::systems::controllers::id_mpc::DoMain();
}