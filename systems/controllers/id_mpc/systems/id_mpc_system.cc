#include "systems/controllers/id_mpc/systems/id_mpc_system.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/id_mpc/costs/mpc_reference.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::State;

using Eigen::VectorXd;
using solvers::QPData;

IDMPCSystem::IDMPCSystem(
    IDMPCParams params,
    std::unique_ptr<ConstrainedDynamicsInfo> dynamics) :
    trajopt_(params, std::move(dynamics)),
    solver_(trajopt_.num_vars(), trajopt_.num_constraints(),
      [this](const VectorXd& x, QPData& qp) {
        this->trajopt_.ConstructSQPProgram(x, qp);
      },
      [this](const VectorXd& x) {
        return this->trajopt_.EvaluateConstraintViolation(x);
      },
      [this](const VectorXd& x) {
        return this->trajopt_.EvaluateCost(x);
      }) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(trajopt_.dynamics().get_plant())
  ).get_index();

  input_port_reference_ = DeclareAbstractInputPort(
      "mpc_reference",
      drake::Value<MPCReference>()).get_index();

  MPCSolution model_solution;
  model_solution.sqp_iterate = solver_.AllocateIterate();
  mpc_solution_state_ = DeclareAbstractState(
      drake::Value<MPCSolution>(model_solution));

  DeclareForcedUnrestrictedUpdateEvent(&IDMPCSystem::SolveMPC);

  output_port_mpc_solution_ = DeclareAbstractOutputPort(
      "mpc_solution", lcmt_id_mpc_solution(),
      &IDMPCSystem::CalcOutput).get_index();
}

EventStatus IDMPCSystem::SolveMPC(
    const Context<double> &context, State<double> *system_state) const {
  const auto& reference = get_input_port_reference().Eval<MPCReference>(context);
  const auto& state = EvalVectorInput<OutputVector>(context, input_port_state_);

  auto solution =
      system_state->get_mutable_abstract_state<MPCSolution>(mpc_solution_state_);

  if (solution.is_initial_solve) {
    SetInitialSolverState(*state, solution.sqp_iterate);
    solution.is_initial_solve = false;
  }
  const Eigen::VectorXd& x = state->GetState();

  trajopt_.UpdateProblemData(reference, x);
  solver_.DoSQPStep(solution.sqp_iterate.x_sol, solution.sqp_iterate);
  solution.contact_sequence = reference.active_contacts_;
  return EventStatus::Succeeded();
}

void IDMPCSystem::CalcOutput(const Context<double>& context,
                             lcmt_id_mpc_solution *solution) const {
  auto mpc_solution =
      context.get_abstract_state<MPCSolution>(mpc_solution_state_);
  solution->traj = mpc_solution.solution_trajectories.GenerateLcmObject();
}

void IDMPCSystem::SetInitialSolverState(const OutputVector<double> &x_u_t,
                                        SQPIterate &solver_state) const {
  throw std::runtime_error("not implemented!");
}

}