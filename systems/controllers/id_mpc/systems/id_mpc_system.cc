#include "id_mpc_system.h"

#include "multibody/multibody_utils.h"
#include "systems/controllers/id_mpc/costs/mpc_reference.h"
#include "systems/framework/output_vector.h"

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
      },
      [this](VectorXd& x){
        this->trajopt_.ProjectToQuaternionConstraint(x);
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
      "mpc_solution", lcmt_timestamped_saved_traj(),
      &IDMPCSystem::CalcOutput).get_index();

  plant_context_ = trajopt_.dynamics().get_plant().CreateDefaultContext();

}

EventStatus IDMPCSystem::SolveMPC(
    const Context<double> &context, State<double> *system_state) const {
  const auto& reference = get_input_port_reference().Eval<MPCReference>(context);
  const auto& state = EvalVectorInput<OutputVector>(context, input_port_state_);

  auto& solution =
      system_state->get_mutable_abstract_state<MPCSolution>(mpc_solution_state_);

  if (solution.is_initial_solve) {
    SetInitialSolverState(
        *state,
        reference.active_contacts_,
        solution.sqp_iterate);
    solution.is_initial_solve = false;
  }
  const Eigen::VectorXd& x = state->GetState();

  trajopt_.UpdateProblemData(reference, x);
  solver_.DoSQPStep(solution.sqp_iterate.x_sol, solution.sqp_iterate);
  solution.contact_sequence = reference.active_contacts_;

  drake::solvers::MathematicalProgramResult result;
  result.set_decision_variable_index(trajopt_.get_prog().decision_variable_index());
  result.set_x_val(solution.sqp_iterate.x_sol);
  solution.solution_trajectories = trajopt_.GetSolutionAsLcmTrajectory(result);

  return EventStatus::Succeeded();
}

void IDMPCSystem::CalcOutput(const Context<double>& context,
                             lcmt_timestamped_saved_traj *solution) const {
  auto mpc_solution =
      context.get_abstract_state<MPCSolution>(mpc_solution_state_);
  solution->saved_traj = mpc_solution.solution_trajectories.GenerateLcmObject();
  solution->utime = 1e6 * context.get_time();
}

void IDMPCSystem::SetInitialSolverState(
    const OutputVector<double> &x_u_t,
    const std::vector<std::vector<std::string>>& contacts,
    SQPIterate &solver_state) const {

  trajopt_.dynamics().SetPlantStateIfNew(
      x_u_t.GetState(), plant_context_.get());

  for (size_t i = 0; i < contacts.size() ; ++i) {
    VectorXd lambda = trajopt_.dynamics().EstimateConstraintForcesForFixedPoint(
        *plant_context_, x_u_t.GetEfforts(), contacts.at(i)
    );
    trajopt_.get_prog().SetInitialGuess(
        trajopt_.position_vars(i), x_u_t.GetPositions());
    trajopt_.get_prog().SetInitialGuess(
        trajopt_.velocity_vars(i), x_u_t.GetVelocities());
    if (trajopt_.has_lambdas_at_knot(i)) {
      trajopt_.get_prog().SetInitialGuess(
          trajopt_.lambda_vars(i), lambda);
    }
    if (trajopt_.has_torques_at_knot(i)) {
      trajopt_.get_prog().SetInitialGuess(
          trajopt_.input_vars(i), x_u_t.GetEfforts());
    }
  }
  solver_state.x_sol = trajopt_.get_prog().initial_guess();
}

}