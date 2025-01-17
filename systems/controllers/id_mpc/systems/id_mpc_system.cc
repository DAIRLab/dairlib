#include "systems/controllers/id_mpc/systems/id_mpc_system.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/id_mpc/costs/mpc_reference.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::Context;

IDMPCSystem::IDMPCSystem(IDMPCParams params,
                         std::unique_ptr<ConstrainedDynamicsInfo> dynamics) :
                         trajopt_(params, std::move(dynamics)) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(trajopt_.dynamics().get_plant())
  ).get_index();

  input_port_reference_ = DeclareAbstractInputPort(
      "mpc_reference",
      drake::Value<MPCReference>()).get_index();

  mpc_solution_cache_ = DeclareCacheEntry(
      "mpc_solution", MPCSolution(),
      &IDMPCSystem::SolveMPC).cache_index();

  output_port_mpc_solution_ = DeclareAbstractOutputPort(
      "mpc_solution", lcmt_id_mpc_solution(),
      &IDMPCSystem::CalcOutput).get_index();
}

void IDMPCSystem::SolveMPC(
    const Context<double> &context, MPCSolution *solution) const {
  const auto& reference = get_input_port_reference().Eval<MPCReference>(context);
  const auto& state = EvalVectorInput<OutputVector>(context, input_port_state_);
  const Eigen::VectorXd& x = state->GetState();

  trajopt_.UpdateProblemData(reference, x);

  auto solver_options = drake::solvers::SolverOptions();
  solver_options.SetOption(drake::solvers::SnoptSolver::id(), "Print file",
                           "./snopt.out");
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Major Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Iterations Limit", 1e6);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance", 1e-2);
  solver_options.SetOption(drake::solvers::SnoptSolver::id(),
                           "Major feasibility tolerance", 1e-2);

  trajopt_.get_prog().SetSolverOptions(solver_options);

  const auto& result = solver_.Solve(trajopt_.get_prog());

  solution->contact_sequence = reference.active_contacts_;
  solution->solution_trajectories = trajopt_.GetSolutionAsLcmTrajectory(result);
  trajopt_.get_prog().SetInitialGuessForAllVariables(result.GetSolution());
}

void IDMPCSystem::CalcOutput(const Context<double>& context,
                             lcmt_id_mpc_solution *solution) const {
  auto mpc_solution =
      get_cache_entry(mpc_solution_cache_).Eval<MPCSolution>(context);

  solution->traj = mpc_solution.solution_trajectories.GenerateLcmObject();
}

}