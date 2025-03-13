#include "id_mpc_walking_system.h"
#include "common/eigen_utils.h"

#include "systems/controllers/id_mpc/core/solution_trajectories.h"
#include "systems/controllers/id_mpc/references/mpc_reference.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::multibody::MultibodyPlant;

using Eigen::Vector3d;
using Eigen::VectorXd;

using solvers::QPData;
using solvers::sqp::SQPIterate;
using solvers::sqp::AllocateSQPIterate;

using drake::solvers::SolverOptions;

IDMPCWalkingSystem::IDMPCWalkingSystem(
    IDMPCParams params,
    std::unique_ptr<ConstrainedDynamicsInfo> dynamics,
    GaitParams gait_params,
    const std::string& ncqp_solver_options_yaml) :
    trajopt_(params, std::move(dynamics), gait_params),
    solver_(
        [this](const VectorXd& x, QPData* qp) {
          this->trajopt_.mpc().ConstructSQPProgram(x, qp);
        },
        [this](const VectorXd& x) {
          return this->trajopt_.mpc().EvaluateConstraintViolation(x);
        },
        [this](const VectorXd& x) {
          return this->trajopt_.mpc().EvaluateCost(x);
        },
        [this](VectorXd* x){
          this->trajopt_.mpc().ProjectToQuaternionConstraint(x);
        },
        [this](const VectorXd& z) {
          return this->trajopt_.GetFootholdConstraints(z);
        },
        ncqp_solver_options_yaml) {
  
  input_port_state_ = DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(trajopt_.dynamics().get_plant())
  ).get_index();

  input_port_reference_ = DeclareAbstractInputPort(
      "mpc_reference",
      drake::Value<MPCReference>()).get_index();

  input_port_footholds_ = DeclareAbstractInputPort(
      "convex_polygon_footholds",
      drake::Value<geometry::ConvexPolygonSet>()).get_index();

  MPCSolution model_solution;
  model_solution.sqp_iterate = AllocateSQPIterate(trajopt_.mpc().num_vars());
  mpc_solution_state_ = DeclareAbstractState(
      drake::Value<MPCSolution>(model_solution));

  DeclareForcedUnrestrictedUpdateEvent(&IDMPCWalkingSystem::SolveMPC);

  output_port_mpc_solution_ = DeclareAbstractOutputPort(
      "mpc_solution", lcmt_timestamped_saved_traj(),
      &IDMPCWalkingSystem::CalcOutput).get_index();

  output_port_mpc_debug_ = DeclareAbstractOutputPort(
      "lcmt_id_mpc_walking_debug", lcmt_id_mpc_walking_debug(),
      &IDMPCWalkingSystem::CalcDebug).get_index();

  plant_context_ = trajopt_.dynamics().get_plant().CreateDefaultContext();
}

EventStatus IDMPCWalkingSystem::SolveMPC(
    const Context<double> &context, State<double> *system_state) const {
  const auto& reference = get_input_port_reference().Eval<MPCReference>(context);
  const auto& state = EvalVectorInput<OutputVector>(context, input_port_state_);

  auto& solution =
      system_state->get_mutable_abstract_state<MPCSolution>(mpc_solution_state_);

  if (solution.is_initial_solve) {
    auto footsteps = CalcInitialFootsteps(state->GetPositions(), reference);
    trajopt_.SetFootstepInitialGuess(footsteps);
    SetInitialSolverStateToCurrent(*state, reference.active_contacts_, solution.sqp_iterate);
    solution.is_initial_solve = false;
  } else {
    ShiftSolution(reference.knot_times_, &solution);
  }

  if (not reference.touchdown_ee_names_to_update_.empty()) {
    const auto pp = CalcFootstepLocations(reference, solution.sqp_iterate.x_sol);
    trajopt_.UpdateFootstepLocationsInStackedVariables(pp, &solution.sqp_iterate.x_sol);
  }

  const Eigen::VectorXd& x = state->GetState();

  const auto& footholds =
      get_input_port_footholds().Eval<geometry::ConvexPolygonSet>(context);

  trajopt_.UpdateProblemData(reference, x, solution.sqp_iterate.x_sol, footholds);
  solver_.DoSQPStep(solution.sqp_iterate.x_sol, &solution.sqp_iterate);
  solution.contact_sequence = reference.active_contacts_;

  drake::solvers::MathematicalProgramResult result;
  result.set_decision_variable_index(
      trajopt_.mutable_mpc().get_prog().decision_variable_index());
  result.set_x_val(solution.sqp_iterate.x_sol);
  solution.solution_trajectories = trajopt_.mpc().GetSolutionAsLcmTrajectory(result);

  return EventStatus::Succeeded();
}

void IDMPCWalkingSystem::MakeDrivenByStandaloneSimulator(double update_period) {
  DeclareInitializationUnrestrictedUpdateEvent(
      &IDMPCWalkingSystem::SolveMPC);
  DeclarePeriodicUnrestrictedUpdateEvent(
      update_period, 0, &IDMPCWalkingSystem::SolveMPC);
}

void IDMPCWalkingSystem::CalcOutput(const Context<double>& context,
                             lcmt_timestamped_saved_traj *solution) const {
  const auto& mpc_solution =
      context.get_abstract_state<MPCSolution>(mpc_solution_state_);
  solution->saved_traj = mpc_solution.solution_trajectories.GenerateLcmObject();
  solution->utime = 1e6 * context.get_time();
}

void IDMPCWalkingSystem::CalcDebug(const Context<double> &context,
                                   lcmt_id_mpc_walking_debug *debug) const {
  CalcOutput(context, &debug->solution);

  const auto& mpc_solution =
      context.get_abstract_state<MPCSolution>(mpc_solution_state_);

  debug->reference = ConvertToLcm(
      get_input_port_reference().Eval<MPCReference>(context),
          context.get_time());
  debug->utime = debug->solution.utime;
  debug->n_footsteps = trajopt_.n_footsteps();
  debug->footsteps.clear();

  for (const auto& p: trajopt_.get_footstep_solutions(mpc_solution.sqp_iterate.x_sol)) {
    auto footstep_vec = CopyVectorXdToStdVector(p);
    debug->footsteps.push_back(footstep_vec);
  }
}

std::vector<Eigen::Vector3d> IDMPCWalkingSystem::CalcInitialFootsteps(
    const VectorXd &q, const MPCReference &ref) const {

  const MultibodyPlant<double>& plant = trajopt_.dynamics().get_plant();
  plant.SetPositions(plant_context_.get(), q);

  std::vector<Vector3d> pp(trajopt_.n_footsteps(), Vector3d::Zero());

  int idx = 1;
  for (size_t i = 0; i < ref.touchdown_ee_names_.size(); ++i) {
    if (not ref.touchdown_ee_names_[i].empty()) {
      Vector3d p;
      const std::string& contact_frame = ref.touchdown_ee_names_[i];
      plant.CalcPointsPositions(
          *plant_context_, plant.GetBodyByName(contact_frame).body_frame(),
          ref.touchdown_ee_points_[i], plant.world_frame(), &p);
      pp.at(idx) = p;
      ++idx;
    }
  }
  return pp;
}

std::vector<Eigen::Vector3d> IDMPCWalkingSystem::CalcFootstepLocations(
    const MPCReference& ref, const Eigen::VectorXd& z) const {

  DRAKE_DEMAND(not ref.touchdown_ee_names_to_update_.empty());

  std::vector<Vector3d> pp = std::vector<Vector3d>(
      trajopt_.n_footsteps(),Vector3d::Zero());

  int idx = 1;
  const auto& plant = trajopt_.dynamics().get_plant();

  for (size_t i = 0; i < ref.touchdown_ee_names_to_update_.size(); ++i) {
    const std::string& contact_frame = ref.touchdown_ee_names_to_update_.at(i);
    if (not contact_frame.empty()) {
      Vector3d p;
      const VectorXd q =
          trajopt_.mpc().GetDecisionVariableValue(trajopt_.mpc().position_vars(i), z);
      plant.CalcPointsPositions(
          *plant_context_, plant.GetBodyByName(contact_frame).body_frame(),
          ref.touchdown_ee_points_[i], plant.world_frame(), &p);
      pp.at(idx) = p;
      ++idx;
    }
  }

  return pp;
}


void IDMPCWalkingSystem::SetInitialSolverStateToCurrent(
    const OutputVector<double> &x_u_t,
    const std::vector<std::vector<std::string>>& contacts,
    SQPIterate &solver_state) const {

  trajopt_.dynamics().SetPlantStateIfNew(
      x_u_t.GetState(), plant_context_.get());

  auto& mpc = trajopt_.mutable_mpc();

  for (size_t i = 0; i < contacts.size() ; ++i) {
    VectorXd lambda = trajopt_.dynamics().EstimateConstraintForcesForFixedPoint(
        *plant_context_, x_u_t.GetEfforts(), contacts.at(i)
    );
    mpc.get_prog().SetInitialGuess(
        mpc.position_vars(i), x_u_t.GetPositions());
    mpc.get_prog().SetInitialGuess(
        mpc.velocity_vars(i), x_u_t.GetVelocities());
    if (mpc.has_lambdas_at_knot(i)) {
      mpc.get_prog().SetInitialGuess(
          mpc.lambda_vars(i), lambda);
    }
    if (mpc.has_torques_at_knot(i)) {
      mpc.get_prog().SetInitialGuess(
          mpc.input_vars(i), x_u_t.GetEfforts());
    }
  }
  solver_state.x_sol = mpc.get_prog().initial_guess();
}

void IDMPCWalkingSystem::ShiftSolution(const std::vector<double> &knots,
                                       MPCSolution *prev_sol) const {

  SolutionTraj prev_trajs = SolutionTraj::FromLcmTrajectory(
      prev_sol->solution_trajectories,
      trajopt_.dynamics().get_plant(),
      plant_context_.get()
  );

  auto& mpc = trajopt_.mutable_mpc();
  VectorXd* z = &prev_sol->sqp_iterate.x_sol;

  for (size_t i = 0; i < knots.size() ; ++i) {
    double t = knots.at(i);
    mpc.SetDecisionVariableValue(
        mpc.position_vars(i), prev_trajs.q.value(t), z);
    mpc.SetDecisionVariableValue(
        mpc.velocity_vars(i), prev_trajs.v.value(t), z);
    if (mpc.has_torques_at_knot(i)) {
      mpc.SetDecisionVariableValue(
          mpc.input_vars(i), prev_trajs.u.value(t), z);
    }
    if (mpc.has_lambdas_at_knot(i)) {
      mpc.SetDecisionVariableValue(
          mpc.lambda_vars(i), prev_trajs.lambda.value(t), z);
    }
  }
}

}