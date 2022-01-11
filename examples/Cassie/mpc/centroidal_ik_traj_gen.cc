#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"
#include "lcm/lcm_trajectory.h"
#include "centroidal_ik_traj_gen.h"
#include "dairlib/lcmt_saved_traj.hpp"

#include "drake/solvers/solve.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/com_position_constraint.h"
#include "drake/math/saturate.h"

using dairlib::multibody::SingleRigidBodyPlant;
using dairlib::multibody::SetPositionsAndVelocitiesIfNew;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

using drake::AutoDiffXd;
using drake::math::ExtractValue;
using drake::math::RollPitchYaw;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::SnoptSolver;
using drake::multibody::MultibodyPlant;
using drake::multibody::CentroidalMomentumConstraint;
using drake::multibody::UnitQuaternionConstraint;
using drake::multibody::PositionConstraint;
using drake::multibody::ComPositionConstraint;
using drake::systems::Context;
using drake::systems::BasicVector;
using dairlib::systems::OutputVector;
using drake::TypeSafeIndex;

namespace dairlib::mpc{

CentroidalIKTrajGen::CentroidalIKTrajGen(
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    Context<AutoDiffXd>* context_ad,
    const MultibodyPlant<double>& plant,
    Context<double>* context,
    const Matrix3d& I, double mass, double dt, double stance_duration) :
    plant_ad_(plant_ad),
    context_ad_(context_ad),
    plant_(plant),
    context_(context),
    I_b_(I),
    mass_(mass),
    dt_(dt),
    stance_duration_(stance_duration) {

  fsm_port_ = this->DeclareVectorInputPort("fsm", 1).get_index();
  touchdown_time_port_ = this->DeclareVectorInputPort(
      "touchdown_time", 1).get_index();
  state_port_ = this->DeclareVectorInputPort(
      "x, u, t",
      OutputVector<double>(plant.num_positions(),
                                       plant.num_velocities(),
                                       plant.num_actuators())).get_index();
  mpc_traj_port_ = this->DeclareAbstractInputPort(
          "lcmt_saved_trajectory",
          drake::Value<dairlib::lcmt_saved_traj>{}).get_index();


  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  pelvis_traj_port_ = this->DeclareAbstractOutputPort(
      "orientation_traj", traj_inst, &CentroidalIKTrajGen::AssignPelvisTraj)
          .get_index();

  swing_foot_traj_port_ = this->DeclareAbstractInputPort(
      "swing_foot_xyz",
      drake::Value<drake::trajectories::Trajectory<double>>(empty_pp_traj))
      .get_index();

  BasicVector<double> model_solution_vec(
      2 * plant.num_positions() + 2* plant.num_velocities());
  ik_sol_cache_entry_ = &DeclareCacheEntry(
      "ik_sol", model_solution_vec,
      &CentroidalIKTrajGen::CalcTraj,
      {input_port_ticket(TypeSafeIndex(mpc_traj_port_))});

  solver_options_.SetOption(SnoptSolver::id(), "Print file", "../snopt.out");

}

void CentroidalIKTrajGen::CalcTraj(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *solvec) const {

  /* Read input ports */
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, mpc_traj_port_);
  DRAKE_ASSERT(input != nullptr);

  const drake::AbstractValue* swing_ft_traj_value =
      this->EvalAbstractInput(context, swing_foot_traj_port_);
  DRAKE_DEMAND(swing_ft_traj_value != nullptr);
  const auto& swing_ft_traj =
      swing_ft_traj_value->get_value<drake::trajectories::Trajectory<double>>();

  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd state = robot_output->GetState();
  double fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  int stance_mode = ((int) fsm_state == 0 || (int) fsm_state == 3) ? 0 : 1;
  double prev_event_time =
      this->EvalVectorInput(context, touchdown_time_port_)->get_value()(0);
  VectorXd x = robot_output->GetState();

  // Only solve IK again if there is a new MPC solution
  const auto& input_msg = input->get_value<lcmt_saved_traj>();
  double mpc_timestamp = input_msg.trajectories.front().time_vec.front();
  if (mpc_timestamp == mpc_timestamp_) {
    solvec->get_mutable_value() = prev_ik_sol_;
    std::cout << "got new mpc message" << std::endl;
    return;
  }
  mpc_timestamp_ = mpc_timestamp;
  LcmTrajectory mpc_traj(input_msg);

  SetPositionsAndVelocitiesIfNew<double>(plant_, state, context_);

  Eigen::MatrixXd angular_momentum = I_b_ *
      mpc_traj.GetTrajectory("orientation").datapoints.bottomRows(3);
  Eigen::MatrixXd linear_momentum = mass_ *
      mpc_traj.GetTrajectory("com_traj").datapoints.bottomRows(3);
  Eigen::MatrixXd com_position =
      mpc_traj.GetTrajectory("com_traj").datapoints.topRows(3);

  // Solve the IK problem
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;
  double end_time = prev_event_time + stance_duration_;
  double mode_duration = end_time - start_time;

  start_time = drake::math::saturate(start_time, -1,end_time - 0.001);
  MathematicalProgram prog;
  std::vector<VectorXDecisionVariable> qq, vv, hh, rr;

  // Make decision variables
  for (int i = 0; i < 2; i++) {
    qq.push_back(prog.NewContinuousVariables(
      plant_.num_positions(), "q" + std::to_string(i)));
    vv.push_back(prog.NewContinuousVariables(
        plant_.num_velocities(), "v" + std::to_string(i)));
    hh.push_back(prog.NewContinuousVariables(6, "h" + std::to_string(i)));
    rr.push_back(prog.NewContinuousVariables(3));
  }

  Vector3d stance_foot_pos;
  Vector3d swing_foot_pos_i = swing_ft_traj.value(start_time);
  Vector3d swing_foot_pos_f = swing_ft_traj.value(start_time + dt_);
  plant_.CalcPointsPositions(
      *context_,
      plant_.GetBodyByName(toe_frames_.at(stance_mode)).body_frame(),
      toe_mid_, plant_.world_frame(), &stance_foot_pos);

  auto centroidal_constraint_ptr =
      std::make_shared<CentroidalMomentumConstraint>(
          &plant_ad_, std::nullopt,
          context_ad_, false);
  auto quat_norm_constraint = std::make_shared<UnitQuaternionConstraint>();
  auto stance_foot_pos_contraint = std::make_shared<PositionConstraint>(
      &plant_ad_, plant_ad_.world_frame(),
      stance_foot_pos, stance_foot_pos,
      plant_ad_.GetBodyByName(toe_frames_.at(stance_mode)).body_frame(),
      toe_mid_, context_ad_);
  auto swing_foot_init_pos_constraint = std::make_shared<PositionConstraint>(
      &plant_ad_, plant_ad_.world_frame(),
      swing_foot_pos_i, swing_foot_pos_i,
      plant_ad_.GetBodyByName(toe_frames_.at(1-stance_mode)).body_frame(),
      toe_mid_, context_ad_);
  auto swing_foot_final_pos_constraint = std::make_shared<PositionConstraint>(
      &plant_ad_, plant_ad_.world_frame(),
      swing_foot_pos_f, swing_foot_pos_f,
      plant_ad_.GetBodyByName(toe_frames_.at(1-stance_mode)).body_frame(),
      toe_mid_, context_ad_);
  auto com_pos_constraint = std::make_shared<ComPositionConstraint>(
      &plant_, std::nullopt,
          plant_.world_frame(), context_);

  for (int i = 0; i < 2; i++) {
    prog.AddConstraint(centroidal_constraint_ptr, {qq.at(i), vv.at(i), hh.at(i)});
    prog.AddConstraint(stance_foot_pos_contraint, qq.at(i));
    prog.AddConstraint(quat_norm_constraint, qq.at(i).head(4));
    prog.AddConstraint(com_pos_constraint, {qq.at(i), rr.at(i)});
    prog.AddLinearEqualityConstraint(hh.at(i).head(3) == angular_momentum.col(i));
    prog.AddLinearEqualityConstraint(hh.at(i).tail(3) == linear_momentum.col(i));
    prog.AddLinearEqualityConstraint(rr.at(i) == com_position.col(i));
    prog.AddCost(vv.at(i).transpose() * vv.at(i));
  }
  for (int i = 0; i < 2; i++) {
    prog.SetInitialGuess(qq.at(i), state.head(plant_.num_positions()));
    prog.SetInitialGuess(vv.at(i), state.tail(plant_.num_velocities()));
    prog.SetInitialGuess(hh.at(i).head(3), angular_momentum.col(i));
    prog.SetInitialGuess(hh.at(i).tail(3), linear_momentum.col(i));
    prog.SetInitialGuess(rr.at(i), com_position.col(i));
  }

  auto result = solver_.Solve(prog,
                              prog.initial_guess(),
                              solver_options_);
  if (!result.is_success()) {
    std::cout << "Attempted IK solve with " << result.get_solver_id() << "\n";
    throw std::runtime_error(
        "Error, IK did not solve, exited with status " +
        drake::solvers::to_string(result.get_solution_result()));
  }

  solvec->get_mutable_value().head(plant_.num_positions()) =
      result.GetSolution(qq.at(0));
  solvec->get_mutable_value().segment(
      plant_.num_positions(), plant_.num_velocities()) =
      result.GetSolution(vv.at(0));
  solvec->get_mutable_value().segment(
      plant_.num_positions() + plant_.num_velocities(), plant_.num_positions()) =
      result.GetSolution(qq.at(1));
  solvec->get_mutable_value().tail(plant_.num_velocities()) =
      result.GetSolution(vv.at(1));
  prev_ik_sol_ = solvec->value();
  breaks_ = Eigen::Vector2d(start_time, start_time + dt_);
}

void CentroidalIKTrajGen::AssignPelvisTraj(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *output_traj) const {
  const VectorXd& x = ik_sol_cache_entry_->
      Eval<BasicVector<double>>(context).get_value();
  const VectorXd& x0 = x.head(plant_.num_positions() + plant_.num_velocities());
  const VectorXd& x1 = x.tail(plant_.num_positions() + plant_.num_velocities());

  MatrixXd knots = MatrixXd::Zero(3,2);
  MatrixXd knots_dot = MatrixXd::Zero(3,2);

  SetPositionsAndVelocitiesIfNew<double>(plant_, x0, context_);
  knots.col(0) = RollPitchYaw<double>(plant_.EvalBodyPoseInWorld(
      *context_, plant_.GetBodyByName("pelvis"))
      .rotation()).vector();
  knots_dot.col(0) =
      plant_.EvalBodySpatialVelocityInWorld(
          *context_,
          plant_.GetBodyByName("pelvis")).rotational();

  SetPositionsAndVelocitiesIfNew<double>(plant_, x1, context_);
  knots.col(1) = RollPitchYaw<double>(plant_.EvalBodyPoseInWorld(
              *context_, plant_.GetBodyByName("pelvis"))
          .rotation()).vector();
  knots_dot.col(1) =
      plant_.EvalBodySpatialVelocityInWorld(
          *context_,
          plant_.GetBodyByName("pelvis")).rotational();
  auto* casted_traj =
  (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
      output_traj);
  *casted_traj = PiecewisePolynomial<double>::CubicHermite(
      breaks_, knots, knots_dot);
}


void CentroidalIKTrajGen::AssignSwingFootTraj(
    const drake::systems::Context<double> &context,
    drake::trajectories::Trajectory<double> *output_traj) const {

}

}