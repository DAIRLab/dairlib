#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"
#include "lcm/lcm_trajectory.h"
#include "centroidal_ik_traj_gen.h"
#include "dairlib/lcmt_saved_traj.hpp"

#include "drake/solvers/solve.h"
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

using drake::AutoDiffXd;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::VectorXDecisionVariable;
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
    const MultibodyPlant<double>& plant,
    const Matrix3d& I, double mass, double dt, double stance_duration) :
    plant_ad_(plant_ad),
    plant_(plant),
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

  BasicVector<double> model_solution_vec(
      2 * plant.num_positions() + 2* plant.num_velocities());
  ik_sol_cache_entry_ = &DeclareCacheEntry(
      "ik_sol", model_solution_vec,
      &CentroidalIKTrajGen::CalcTraj,
      {input_port_ticket(TypeSafeIndex(mpc_traj_port_))});
}

void CentroidalIKTrajGen::CalcTraj(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *solvec) const {

  /* Read input ports */
  const drake::AbstractValue* input =
      this->EvalAbstractInput(context, mpc_traj_port_);
  DRAKE_ASSERT(input != nullptr);
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
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
  Eigen::MatrixXd angular_momentum = I_b_ *
      mpc_traj.GetTrajectory("orientation").datapoints.bottomRows(3);
  Eigen::MatrixXd linear_momentum = mass_ *
      mpc_traj.GetTrajectory("com_traj").datapoints.bottomRows(3);


  // Solve the IK problem
  double timestamp = robot_output->get_timestamp();
  double start_time = timestamp;
  double end_time = prev_event_time + stance_duration_;
  double mode_duration = end_time - start_time;

  start_time = drake::math::saturate(start_time, -1,end_time - 0.001);
  MathematicalProgram prog;
  std::vector<VectorXDecisionVariable> qq, vv, hh, rr;

  int N = round(mode_duration / dt_) + 1;

  // Make decision variables
  for (int i = 0; i < N; i++) {
    qq.push_back(prog.NewContinuousVariables(
      plant_.num_positions(), "q" + std::to_string(i)));
    vv.push_back(prog.NewContinuousVariables(
        plant_.num_velocities(), "v" + std::to_string(i)));
    hh.push_back(prog.NewContinuousVariables(6, "h" + std::to_string(i)));
    rr.push_back(prog.NewContinuousVariables(3));
  }

  // Initial state constraint
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_output->GetState(), context_);
  auto CoM = plant_.CalcCenterOfMassPositionInWorld(*context_);
  auto spatial_momentum = plant_.CalcSpatialMomentumInWorldAboutPoint(*context_, CoM);
  VectorXd h_i = VectorXd::Zero(6);
  h_i.head(3) = spatial_momentum.rotational();
  h_i.tail(3) = spatial_momentum.translational();
  prog.AddLinearEqualityConstraint(qq.front() == robot_output->GetPositions());
  prog.AddLinearEqualityConstraint(vv.front() == robot_output->GetVelocities());
  prog.AddLinearEqualityConstraint(hh.front() == h_i);

  // knot point constraints
  Vector3d stance_foot_pos, swing_foot_pos_i, swing_foot_pos_f;

  plant_.CalcPointsPositions(
      *context_,
      plant_.GetBodyByName(toe_frames_.at(stance_mode)).body_frame(),
      toe_mid_, plant_.world_frame(), &stance_foot_pos);
  plant_.CalcPointsPositions(
      *context_,
      plant_.GetBodyByName(toe_frames_.at(1-stance_mode)).body_frame(),
      toe_mid_, plant_.world_frame(), &swing_foot_pos_i);
  swing_foot_pos_f =
      mpc_traj.GetTrajectory("swing_foot_traj").datapoints;
  auto centroidal_constraint_ptr =
      std::make_shared<CentroidalMomentumConstraint>(
          &plant_ad_,std::vector<drake::multibody::ModelInstanceIndex>(),
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
      &plant_, std::vector<drake::multibody::ModelInstanceIndex>(),
          plant_.world_frame(), context_);

  for (int i = 1; i < N; i++) {
    prog.AddConstraint(centroidal_constraint_ptr, {qq.at(i), vv.at(i), hh.at(i)});
    prog.AddConstraint(stance_foot_pos_contraint, qq.at(i));
    prog.AddConstraint(quat_norm_constraint, qq.at(i).head(4));
    prog.AddConstraint(com_pos_constraint, {qq.at(i), rr.at(i)});
    prog.AddLinearEqualityConstraint(hh.at(i).head(3) == angular_momentum.col(i));
    prog.AddLinearEqualityConstraint(hh.at(i).tail(3) == linear_momentum.col(i));
    prog.AddQuadraticCost((vv.at(i) - vv.at(i-1)).transpose() * (vv.at(i) - vv.at(i-1)));
  }

  auto result = drake::solvers::Solve(prog);

  if (!result.is_success()) {
    throw std::runtime_error(
        "Error, IK did not solve, exited with status " +
        std::to_string(result.get_solution_result()));
  }
}
}