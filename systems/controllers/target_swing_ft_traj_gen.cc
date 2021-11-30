#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"
#include "drake/math/saturate.h"
#include "systems/controllers/target_swing_ft_traj_gen.h"
#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers {

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;

using drake::systems::DiscreteValues;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::EventStatus;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix;

using multibody::SetPositionsAndVelocitiesIfNew;

TargetSwingFtTrajGen::TargetSwingFtTrajGen(
    const MultibodyPlant<double>& plant,
    const std::vector<int>& left_right_support_fsm_states,
    const std::vector<double>& left_right_support_durations,
    const std::vector<std::pair<const Eigen::Vector3d,
                                const drake::multibody::Frame<double>&>> pts,
    const SwingFootTajGenOptions opts) :
    plant_(plant),
    plant_context_(plant_.CreateDefaultContext()),
    opts_(opts),
    left_right_support_fsm_states_(left_right_support_fsm_states),
    pts_(pts) {

  /* PORTS */
  state_port_ = this->DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(),
          plant.num_velocities(),
          plant.num_actuators())).get_index();
  fsm_port_ =
      this->DeclareVectorInputPort(
          "fsm", BasicVector<double>(1)).get_index();
  liftoff_time_port_ =
      this->DeclareVectorInputPort(
          "t_liftoff", BasicVector<double>(1)).get_index();

  foot_target_port_ =
      this->DeclareVectorInputPort(
          "foot_target", BasicVector<double>(3)).get_index();

  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  drake::trajectories::Trajectory<double> &traj_instance = pp;

  this->DeclareAbstractOutputPort("swing_foot_xyz", traj_instance,
                                  &TargetSwingFtTrajGen::CalcTrajs);

  DeclarePerStepDiscreteUpdateEvent(
      &TargetSwingFtTrajGen::DiscreteVariableUpdate);

  /* STATE VARIABLES */
  starting_foot_pos_idx_ = this->DeclareDiscreteState(3);
  prev_fsm_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));
  // Construct maps
  duration_map_.insert({left_right_support_fsm_states.at(0),
                        left_right_support_durations.at(0)});
  duration_map_.insert({left_right_support_fsm_states.at(1),
                        left_right_support_durations.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), pts.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(1), pts.at(0)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(0), pts.at(0)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(1), pts.at(1)});
}

void TargetSwingFtTrajGen::CalcTrajs(
    const Context<double> &context, Trajectory<double> *traj) const {
  auto *pp_traj =
  (PiecewisePolynomial<double> *) dynamic_cast<PiecewisePolynomial<double>*>(
      traj);

  int fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value()[0];
  double liftoff_time =
      this->EvalVectorInput(context, liftoff_time_port_)->get_value()[0];
  // swing phase if current state is in left_right_support_fsm_states_
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), fsm_state);
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  Vector3d target_pos =
      this->EvalVectorInput(context, foot_target_port_)->get_value();

  if (is_single_support_phase && !target_pos.isApprox(Vector3d::Zero(), 1e-10)) {
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    double end_time = liftoff_time + duration_map_.at(fsm_state);
    double stance_ft_height = CalcStanceFootHeight(
        robot_output->GetState(),stance_foot_map_.at(fsm_state));

    Vector3d starting_pos =
        context.get_discrete_state(starting_foot_pos_idx_).get_value();
    *pp_traj = CreateSplineForSwingFoot(
        liftoff_time, end_time, starting_pos, target_pos, stance_ft_height);

  } else if (is_single_support_phase) {
    Vector3d pos;
    plant_.CalcPointsPositions(
        *plant_context_, swing_foot_map_.at(fsm_state).second,
        swing_foot_map_.at(fsm_state).first,
        plant_.world_frame(), &pos);
    *pp_traj = PiecewisePolynomial<double>(pos);
    std::cout << "constant position" << std::endl;
  } else {
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}


EventStatus TargetSwingFtTrajGen::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  VectorXd fsm_state = this->EvalVectorInput(context, fsm_port_)->get_value();

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_idx_)
      .get_mutable_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));
  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // when entering a new state which is in left_right_support_fsm_states
  if ((fsm_state(0) != prev_fsm_state(0)) && is_single_support_phase) {
    prev_fsm_state(0) = fsm_state(0);

    auto swing_foot_pos_at_liftoff =
        discrete_state->get_mutable_vector(starting_foot_pos_idx_)
            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_,
                                         q,
                                         plant_context_.get());

    // Swing foot position (Forward Kinematics) at touchdown
    auto swing_foot = swing_foot_map_.at(int(fsm_state(0)));
    plant_.CalcPointsPositions(*plant_context_,
                               swing_foot.second,
                               swing_foot.first,
                               plant_.world_frame(),
                               &swing_foot_pos_at_liftoff);
  }

  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> TargetSwingFtTrajGen::CreateSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval,
    const Eigen::Vector3d& init_foot_pos,
    const Eigen::Vector3d& target_foot_pos,
    double stance_foot_height) const {

  Matrix3d knots;
  Vector3d breaks = {start_time_of_this_interval,
                     0.5 * (start_time_of_this_interval + end_time_of_this_interval),
                     end_time_of_this_interval};
  Vector3d final_foot_pos = target_foot_pos +
      Vector3d(0, 0, stance_foot_height);
  Vector3d starting_foot_pos = init_foot_pos +
      Vector3d(0, 0, stance_foot_height);

  knots.col(0) = starting_foot_pos;
  knots.col(1) = 0.5 * (starting_foot_pos + final_foot_pos) +
                            Vector3d(0, 0, opts_.mid_foot_height);
  knots.col(2) = final_foot_pos;

  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
      breaks, knots, Vector3d::Zero(),
      Vector3d(0, 0, opts_.desired_final_vertical_foot_velocity));

}

double TargetSwingFtTrajGen::CalcStanceFootHeight(
    const Eigen::VectorXd& x,
    const std::pair<const Eigen::Vector3d,
                    const drake::multibody::Frame<double>&> pt) const {
  SetPositionsAndVelocitiesIfNew<double>(plant_, x, plant_context_.get());
  Vector3d pos;
  plant_.CalcPointsPositions(*plant_context_, pt.second, pt.first,
                             plant_.world_frame(), &pos);
  return pos(2);
}
}