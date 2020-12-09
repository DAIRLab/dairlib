#include "systems/controllers/swing_ft_traj_gen.h"

#include <math.h>
#include <algorithm>  // std::min
#include <string>

#include "systems/controllers/control_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;

using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace systems {

SwingFootTrajGenerator::SwingFootTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::vector<int> left_right_support_fsm_states,
    std::vector<double> left_right_support_durations,
    std::vector<std::pair<const Vector3d, const Frame<double>&>>
        left_right_foot,
    std::string floating_base_body_name, double mid_foot_height,
    double desired_final_foot_height,
    double desired_final_vertical_foot_velocity,
    double max_com_to_footstep_dist, double footstep_offset,
    double center_line_offset, bool add_speed_regularization,
    bool is_feet_collision_avoid, bool is_using_predicted_com,
    int footstep_option)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName(floating_base_body_name)),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      max_com_to_footstep_dist_(max_com_to_footstep_dist),
      footstep_offset_(footstep_offset),
      center_line_offset_(center_line_offset),
      add_speed_regularization_(add_speed_regularization),
      is_feet_collision_avoid_(is_feet_collision_avoid),
      is_using_predicted_com_(is_using_predicted_com),
      footstep_option_(footstep_option) {
  this->set_name("swing_ft_traj");

  DRAKE_DEMAND(0 <= footstep_option && footstep_option <= 1);

  DRAKE_DEMAND(left_right_support_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_support_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  fsm_switch_time_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  if (is_using_predicted_com) {
    com_port_ =
        this->DeclareAbstractInputPort(
                "com_traj",
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
  }
  if (add_speed_regularization) {
    speed_control_port_ =
        this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  }
  // Provide an instance to allocate the memory first (for the output)
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("swing_ft_traj", traj_instance,
                                  &SwingFootTrajGenerator::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(
      &SwingFootTrajGenerator::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  prev_liftoff_swing_foot_idx_ = this->DeclareDiscreteState(3);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(
      -std::numeric_limits<double>::infinity() * VectorXd::Ones(1));

  // Construct maps
  duration_map_.insert({left_right_support_fsm_states.at(0),
                        left_right_support_durations.at(0)});
  duration_map_.insert({left_right_support_fsm_states.at(1),
                        left_right_support_durations.at(1)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(0)});
  stance_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(1), left_right_foot.at(0)});
}

EventStatus SwingFootTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
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
        discrete_state->get_mutable_vector(prev_liftoff_swing_foot_idx_)
            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    VectorXd q = robot_output->GetPositions();
    multibody::SetPositionsIfNew<double>(plant_, q, context_);

    // Swing foot position (Forward Kinematics) at touchdown
    auto swing_foot = swing_foot_map_.at(int(fsm_state(0)));
    plant_.CalcPointsPositions(*context_, swing_foot.second, swing_foot.first,
                               world_, &swing_foot_pos_at_liftoff);
  }

  return EventStatus::Succeeded();
}

void SwingFootTrajGenerator::CalcFootStepAndStanceFootHeight(
    const Context<double>& context, const OutputVector<double>* robot_output,
    const double end_time_of_this_interval, Vector2d* x_fs,
    double* stance_foot_height) const {
  // Read in finite state machine
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  VectorXd q = robot_output->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Stance foot position
  auto stance_foot = stance_foot_map_.at(int(fsm_state(0)));
  Vector3d stance_foot_pos;
  plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                             world_, &stance_foot_pos);

  // Get com or predicted com
  Vector3d com;
  Vector3d com_dot;
  if (is_using_predicted_com_) {
    // com and com_dot at the end of the step (predicted)
    const drake::AbstractValue* com_traj_output =
        this->EvalAbstractInput(context, com_port_);
    DRAKE_ASSERT(com_traj_output != nullptr);
    const auto& com_traj =
        com_traj_output->get_value<drake::trajectories::Trajectory<double>>();
    com = com_traj.value(end_time_of_this_interval);
    com_dot = com_traj.MakeDerivative(1)->value(end_time_of_this_interval);
  } else {
    // Get the current center of mass position and velocity

    MatrixXd J_com(3, plant_.num_velocities());
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, world_, world_, &J_com);
    VectorXd v = robot_output->GetVelocities();
    com = plant_.CalcCenterOfMassPosition(*context_);
    com_dot = J_com * v;
  }

  // Filter the com vel
  if (robot_output->get_timestamp() != last_timestamp_) {
    double dt = robot_output->get_timestamp() - last_timestamp_;
    last_timestamp_ = robot_output->get_timestamp();
    double alpha =
        2 * M_PI * dt * cutoff_freq_ / (2 * M_PI * dt * cutoff_freq_ + 1);
    filterred_com_vel_ = alpha * com_dot + (1 - alpha) * filterred_com_vel_;
  }
  com_dot = filterred_com_vel_;

  // Compute footstep location
  double omega = sqrt(9.81 / com(2));
  if (footstep_option_ == 0) {
    *x_fs << (com(0) + com_dot(0) / omega), (com(1) + com_dot(1) / omega);
  } else if (footstep_option_ == 1) {
    // Use LIPM to derive neutral point
    double T = duration_map_.at(int(fsm_state(0)));
    Vector2d com_wrt_foot =
        com_dot.head(2) *
        ((exp(omega * T) - 1) / (exp(2 * omega * T) - 1) -
         (exp(-omega * T) - 1) / (exp(-2 * omega * T) - 1)) /
        omega;
    *x_fs = com.head(2) - com_wrt_foot;
  }

  // Walking position control
  if (add_speed_regularization_) {
    // Read in speed regularization term
    auto speed_control_output = (BasicVector<double>*)this->EvalVectorInput(
        context, speed_control_port_);
    *x_fs += speed_control_output->get_value();
  }

  if (is_feet_collision_avoid_) {
    // Get approximated heading angle of pelvis
    Vector3d pelvis_heading_vec =
        plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
    double approx_pelvis_yaw =
        atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

    // Shift footstep a little away from com line and toward the swing foot, so
    // that the foot placement position at steady state is right below the hip
    // joint
    Vector2d shift_foothold_dir;
    if (fsm_state(0) == left_right_support_fsm_states_[1]) {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 1 / 2),
          sin(approx_pelvis_yaw + M_PI * 1 / 2);
    } else {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 3 / 2),
          sin(approx_pelvis_yaw + M_PI * 3 / 2);
    }
    *x_fs += shift_foothold_dir * footstep_offset_;

    *x_fs = ImposeHalfplaneGuard(
        *x_fs, (fsm_state(0) == left_right_support_fsm_states_[0]),
        approx_pelvis_yaw, com.head(2), stance_foot_pos.head(2),
        center_line_offset_);
  }

  // Cap by the step length
  *x_fs = ImposeStepLengthGuard(*x_fs, com.head(2), max_com_to_footstep_dist_);

  // Assignment for stance foot height
  *stance_foot_height = stance_foot_pos(2);
}

PiecewisePolynomial<double> SwingFootTrajGenerator::CreateSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const double stance_duration,
    const Vector3d& init_swing_foot_pos, const Vector2d& x_fs,
    double stance_foot_height) const {
  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {
      start_time_of_this_interval,
      (start_time_of_this_interval + end_time_of_this_interval) / 2,
      end_time_of_this_interval};


  //TODO(yangwill): make the acceleration continuous
  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y[0](0, 0) = init_swing_foot_pos(0);
  Y[1](0, 0) = (init_swing_foot_pos(0) + x_fs(0)) / 2;
  Y[2](0, 0) = x_fs(0);
  // y
  Y[0](1, 0) = init_swing_foot_pos(1);
  Y[1](1, 0) = (init_swing_foot_pos(1) + x_fs(1)) / 2;
  Y[2](1, 0) = x_fs(1);
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = init_swing_foot_pos(2);
  Y[1](2, 0) = mid_foot_height_ + stance_foot_height;
  Y[2](2, 0) = desired_final_foot_height_ + stance_foot_height;

  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = (x_fs(0) - init_swing_foot_pos(0)) / stance_duration;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = (x_fs(1) - init_swing_foot_pos(1)) / stance_duration;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = 0;
  Y_dot[1](2, 0) = 0;
  Y_dot[2](2, 0) = desired_final_vertical_foot_velocity_;
  // Use CubicWithContinuousSecondDerivatives instead of CubicHermite to make
  // the traj smooth at the mid point
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives
          (T_waypoint, Y, Y_dot.at(0), Y_dot.at(2));

  return swing_foot_spline;
}

void SwingFootTrajGenerator::CalcTrajs(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj for polymorphism
  auto* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // Get discrete states
  const auto swing_foot_pos_at_liftoff =
      context.get_discrete_state(prev_liftoff_swing_foot_idx_).get_value();
  // Read in finite state machine switch time
  VectorXd prev_lift_off_time =
      this->EvalVectorInput(context, fsm_switch_time_port_)->get_value();

  // Read in finite state machine
  auto fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // Generate trajectory if it's currently in swing phase.
  // Otherwise, generate a constant trajectory
  if (is_single_support_phase) {
    // Read in current robot state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get current time
    double timestamp = robot_output->get_timestamp();
    auto current_time = static_cast<double>(timestamp);

    // Get the start time and the end time of the current stance phase
    double start_time_of_this_interval = prev_lift_off_time(0);
    double end_time_of_this_interval =
        prev_lift_off_time(0) + duration_map_.at(int(fsm_state(0)));

    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    if ((end_time_of_this_interval <= current_time + 0.001)) {
      end_time_of_this_interval = current_time + 0.002;
    }

    // Get Capture Point
    double stance_foot_height;
    Vector2d x_fs(0, 0);
    CalcFootStepAndStanceFootHeight(context, robot_output,
                                    end_time_of_this_interval, &x_fs,
                                    &stance_foot_height);

    // Swing foot position at touchdown
    Vector3d init_swing_foot_pos = swing_foot_pos_at_liftoff;

    // Assign traj
    *pp_traj = CreateSplineForSwingFoot(
        start_time_of_this_interval, end_time_of_this_interval,
        duration_map_.at(int(fsm_state(0))), init_swing_foot_pos, x_fs,
        stance_foot_height);

  } else {
    // Assign a constant traj
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}
}  // namespace systems
}  // namespace dairlib
