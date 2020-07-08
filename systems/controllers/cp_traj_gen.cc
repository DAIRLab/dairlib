#include "systems/controllers/cp_traj_gen.h"

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

CPTrajGenerator::CPTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    std::vector<int> left_right_support_fsm_states,
    std::vector<double> left_right_support_durations,
    std::vector<std::pair<const Vector3d, const Frame<double>&>>
        left_right_foot,
    std::string floating_base_body_name, double mid_foot_height,
    double desired_final_foot_height,
    double desired_final_vertical_foot_velocity, double max_CoM_to_CP_dist,
    bool add_extra_control, bool is_feet_collision_avoid,
    bool is_using_predicted_com, double cp_offset, double center_line_offset)
    : plant_(plant),
      left_right_support_fsm_states_(left_right_support_fsm_states),
      mid_foot_height_(mid_foot_height),
      desired_final_foot_height_(desired_final_foot_height),
      desired_final_vertical_foot_velocity_(
          desired_final_vertical_foot_velocity),
      max_CoM_to_CP_dist_(max_CoM_to_CP_dist),
      add_extra_control_(add_extra_control),
      is_feet_collision_avoid_(is_feet_collision_avoid),
      is_using_predicted_com_(is_using_predicted_com),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName(floating_base_body_name)),
      cp_offset_(cp_offset),
      center_line_offset_(center_line_offset) {
  this->set_name("cp_traj");

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

  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  if (is_using_predicted_com) {
    com_port_ =
        this->DeclareAbstractInputPort(
                "CoM_traj",
                drake::Value<drake::trajectories::Trajectory<double>>(pp))
            .get_index();
  }
  if (add_extra_control) {
    fp_port_ = this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  }
  // Provide an instance to allocate the memory first (for the output)
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("cp_traj", traj_instance,
                                  &CPTrajGenerator::CalcTrajs);

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&CPTrajGenerator::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  prev_td_swing_foot_idx_ = this->DeclareDiscreteState(3);
  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));

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
      {left_right_support_fsm_states.at(1), left_right_foot.at(1)});
  swing_foot_map_.insert(
      {left_right_support_fsm_states.at(0), left_right_foot.at(0)});

  // Create context
  context_ = plant_.CreateDefaultContext();
}

EventStatus CPTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
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

    auto swing_foot_pos_td =
        discrete_state->get_mutable_vector(prev_td_swing_foot_idx_)
            .get_mutable_value();
    auto prev_td_time = discrete_state->get_mutable_vector(prev_td_time_idx_)
                            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get time
    double timestamp = robot_output->get_timestamp();
    double current_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_time;

    VectorXd q = robot_output->GetPositions();
    plant_.SetPositions(context_.get(), q);

    // Swing foot position (Forward Kinematics) at touchdown
    auto swing_foot = swing_foot_map_.at(int(fsm_state(0)));
    plant_.CalcPointsPositions(*context_, swing_foot.second, swing_foot.first,
                               world_, &swing_foot_pos_td);
  }

  return EventStatus::Succeeded();
}

void CPTrajGenerator::calcCpAndStanceFootHeight(
    const Context<double>& context, const OutputVector<double>* robot_output,
    const double end_time_of_this_interval, Vector2d* final_CP,
    VectorXd* stance_foot_height) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  VectorXd q = robot_output->GetPositions();
  plant_.SetPositions(context_.get(), q);

  // Stance foot position
  auto stance_foot = stance_foot_map_.at(int(fsm_state(0)));
  Vector3d stance_foot_pos;
  plant_.CalcPointsPositions(*context_, stance_foot.second, stance_foot.first,
                             world_, &stance_foot_pos);

  // Get CoM or predicted CoM
  Vector3d CoM;
  Vector3d dCoM;
  if (is_using_predicted_com_) {
    // CoM and dCoM at the end of the step (predicted)
    const drake::AbstractValue* com_traj_output =
        this->EvalAbstractInput(context, com_port_);
    DRAKE_ASSERT(com_traj_output != nullptr);
    const auto& com_traj =
        com_traj_output->get_value<drake::trajectories::Trajectory<double>>();
    CoM = com_traj.value(end_time_of_this_interval);
    dCoM = com_traj.MakeDerivative(1)->value(end_time_of_this_interval);
  } else {
    // Get the current center of mass position and velocity

    MatrixXd J_com(3, plant_.num_velocities());
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, world_, world_, &J_com);
    VectorXd v = robot_output->GetVelocities();
    CoM = plant_.CalcCenterOfMassPosition(*context_);
    dCoM = J_com * v;
  }

  double pred_omega = sqrt(9.81 / CoM(2));

  Vector2d CP;
  CP << (CoM(0) + dCoM(0) / pred_omega), (CoM(1) + dCoM(1) / pred_omega);

  // Walking position control
  if (add_extra_control_) {
    // Read in foot placement
    const BasicVector<double>* fp_output =
        (BasicVector<double>*)this->EvalVectorInput(context, fp_port_);
    CP += fp_output->get_value();
  }

  if (is_feet_collision_avoid_) {
    // Get approximated heading angle of pelvis
    Vector3d pelvis_heading_vec =
        plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
    double approx_pelvis_yaw =
        atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

    // Shift CP a little away from CoM line and toward the swing foot, so that
    // the foot placement position at steady state is right below the hip joint
    Vector2d shift_foothold_dir;
    if (fsm_state(0) == left_right_support_fsm_states_[1]) {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 1 / 2),
          sin(approx_pelvis_yaw + M_PI * 1 / 2);
    } else {
      shift_foothold_dir << cos(approx_pelvis_yaw + M_PI * 3 / 2),
          sin(approx_pelvis_yaw + M_PI * 3 / 2);
    }
    CP = CP + shift_foothold_dir * cp_offset_;

    CP = ImposeHalfplaneGuard(
        CP, (fsm_state(0) == left_right_support_fsm_states_[0]),
        approx_pelvis_yaw, CoM.head(2), stance_foot_pos.head(2),
        center_line_offset_);
  }

  // Cap by the step length
  CP = ImposeStepLengthGuard(CP, CoM.head(2), max_CoM_to_CP_dist_);

  // Assignment
  (*stance_foot_height)(0) = stance_foot_pos(2);
  *final_CP = CP;
}

PiecewisePolynomial<double> CPTrajGenerator::createSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const double stance_duration,
    const Vector3d& init_swing_foot_pos, const Vector2d& CP,
    const VectorXd& stance_foot_height) const {
  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {
      start_time_of_this_interval,
      (start_time_of_this_interval + end_time_of_this_interval) / 2,
      end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y[0](0, 0) = init_swing_foot_pos(0);
  Y[1](0, 0) = (init_swing_foot_pos(0) + CP(0)) / 2;
  Y[2](0, 0) = CP(0);
  // y
  Y[0](1, 0) = init_swing_foot_pos(1);
  Y[1](1, 0) = (init_swing_foot_pos(1) + CP(1)) / 2;
  Y[2](1, 0) = CP(1);
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = init_swing_foot_pos(2);
  Y[1](2, 0) = mid_foot_height_ + stance_foot_height(0);
  Y[2](2, 0) = desired_final_foot_height_ + stance_foot_height(0);

  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = (CP(0) - init_swing_foot_pos(0)) / stance_duration;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = (CP(1) - init_swing_foot_pos(1)) / stance_duration;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = 0;
  Y_dot[1](2, 0) = 0;
  Y_dot[2](2, 0) = desired_final_vertical_foot_velocity_;
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::CubicHermite(T_waypoint, Y, Y_dot);

  return swing_foot_spline;
}

void CPTrajGenerator::CalcTrajs(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Cast traj for polymorphism
  PiecewisePolynomial<double>* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  // Get discrete states
  const auto swing_foot_pos_td =
      context.get_discrete_state(prev_td_swing_foot_idx_).get_value();
  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();

  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  // Find fsm_state in left_right_support_fsm_states
  auto it = find(left_right_support_fsm_states_.begin(),
                 left_right_support_fsm_states_.end(), int(fsm_state(0)));

  // swing phase if current state is in left_right_support_fsm_states_
  bool is_single_support_phase = it != left_right_support_fsm_states_.end();

  // Generate trajectory based on CP if it's currently in swing phase.
  // Otherwise, generate a constant trajectory
  if (is_single_support_phase) {
    // Read in current robot state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get current time
    double timestamp = robot_output->get_timestamp();
    auto current_time = static_cast<double>(timestamp);

    // Get the start time and the end time of the current stance phase
    double start_time_of_this_interval = prev_td_time(0);
    double end_time_of_this_interval =
        prev_td_time(0) + duration_map_.at(int(fsm_state(0)));

    // Ensure current_time < end_time_of_this_interval to avoid error in
    // creating trajectory.
    if ((end_time_of_this_interval <= current_time + 0.001)) {
      end_time_of_this_interval = current_time + 0.002;
    }

    // Get Capture Point
    VectorXd stance_foot_height = VectorXd::Zero(1);
    Vector2d CP(0, 0);
    calcCpAndStanceFootHeight(context, robot_output, end_time_of_this_interval,
                              &CP, &stance_foot_height);

    // Swing foot position at touchdown
    Vector3d init_swing_foot_pos = swing_foot_pos_td;

    // Assign traj
    *pp_traj = createSplineForSwingFoot(
        start_time_of_this_interval, end_time_of_this_interval,
        duration_map_.at(int(fsm_state(0))), init_swing_foot_pos, CP,
        stance_foot_height);

  } else {
    // Assign a constant traj
    *pp_traj = PiecewisePolynomial<double>(Vector3d::Zero());
  }
}
}  // namespace systems
}  // namespace dairlib
