#include "examples/goldilocks_models/controller/planner_preprocessing.h"

#include "examples/Cassie/cassie_utils.h"
#include "solvers/nonlinear_constraint.h"
#include "systems/controllers/osc/osc_utils.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

using std::cout;
using std::endl;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::JacobianWrtVariable;
using drake::solvers::MathematicalProgram;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::SolutionResult;
using drake::solvers::VectorXDecisionVariable;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;
using dairlib::systems::TimestampedVector;

namespace dairlib {
namespace goldilocks_models {

///
/// CurrentStanceFoot
///

CurrentStanceFoot::CurrentStanceFoot(
    const std::vector<int>& left_right_support_fsm_states)
    : left_right_support_fsm_states_(left_right_support_fsm_states) {
  // Input/Output Setup
  controller_signal_port_ =
      this->DeclareVectorInputPort("ctrl_thread", TimestampedVector<double>(5))
          .get_index();

  this->DeclareVectorOutputPort("stance_foot", BasicVector<double>(1),
                                &CurrentStanceFoot::GetStance);
}

void CurrentStanceFoot::GetStance(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* stance_foot) const {
  // Read in fsm state and lift-off time
  const BasicVector<double>* controller_signal_port =
      this->EvalVectorInput(context, controller_signal_port_);
  int fsm_state = (int)controller_signal_port->get_value()(0);

  // Find fsm_state in left_right_support_fsm_states_
  bool is_single_support_phase =
      find(left_right_support_fsm_states_.begin(),
           left_right_support_fsm_states_.end(),
           fsm_state) != left_right_support_fsm_states_.end();
  if (!is_single_support_phase) {
    // do nothing here to use the previous start_with_right_stance_
  } else {
    start_with_right_stance_ =
        fsm_state == left_right_support_fsm_states_.at(1);
  }

  // Assign
  stance_foot->get_mutable_value() =
      start_with_right_stance_ * VectorXd::Ones(1);
}

///
/// PhaseInFirstMode
///

PhaseInFirstMode::PhaseInFirstMode(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    double stride_period)
    : stride_period_(stride_period) {
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_feedback.num_positions(),
                                              plant_feedback.num_velocities(),
                                              plant_feedback.num_actuators()))
          .get_index();
  controller_signal_port_ =
      this->DeclareVectorInputPort("ctrl_thread", TimestampedVector<double>(5))
          .get_index();

  this->DeclareVectorOutputPort("phase", BasicVector<double>(1),
                                &PhaseInFirstMode::CalcPhase);
}

void PhaseInFirstMode::CalcPhase(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* init_phase_output) const {
  // Read in fsm state and lift-off time
  const auto controller_signal_port =
      static_cast<const TimestampedVector<double>*>(
          this->EvalVectorInput(context, controller_signal_port_));
  double lift_off_time = controller_signal_port->get_value()(1);

  // Get time
  // Note that we cannot use context time anymore because we have per-step
  // update in the downstream
  auto current_time = controller_signal_port->get_timestamp();

  double time_in_first_mode = current_time - lift_off_time;

  // Calc phase
  double init_phase = time_in_first_mode / stride_period_;

  // Assign init_phase
  init_phase_output->get_mutable_value() << init_phase;

  if (init_phase >= 1) {
    cout << "WARNING: phase = " << init_phase
         << " (>= 1). There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    cout << "fsm_state = " << controller_signal_port->get_value()(0) << endl;
    cout << "lift_off_time = " << lift_off_time << endl;
    cout << "current_time = " << current_time << endl;
    cout << "time_in_first_mode = " << time_in_first_mode << endl;
    DRAKE_UNREACHABLE();
    init_phase = 1 - 1e-8;
  }
  if (init_phase < 0) {
    cout << "WARNING: phase = " << init_phase
         << " (< 0). There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    cout << "fsm_state = " << controller_signal_port->get_value()(0) << endl;
    cout << "lift_off_time = " << lift_off_time << endl;
    cout << "current_time = " << current_time << endl;
    cout << "time_in_first_mode = " << time_in_first_mode << endl;
    DRAKE_UNREACHABLE();
    init_phase = 0;
  }
}

///
/// PlannerFinalPosition
///

// Constructor for global position tracking
PlannerFinalPosition::PlannerFinalPosition(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const Eigen::VectorXd& global_target_pos)
    : PlannerFinalPosition(plant_feedback, 0) {
  global_target_pos_ = global_target_pos;
}

// Constructor for constant step length
PlannerFinalPosition::PlannerFinalPosition(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const Eigen::VectorXd& const_step_length, int n_step)
    : PlannerFinalPosition(plant_feedback, 1) {
  const_step_length_ = const_step_length;
  n_step_ = n_step;
}

// Constructor for radio speed command
PlannerFinalPosition::PlannerFinalPosition(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    double stride_period, int n_step)
    : PlannerFinalPosition(plant_feedback, 2) {
  stride_period_ = stride_period;
  n_step_ = n_step;

  controller_signal_port_ =
      this->DeclareVectorInputPort("ctrl_thread", TimestampedVector<double>(5))
          .get_index();
}

// Base constructor
PlannerFinalPosition::PlannerFinalPosition(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    int high_level_command_mode)
    : high_level_command_mode_(high_level_command_mode) {
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_feedback.num_positions(),
                                              plant_feedback.num_velocities(),
                                              plant_feedback.num_actuators()))
          .get_index();
  phase_port_ =
      this->DeclareVectorInputPort("phase", BasicVector<double>(1)).get_index();

  this->DeclareVectorOutputPort("final_pos", BasicVector<double>(2),
                                &PlannerFinalPosition::CalcFinalPos);
}

void PlannerFinalPosition::CalcFinalPos(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* local_final_pos_output) const {
  if (high_level_command_mode_ == 0) {
    // Read in current robot state
    const VectorXd& current_pelvis_pos_xy =
        static_cast<const OutputVector<double>*>(
            this->EvalVectorInput(context, state_port_))
            ->GetPositions()
            .segment<2>(4);

    Vector2d global_pos_diff = global_target_pos_ - current_pelvis_pos_xy;

    // Rotate the position from global to local
    const VectorXd& quat = static_cast<const OutputVector<double>*>(
                               this->EvalVectorInput(context, state_port_))
                               ->GetPositions()
                               .head<4>();
    Vector3d pelvis_x = Quaterniond(quat(0), quat(1), quat(2), quat(3))
                            .toRotationMatrix()
                            .col(0);
    double yaw = atan2(pelvis_x(1), pelvis_x(0));
    Eigen::Rotation2D<double> rot(-yaw);

    // Assign local_final_pos
    local_final_pos_output->get_mutable_value()
        << rot.toRotationMatrix() * global_pos_diff;

    /*cout << "current_pelvis_pos_xy = " << current_pelvis_pos_xy << endl;
    cout << "pelvis_x = " << pelvis_x.transpose() << endl;
    cout << "yaw = " << yaw << endl;
    cout << "global_pos_diff = " << global_pos_diff.transpose() << endl;
    cout << "rot = \n" << rot.toRotationMatrix() << endl;
    cout << "rot * global_pos_diff = " << (rot.toRotationMatrix() *
    global_pos_diff).transpose()
         << endl;*/

  } else if (high_level_command_mode_ == 1) {
    double init_phase =
        this->EvalVectorInput(context, phase_port_)->get_value()(0);
    Vector2d local_pos_diff = const_step_length_ * (n_step_ - init_phase);

    // Assign local_final_pos
    local_final_pos_output->get_mutable_value() << local_pos_diff;

  } else if (high_level_command_mode_ == 2) {
    auto des_vel = this->EvalVectorInput(context, controller_signal_port_)
                       ->get_value()
                       .segment<2>(3);

    double init_phase =
        this->EvalVectorInput(context, phase_port_)->get_value()(0);
    Vector2d local_pos_diff = des_vel * stride_period_ * (n_step_ - init_phase);

    // Assign local_final_pos
    local_final_pos_output->get_mutable_value() << local_pos_diff;

  } else {
    DRAKE_UNREACHABLE();
  }
}

///
/// InitialStateForPlanner
///

void CalcCOM(const drake::multibody::MultibodyPlant<double>& plant,
             const drake::systems::Context<double>& plant_context,
             const VectorXd& state) {
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d front_contact_point = left_toe.first;
  Vector3d rear_contact_point = left_heel.first;
  Vector3d mid_contact_point = (front_contact_point + rear_contact_point) / 2;
  auto left_toe_mid =
      BodyPoint(mid_contact_point, plant.GetFrameByName("toe_left"));

  // Get CoM position
  VectorXd CoM(3);
  CoM = plant.CalcCenterOfMassPositionInWorld(plant_context);

  // Stance foot position
  VectorXd stance_foot_pos(3);
  plant.CalcPointsPositions(plant_context, left_toe_mid.second,
                            left_toe_mid.first, plant.world_frame(),
                            &stance_foot_pos);
  VectorXd pos_st_to_CoM = CoM - stance_foot_pos;
  /*cout << "CoM = " << CoM.transpose() << endl;
  cout << "stance_foot_pos = " << stance_foot_pos.transpose() << endl;
  cout << "pos_st_to_CoM = " << pos_st_to_CoM.transpose() << endl;*/

  // Get CoM velocity
  Eigen::MatrixXd J_com(3, plant.num_velocities());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      plant_context, JacobianWrtVariable::kV, plant.world_frame(),
      plant.world_frame(), &J_com);
  // Stance foot velocity
  Eigen::MatrixXd J_sf(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      plant_context, JacobianWrtVariable::kV, left_toe_mid.second,
      left_toe_mid.first, plant.world_frame(), plant.world_frame(), &J_sf);
  VectorXd CoM_vel = J_com * state.tail(plant.num_velocities());
  VectorXd stance_foot_vel = J_sf * state.tail(plant.num_velocities());
  VectorXd vel_st_to_CoM = CoM_vel - stance_foot_vel;
  /*cout << "CoM_vel= " << CoM_vel.transpose() << endl;
  cout << "stance_foot_vel= " << stance_foot_vel.transpose() << endl;
  cout << "vel_st_to_CoM= " << vel_st_to_CoM.transpose() << endl;*/
};

// The position of the origin of the toe body frame
void CalcFeetPos(const drake::multibody::MultibodyPlant<double>& plant,
                 const drake::systems::Context<double>& plant_context,
                 const VectorXd& state, Vector3d* left_foot_pos,
                 Vector3d* right_foot_pos) {
  auto left_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_left"));
  auto right_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_right"));

  Vector3d foot_pos = Vector3d::Zero();
  plant.CalcPointsPositions(plant_context, left_toe_origin.second,
                            left_toe_origin.first, plant.world_frame(),
                            &foot_pos);
  (*left_foot_pos) = foot_pos;
  //  cout << "left_foot_pos= " << left_foot_pos->transpose() << endl;
  plant.CalcPointsPositions(plant_context, right_toe_origin.second,
                            right_toe_origin.first, plant.world_frame(),
                            &foot_pos);
  (*right_foot_pos) = foot_pos;
  //  cout << "right_foot_pos= " << right_foot_pos->transpose() << endl;
}

// The velocity of the origin of the toe body frame
void CalcFeetVel(const drake::multibody::MultibodyPlant<double>& plant,
                 const drake::systems::Context<double>& plant_context,
                 const VectorXd& state, Vector3d* left_foot_vel,
                 Vector3d* right_foot_vel) {
  auto left_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_left"));
  auto right_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_right"));

  // Left foot Vel
  Eigen::MatrixXd J(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      plant_context, JacobianWrtVariable::kV, left_toe_origin.second,
      left_toe_origin.first, plant.world_frame(), plant.world_frame(), &J);
  (*left_foot_vel) = J * state.tail(plant.num_velocities());
  //  cout << "left_foot_vel= " << left_foot_vel->transpose() << endl;
  plant.CalcJacobianTranslationalVelocity(
      plant_context, JacobianWrtVariable::kV, right_toe_origin.second,
      right_toe_origin.first, plant.world_frame(), plant.world_frame(), &J);
  (*right_foot_vel) = J * state.tail(plant.num_velocities());
  //  cout << "right_foot_vel= " << right_foot_vel->transpose() << endl;
}

InitialStateForPlanner::InitialStateForPlanner(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const drake::multibody::MultibodyPlant<double>& plant_control, int n_step,
    double stride_period, bool feedback_is_spring_model)
    : nq_(plant_control.num_positions()),
      nv_(plant_control.num_velocities()),
      n_step_(n_step),
      stride_period_(stride_period),
      plant_feedback_(plant_feedback),
      plant_control_(plant_control),
      context_feedback_(plant_feedback.CreateDefaultContext()),
      context_control_(plant_control.CreateDefaultContext()),
      toe_mid_left_(BodyPoint((LeftToeFront(plant_control).first +
                               LeftToeRear(plant_control).first) /
                                  2,
                              plant_control.GetFrameByName("toe_left"))),
      toe_mid_right_(BodyPoint((LeftToeFront(plant_control).first +
                                LeftToeRear(plant_control).first) /
                                   2,
                               plant_control.GetFrameByName("toe_right"))),
      toe_origin_left_(BodyPoint(Vector3d::Zero(),
                                 plant_control_.GetFrameByName("toe_left"))),
      toe_origin_right_(BodyPoint(Vector3d::Zero(),
                                  plant_control_.GetFrameByName("toe_right"))),
      feedback_is_spring_model_(feedback_is_spring_model) {
  // Input/Output Setup
  stance_foot_port_ =
      this->DeclareVectorInputPort("stance_foot", BasicVector<double>(1))
          .get_index();
  state_port_ =
      this->DeclareVectorInputPort(
              "x, u, t", OutputVector<double>(plant_feedback.num_positions(),
                                              plant_feedback.num_velocities(),
                                              plant_feedback.num_actuators()))
          .get_index();
  phase_port_ =
      this->DeclareVectorInputPort("phase", BasicVector<double>(1)).get_index();
  controller_signal_port_ =
      this->DeclareVectorInputPort("ctrl_thread", TimestampedVector<double>(5))
          .get_index();

  adjusted_state_port_ =
      this->DeclareVectorOutputPort(
              "x, u, t",
              OutputVector<double>(plant_control.num_positions(),
                                   plant_control.num_velocities(),
                                   plant_control.num_actuators()),
              &InitialStateForPlanner::CopyAdjustedState)
          .get_index();
  adjustment_port_ =
      this->DeclareVectorOutputPort("quat_xyz_shift", BasicVector<double>(7),
                                    &InitialStateForPlanner::CopyAdjustment)
          .get_index();

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(&InitialStateForPlanner::AdjustState);
  adjusted_state_idx_ = this->DeclareDiscreteState(
      plant_control.num_positions() + plant_control.num_velocities());
  quat_xyz_shift_idx_ = this->DeclareDiscreteState(7);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_control);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_control);

  // Create index maps
  pos_map_w_spr_ = multibody::makeNameToPositionsMap(plant_feedback);
  pos_map_wo_spr_ = multibody::makeNameToPositionsMap(plant_control);
  vel_map_wo_spr_ = multibody::makeNameToVelocitiesMap(plant_control);

  if (feedback_is_spring_model) {
    spring_pos_idx_list_w_spr_ = {
        pos_map_w_spr_.at("knee_joint_left"),
        pos_map_w_spr_.at("knee_joint_right"),
        pos_map_w_spr_.at("ankle_spring_joint_left"),
        pos_map_w_spr_.at("ankle_spring_joint_right")};
  } else {
    spring_pos_idx_list_w_spr_ = {};
  }
  knee_ankle_pos_idx_list_ = {pos_map_wo_spr_.at("knee_left"),
                              pos_map_wo_spr_.at("knee_right"),
                              pos_map_wo_spr_.at("ankle_joint_left"),
                              pos_map_wo_spr_.at("ankle_joint_right")};
  knee_ankle_vel_idx_list_ = {vel_map_wo_spr_.at("knee_leftdot"),
                              vel_map_wo_spr_.at("knee_rightdot"),
                              vel_map_wo_spr_.at("ankle_joint_leftdot"),
                              vel_map_wo_spr_.at("ankle_joint_rightdot")};

  joint_vel_idx_list_left_ = {vel_map_wo_spr_.at("hip_roll_leftdot"),
                              vel_map_wo_spr_.at("hip_yaw_leftdot"),
                              vel_map_wo_spr_.at("hip_pitch_leftdot"),
                              vel_map_wo_spr_.at("knee_leftdot"),
                              vel_map_wo_spr_.at("ankle_joint_leftdot"),
                              vel_map_wo_spr_.at("toe_leftdot")};
  joint_vel_idx_list_right_ = {vel_map_wo_spr_.at("hip_roll_rightdot"),
                               vel_map_wo_spr_.at("hip_yaw_rightdot"),
                               vel_map_wo_spr_.at("hip_pitch_rightdot"),
                               vel_map_wo_spr_.at("knee_rightdot"),
                               vel_map_wo_spr_.at("ankle_joint_rightdot"),
                               vel_map_wo_spr_.at("toe_rightdot")};

  idx_toe_leftdot_ = vel_map_wo_spr_.at("toe_leftdot");
  idx_toe_rightdot_ = vel_map_wo_spr_.at("toe_rightdot");
}

EventStatus InitialStateForPlanner::AdjustState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      static_cast<const OutputVector<double>*>(
          this->EvalVectorInput(context, state_port_));
  VectorXd x_original(map_position_from_spring_to_no_spring_.rows() +
                      map_velocity_from_spring_to_no_spring_.rows());
  x_original << map_position_from_spring_to_no_spring_ *
                    robot_output->GetPositions(),
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();

  // Get phase in the first mode
  double init_phase =
      this->EvalVectorInput(context, phase_port_)->get_value()(0);

  // Get stance foot
  bool is_right_stance =
      (bool)this->EvalVectorInput(context, stance_foot_port_)->get_value()(0);
  bool is_left_stance = !is_right_stance;

  ///
  /// Adjust the knee/ankle joints to match the feet vel between the two models
  ///
  // Use IK to get a state of the model without springs so that the feet
  // velocity match well with the real robot's.
  // The max vel error after the adjustment seems to be always below 0.045 m/s.

  VectorXd x_w_spr = robot_output->GetState();
  plant_feedback_.SetPositions(context_feedback_.get(),
                               x_w_spr.head(plant_feedback_.num_positions()));
  Vector3d left_foot_pos_w_spr;
  Vector3d right_foot_pos_w_spr;
  Vector3d left_foot_vel_w_spr;
  Vector3d right_foot_vel_w_spr;
  CalcFeetPos(plant_feedback_, *context_feedback_, x_w_spr,
              &left_foot_pos_w_spr, &right_foot_pos_w_spr);
  CalcFeetVel(plant_feedback_, *context_feedback_, x_w_spr,
              &left_foot_vel_w_spr, &right_foot_vel_w_spr);

  //  cout << "\n================= Time = " +
  //              std::to_string(
  //                  static_cast<const TimestampedVector<double>*>(
  //                      this->EvalVectorInput(context,
  //                      controller_signal_port_))
  //                      ->get_timestamp()) +
  //              " =======================\n\n";
  //  cout << "  IK || Time of arrival: " << context.get_time() << "
  //  | ";
  VectorXd x_adjusted1 = x_original;
  AdjustKneeAndAnklePos(x_w_spr, left_foot_pos_w_spr, right_foot_pos_w_spr,
                        x_original, &x_adjusted1);
  VectorXd x_adjusted2 = x_adjusted1;
  AdjustKneeAndAnkleVel(left_foot_vel_w_spr, right_foot_vel_w_spr, x_adjusted1,
                        &x_adjusted2);

  ///
  /// Zero out the stance foot velocity
  ///
  if (is_left_stance) {
    left_foot_vel_w_spr = Vector3d::Zero();
  } else {
    right_foot_vel_w_spr = Vector3d::Zero();
  }
  ZeroOutStanceFootVel(is_left_stance, &x_adjusted2);

  ///
  /// Check IK results
  ///
  if (feedback_is_spring_model_) {
    CheckAdjustemnt(x_w_spr, x_original, x_adjusted2, left_foot_pos_w_spr,
                    right_foot_pos_w_spr, left_foot_vel_w_spr,
                    right_foot_vel_w_spr, is_left_stance);
  }

  ///
  /// Heuristic -- zero stance toe joint in the beginning of stance
  ///
  // We need this because the ROM uses mid_contact_point (we want this to be 0),
  // and the toe joint vel is sometimes big in the beginning of stance.
  // See folder: 20210912_bad_solve/2_reproduce_same_type_of_bad_solve
  if (init_phase * stride_period_ < window_) {
    if (is_left_stance) {
      x_adjusted2(nq_ + idx_toe_leftdot_) = 0;
    } else {
      x_adjusted2(nq_ + idx_toe_rightdot_) = 0;
    }
  }

  ///
  /// Shift and rotate Cassie's floating base configuration
  ///
  // Rotate Cassie's floating base configuration to face toward world's x
  // direction and translate the x and y position to the origin

  // Our original plan was to move the touchdown state to the origin.
  //  But right now we move the current state (instead of touchdown state) to
  //  the corresponding x-y position based on init phase and desired traj

  // Rotate Cassie about the world’s z axis such that the x axis of the pelvis
  // frame is in the world’s x-z plane and toward world’s x axis.
  VectorXd x_adjusted3 = x_adjusted2;
  Quaterniond quat(x_adjusted3(0), x_adjusted3(1), x_adjusted3(2),
                   x_adjusted3(3));
  Vector3d pelvis_x = quat.toRotationMatrix().col(0);
  pelvis_x(2) = 0;
  Vector3d world_x(1, 0, 0);
  Quaterniond relative_qaut = Quaterniond::FromTwoVectors(pelvis_x, world_x);
  Quaterniond rotated_quat = relative_qaut * quat;
  x_adjusted3.head(4) << rotated_quat.w(), rotated_quat.vec();
  // cout << "pelvis_Rxyz = \n" << quat.toRotationMatrix() << endl;
  // cout << "rotated_pelvis_Rxyz = \n" << rotated_quat.toRotationMatrix() <<
  // endl;

  // Shift pelvis in x, y direction
  // TODO: WARNING: if you are going to shift it to non-zero position, you need
  // to check if RotateBetweenGlobalAndLocalFrame() still works.
  x_adjusted3(pos_map_wo_spr_.at("base_x")) = 0;
  x_adjusted3(pos_map_wo_spr_.at("base_y")) = 0;

  // Shift pelvis in z direction
  if (prev_is_left_stance_ != is_left_stance) {
    prev_is_left_stance_ = is_left_stance;
    // Get stance foot height in the beginning of the fsm
    plant_control_.SetPositions(context_control_.get(), x_adjusted3.head(nq_));
    stance_foot_height_ = GetStanceFootHeight(
        is_left_stance ? toe_mid_left_ : toe_mid_right_, *context_control_);
  }
  x_adjusted3(pos_map_wo_spr_.at("base_z")) -= stance_foot_height_;

  // Also need to rotate floating base velocities (wrt global frame)
  x_adjusted3.segment<3>(nq_) =
      relative_qaut.toRotationMatrix() * x_adjusted3.segment<3>(nq_);
  x_adjusted3.segment<3>(nq_ + 3) =
      relative_qaut.toRotationMatrix() * x_adjusted3.segment<3>(nq_ + 3);

  ///
  /// Assign
  ///
  discrete_state->get_mutable_vector(adjusted_state_idx_).get_mutable_value()
      << x_adjusted3;
  // The shift from global to local
  // Note: I believe the rotation transformation is from global to local, and
  //  the translation transformation is also from global to local.
  //  It's not a big deal if any of these directions is wrong, because it's just
  //  a change of sign. That is, to update the code, I only need to flip the
  //  sign for the translation at the places where it's used.
  discrete_state->get_mutable_vector(quat_xyz_shift_idx_).get_mutable_value()
      << relative_qaut.w(),
      relative_qaut.vec(),
      x_adjusted3.segment<3>(4) - x_adjusted2.segment<3>(4);

  return EventStatus::Succeeded();
}

void InitialStateForPlanner::CopyAdjustedState(
    const drake::systems::Context<double>& context,
    OutputVector<double>* output) const {
  output->SetState(context.get_discrete_state(adjusted_state_idx_).get_value());
  output->set_timestamp(static_cast<const OutputVector<double>*>(
                            this->EvalVectorInput(context, state_port_))
                            ->get_timestamp());
}

void InitialStateForPlanner::CopyAdjustment(
    const drake::systems::Context<double>& context,
    BasicVector<double>* output) const {
  output->get_mutable_value() =
      context.get_discrete_state(quat_xyz_shift_idx_).get_value();
}

double InitialStateForPlanner::GetStanceFootHeight(
    const BodyPoint& stance_toe_mid,
    const drake::systems::Context<double>& context) const {
  drake::VectorX<double> pt(3);
  this->plant_control_.CalcPointsPositions(context, stance_toe_mid.second,
                                           stance_toe_mid.first,
                                           plant_control_.world_frame(), &pt);
  return pt(2);
}

void InitialStateForPlanner::AdjustKneeAndAnklePos(
    const VectorXd& x_w_spr, const Vector3d& left_foot_pos,
    const Vector3d& right_foot_pos, const VectorXd& x_init_original,
    VectorXd* x_init) const {
  // A rough adjusting. To make it more precise, we will need to change the
  // ankle joint as well.
  // Note that after the adjustment, the four bar linkage constraint is not
  // satisfied anymore.

  /// Assign
  if (feedback_is_spring_model_) {
    // Testing -- disabling this for the controller thread (basically assuming
    // no deflection). In RomOscTrackingData, we map state with springs to state
    // without springs by removing the spring joints.If we don't do this, there
    // will be tracking error in x in OSC, and it will keep moving forward.
    x_init->segment<1>(knee_ankle_pos_idx_list_[0]) +=
        x_w_spr.segment<1>(spring_pos_idx_list_w_spr_[0]);
    x_init->segment<1>(knee_ankle_pos_idx_list_[1]) +=
        x_w_spr.segment<1>(spring_pos_idx_list_w_spr_[1]);

    // We don't need to translate ankle joint because it doesn't affect the
    // forward kinematics
    //  x_init->segment<1>(knee_ankle_pos_idx_list_[2]) -=
    //  x_w_spr.segment<1>(spring_pos_idx_list_w_spr_[2]);
    //  x_init->segment<1>(knee_ankle_pos_idx_list_[3]) -=
    //  x_w_spr.segment<1>(spring_pos_idx_list_w_spr_[3]);
  }
}

void InitialStateForPlanner::AdjustKneeAndAnkleVel(
    const Vector3d& left_foot_vel, const Vector3d& right_foot_vel,
    const VectorXd& x_init_original, VectorXd* x_init) const {
  //  auto start_build = std::chrono::high_resolution_clock::now();

  // Get Jacobian for the feet
  plant_control_.SetPositions(context_control_.get(),
                              x_init_original.head(nq_));
  Eigen::MatrixXd J_lf_wo_spr(3, nv_);
  plant_control_.CalcJacobianTranslationalVelocity(
      *context_control_, JacobianWrtVariable::kV, toe_origin_left_.second,
      toe_origin_left_.first, plant_control_.world_frame(),
      plant_control_.world_frame(), &J_lf_wo_spr);
  Eigen::MatrixXd J_rf_wo_spr(3, nv_);
  plant_control_.CalcJacobianTranslationalVelocity(
      *context_control_, JacobianWrtVariable::kV, toe_origin_right_.second,
      toe_origin_right_.first, plant_control_.world_frame(),
      plant_control_.world_frame(), &J_rf_wo_spr);

  // Construct MP
  MathematicalProgram ik;
  auto knee_left = ik.NewContinuousVariables<1>("knee_left");
  auto knee_right = ik.NewContinuousVariables<1>("knee_right");
  auto ankle_joint_left = ik.NewContinuousVariables<1>("ankle_joint_left");
  auto ankle_joint_right = ik.NewContinuousVariables<1>("ankle_joint_right");

  // Update fill in zero vel for the variables
  VectorXd v = x_init_original.tail(nv_);
  for (int i = 0; i < 4; i++) {
    v(knee_ankle_vel_idx_list_.at(i)) = 0;
  }

  // Get matrix A
  // TODO: if you want to speed up the solve, you can make the A matrix 3x2.
  //  This is probably unnecessary because the current solve time is 1e-5
  //  seconds
  MatrixXd A_lf(3, 4);
  MatrixXd A_rf(3, 4);
  for (int i = 0; i < 4; i++) {
    A_lf.col(i) = J_lf_wo_spr.col(knee_ankle_vel_idx_list_.at(i));
    A_rf.col(i) = J_rf_wo_spr.col(knee_ankle_vel_idx_list_.at(i));
  }
  //    cout << "A_lf = \n" << A_lf << endl;
  //    cout << "A_rf = \n" << A_rf << endl;

  VectorXd b_lf = left_foot_vel - J_lf_wo_spr * v;
  VectorXd b_rf = right_foot_vel - J_rf_wo_spr * v;

  ik.Add2NormSquaredCost(
      A_lf, b_lf, {knee_left, knee_right, ankle_joint_left, ankle_joint_right});
  ik.Add2NormSquaredCost(
      A_rf, b_rf, {knee_left, knee_right, ankle_joint_left, ankle_joint_right});

  // Initial guess
  // Solved analytically. We don't need to give a initial guess
  /*ik.SetInitialGuess(knee_left, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(knee_right, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(ankle_joint_left, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(ankle_joint_right, 0.01 * VectorXd::Random(1));*/

  /// Solve
  //  auto start_solve = std::chrono::high_resolution_clock::now();
  const auto result = qp_solver_.Solve(ik, ik.initial_guess());
  //  auto finish = std::chrono::high_resolution_clock::now();

  /*std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  SolutionResult solution_result = result.get_solution_result();
  cout << "Solver:" << result.get_solver_id().name() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";*/

  /// Assign
  x_init->segment<1>(nq_ + knee_ankle_vel_idx_list_[0]) =
      result.GetSolution(knee_left);
  x_init->segment<1>(nq_ + knee_ankle_vel_idx_list_[1]) =
      result.GetSolution(knee_right);
  x_init->segment<1>(nq_ + knee_ankle_vel_idx_list_[2]) =
      result.GetSolution(ankle_joint_left);
  x_init->segment<1>(nq_ + knee_ankle_vel_idx_list_[3]) =
      result.GetSolution(ankle_joint_right);
}

void InitialStateForPlanner::ZeroOutStanceFootVel(bool is_left_stance,
                                                  VectorXd* x_init) const {
  // auto start_build = std::chrono::high_resolution_clock::now();

  // TODO: Clean up code to have only one bodypoint, one context

  const BodyPoint& toe_origin =
      is_left_stance ? toe_origin_left_ : toe_origin_right_;
  const std::vector<int>& idx_list =
      is_left_stance ? joint_vel_idx_list_left_ : joint_vel_idx_list_right_;

  // Get Jacobian for the feet
  plant_control_.SetPositions(context_control_.get(), x_init->head(nq_));
  Eigen::MatrixXd J(3, nv_);
  plant_control_.CalcJacobianTranslationalVelocity(
      *context_control_, JacobianWrtVariable::kV, toe_origin.second,
      toe_origin.first, plant_control_.world_frame(),
      plant_control_.world_frame(), &J);

  // Construct MP
  MathematicalProgram ik;
  int n_eps = 6;
  auto v_eps = ik.NewContinuousVariables(n_eps, "v_eps");

  // Get matrix A
  MatrixXd A(3, n_eps);
  for (int i = 0; i < n_eps; i++) {
    A.col(i) = J.col(idx_list.at(i));
  }
  // Get vector b
  VectorXd b(3);
  b = -J * x_init->tail(nv_);

  ik.AddLinearEqualityConstraint(A, b, v_eps);
  ik.AddQuadraticErrorCost(MatrixXd::Identity(n_eps, n_eps),
                           VectorXd::Zero(n_eps), v_eps);
  //  ik.Add2NormSquaredCost(A, b, v_eps);

  // Initial guess
  // Solved analytically. We don't need to give a initial guess
  //  ik.SetInitialGuess(v_eps, 0.01 * VectorXd::Random(n_eps));

  /// Solve
  // auto start_solve = std::chrono::high_resolution_clock::now();
  const auto result = qp_solver_.Solve(ik, ik.initial_guess());
  // auto finish = std::chrono::high_resolution_clock::now();

  /*std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  SolutionResult solution_result = result.get_solution_result();
  cout << "Solver:" << result.get_solver_id().name() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";*/

  /// Assign
  auto v_eps_sol = result.GetSolution(v_eps);
  for (int i = 0; i < n_eps; i++) {
    x_init->segment<1>(nq_ + idx_list[i]) += v_eps_sol.segment<1>(i);
  }
}

void InitialStateForPlanner::CheckAdjustemnt(
    const VectorXd& x_w_spr, const VectorXd& x_original,
    const VectorXd& x_adjusted, const Vector3d& left_foot_pos_w_spr,
    const Vector3d& right_foot_pos_w_spr, const Vector3d& left_foot_vel_w_spr,
    const Vector3d& right_foot_vel_w_spr, bool is_left_stance) const {
  // Testing -- check the model difference (springs vs no springs).
  // cout << "=== COM and stance foot ===\n";
  //  cout << "\ncassie without springs:\n";
  //  plant_control_.SetPositions(context_control_.get(), x_original.head(nq_));
  //  CalcCOM(plant_control_, *context_control_, x_original);
  //  cout << "\ncassie with springs:\n";
  //  plant_feedback_.SetPositions(context_feedback_.get(),
  //                               x_w_spr.head(plant_feedback_.num_positions()));
  //  CalcCOM(plant_feedback_, *context_feedback_, x_w_spr);
  //  cout << "\ncassie without springs (after adjustment):\n";
  //  plant_control_.SetPositions(context_control_.get(), x_adjusted.head(nq_));
  //  CalcCOM(plant_control_, *context_control_, x_adjusted);
  //  cout << "=== states ===\n\n";
  //  cout << "FOM state (without springs) = \n" << x_original << endl;
  //  cout << "FOM state (without springs, adjusted) = \n" << x_adjusted <<
  //  endl; cout << "FOM state (with springs) = \n" << x_w_spr
  //  <<
  //  "\n\n";

  // Comparing positions
  /*Vector3d left_foot_pos_wo_spr_original;
  Vector3d right_foot_pos_wo_spr_original;
  Vector3d left_foot_pos_wo_spr;
  Vector3d right_foot_pos_wo_spr;
  plant_control_.SetPositions(context_control_.get(), x_original.head(nq_));
  CalcFeetPos(plant_control_, *context_control_, x_original,
              &left_foot_pos_wo_spr_original, &right_foot_pos_wo_spr_original);
  plant_control_.SetPositions(context_control_.get(), x_adjusted.head(nq_));
  CalcFeetPos(plant_control_, *context_control_, x_adjusted,
              &left_foot_pos_wo_spr, &right_foot_pos_wo_spr);
  double left_pos_error_original =
      (left_foot_pos_w_spr - left_foot_pos_wo_spr_original).norm();
  double left_pos_error_improved =
      (left_foot_pos_w_spr - left_foot_pos_wo_spr).norm();
  double right_pos_error_original =
      (right_foot_pos_w_spr - right_foot_pos_wo_spr_original).norm();
  double right_pos_error_improved =
      (right_foot_pos_w_spr - right_foot_pos_wo_spr).norm();
  bool left_pos_did_not_improve =
      left_pos_error_improved > left_pos_error_original;
  bool right_pos_did_not_improve =
      right_pos_error_improved > right_pos_error_original;
  bool left_pos_error_still_too_large = left_pos_error_improved > 0.01;
  bool right_pos_error_still_too_large = right_pos_error_improved > 0.01;
  //  if (true) {
  if (left_pos_did_not_improve || right_pos_did_not_improve ||
      left_pos_error_still_too_large || right_pos_error_still_too_large) {
    cout << "\n=== Feet pos differences ===\n";
    cout << "left_pos_did_not_improve?" << left_pos_did_not_improve << endl;
    cout << "right_pos_did_not_improve?" << right_pos_did_not_improve << endl;
    cout << "left_pos_error_still_too_large?" << left_pos_error_still_too_large
         << endl;
    cout << "right_pos_error_still_too_large?"
         << right_pos_error_still_too_large << endl;
    cout << "left_foot_pos_w_spr = " << left_foot_pos_w_spr.transpose() << endl;
    cout << "right_foot_pos_w_spr = " << right_foot_pos_w_spr.transpose()
         << endl;
    cout << "left_foot_pos_wo_spr_original = "
         << left_foot_pos_wo_spr_original.transpose() << endl;
    cout << "right_foot_pos_wo_spr_original = "
         << right_foot_pos_wo_spr_original.transpose() << endl;
    cout << "left_foot_pos_wo_spr = " << left_foot_pos_wo_spr.transpose()
         << endl;
    cout << "right_foot_pos_wo_spr = " << right_foot_pos_wo_spr.transpose()
         << endl;
    cout << "before adjustment:\n";
    cout << "  left difference = "
         << (left_foot_pos_w_spr - left_foot_pos_wo_spr_original).transpose()
         << endl;
    cout << "  right difference = "
         << (right_foot_pos_w_spr - right_foot_pos_wo_spr_original).transpose()
         << endl;
    cout << "  norm (left, right) = " << left_pos_error_original << ", "
         << right_pos_error_original << endl;
    cout << "after adjustment:\n";
    cout << "  left difference = "
         << (left_foot_pos_w_spr - left_foot_pos_wo_spr).transpose() << endl;
    cout << "  right difference = "
         << (right_foot_pos_w_spr - right_foot_pos_wo_spr).transpose() << endl;
    cout << "  norm (left, right) = " << left_pos_error_improved << ", "
         << right_pos_error_improved << endl;
    cout << "\n";
    cout << "x_w_spr = " << x_w_spr << endl;
    // TODO: before you put the code on the hardware, you need to disable all
    //  the DRAKE_UNREACHABLE()
    // DRAKE_UNREACHABLE();  // Put a check here for future investigation
  }
  cout << "\n\n";*/

  // Comparing velocities
  Vector3d left_foot_vel_wo_spr_original;
  Vector3d right_foot_vel_wo_spr_original;
  Vector3d left_foot_vel_wo_spr_improved;
  Vector3d right_foot_vel_wo_spr_improved;
  plant_control_.SetPositions(context_control_.get(), x_original.head(nq_));
  CalcFeetVel(plant_control_, *context_control_, x_original,
              &left_foot_vel_wo_spr_original, &right_foot_vel_wo_spr_original);
  plant_control_.SetPositions(context_control_.get(), x_adjusted.head(nq_));
  CalcFeetVel(plant_control_, *context_control_, x_adjusted,
              &left_foot_vel_wo_spr_improved, &right_foot_vel_wo_spr_improved);
  double left_vel_error_original =
      (left_foot_vel_w_spr - left_foot_vel_wo_spr_original).norm();
  double left_vel_error_improved =
      (left_foot_vel_w_spr - left_foot_vel_wo_spr_improved).norm();
  double right_vel_error_original =
      (right_foot_vel_w_spr - right_foot_vel_wo_spr_original).norm();
  double right_vel_error_improved =
      (right_foot_vel_w_spr - right_foot_vel_wo_spr_improved).norm();
  bool left_vel_did_not_improve =
      left_vel_error_improved > left_vel_error_original;
  bool right_vel_did_not_improve =
      right_vel_error_improved > right_vel_error_original;
  bool left_vel_error_still_too_large = left_vel_error_improved > 0.03;
  bool right_vel_error_still_too_large = right_vel_error_improved > 0.03;
  bool stance_foot_vel_too_big =
      (is_left_stance && (left_foot_vel_wo_spr_improved.norm() > 0.2)) ||
      (!is_left_stance && (right_foot_vel_wo_spr_improved.norm() > 0.2));
  //  if (true) {
  if (left_vel_did_not_improve || right_vel_did_not_improve ||
      left_vel_error_still_too_large || right_vel_error_still_too_large ||
      stance_foot_vel_too_big) {
    cout << "\n=== Feet vel differences ===\n";
    cout << "left_vel_did_not_improve?" << left_vel_did_not_improve << endl;
    cout << "right_vel_did_not_improve?" << right_vel_did_not_improve << endl;
    cout << "left_vel_error_still_too_large?" << left_vel_error_still_too_large
         << endl;
    cout << "right_vel_error_still_too_large?"
         << right_vel_error_still_too_large << endl;
    cout << "stance_foot_vel_too_big?" << stance_foot_vel_too_big << endl;
    cout << "left_foot_vel_w_spr = " << left_foot_vel_w_spr.transpose() << endl;
    cout << "right_foot_vel_w_spr = " << right_foot_vel_w_spr.transpose()
         << endl;
    cout << "left_foot_vel_wo_spr_original = "
         << left_foot_vel_wo_spr_original.transpose() << endl;
    cout << "right_foot_vel_wo_spr_original = "
         << right_foot_vel_wo_spr_original.transpose() << endl;
    cout << "left_foot_vel_wo_spr_improved = "
         << left_foot_vel_wo_spr_improved.transpose() << endl;
    cout << "right_foot_vel_wo_spr_improved = "
         << right_foot_vel_wo_spr_improved.transpose() << endl;
    cout << "before adjustment:\n";
    cout << "  left difference = "
         << (left_foot_vel_w_spr - left_foot_vel_wo_spr_original).transpose()
         << endl;
    cout << "  right difference = "
         << (right_foot_vel_w_spr - right_foot_vel_wo_spr_original).transpose()
         << endl;
    cout << "  norm (left, right) = " << left_vel_error_original << ", "
         << right_vel_error_original << endl;
    cout << "after adjustment:\n";
    cout << "  left difference = "
         << (left_foot_vel_w_spr - left_foot_vel_wo_spr_improved).transpose()
         << endl;
    cout << "  right difference = "
         << (right_foot_vel_w_spr - right_foot_vel_wo_spr_improved).transpose()
         << endl;
    cout << "  norm (left, right) = " << left_vel_error_improved << ", "
         << right_vel_error_improved << endl;
    cout << "\n";
    cout << "x_w_spr = " << x_w_spr << endl;
    // TODO: before you put the code on the hardware, you need to disable all
    //  the DRAKE_UNREACHABLE()
    // DRAKE_UNREACHABLE();  // Put a check here for future investigation
  }
  cout << "\n\n";
}

}  // namespace goldilocks_models
}  // namespace dairlib
