#include "examples/goldilocks_models/controller/planner_preprocessing.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "solvers/nonlinear_constraint.h"
#include "systems/controllers/osc/osc_utils.h"

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

using std::cout;
using std::endl;

using Eigen::MatrixXd;
using Eigen::Quaterniond;
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
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(TimestampedVector<double>(2)).get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &CurrentStanceFoot::GetStance);
}

void CurrentStanceFoot::GetStance(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* stance_foot) const {
  // Read in fsm state and lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  int fsm_state = (int)fsm_and_lo_time_port->get_value()(0);

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
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(TimestampedVector<double>(2)).get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &PhaseInFirstMode::CalcPhase);
}

void PhaseInFirstMode::CalcPhase(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* init_phase_output) const {
  // Read in fsm state and lift-off time
  const auto fsm_and_lo_time_port =
      static_cast<const TimestampedVector<double>*>(
          this->EvalVectorInput(context, fsm_and_lo_time_port_));
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);

  // Get time
  // Note that we cannot use context time anymore because we have per-step
  // update in the downstream
  auto current_time = fsm_and_lo_time_port->get_timestamp();

  double time_in_first_mode = current_time - lift_off_time;

  // Calc phase
  double init_phase = time_in_first_mode / stride_period_;

  // Assign init_phase
  init_phase_output->get_mutable_value() << init_phase;

  if (init_phase >= 1) {
    cout << "WARNING: phase = " << init_phase
         << " (>= 1). There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    cout << "fsm_state = " << fsm_and_lo_time_port->get_value()(0) << endl;
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
    cout << "fsm_state = " << fsm_and_lo_time_port->get_value()(0) << endl;
    cout << "lift_off_time = " << lift_off_time << endl;
    cout << "current_time = " << current_time << endl;
    cout << "time_in_first_mode = " << time_in_first_mode << endl;
    DRAKE_UNREACHABLE();
    init_phase = 0;
  }
}

///
/// InitialStateForPlanner
///

void CalcCOM(const drake::multibody::MultibodyPlant<double>& plant,
             const VectorXd& state) {
  auto context = plant.CreateDefaultContext();
  plant.SetPositions(context.get(), state.head(plant.num_positions()));

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d front_contact_point = left_toe.first;
  Vector3d rear_contact_point = left_heel.first;
  Vector3d mid_contact_point = (front_contact_point + rear_contact_point) / 2;
  auto left_toe_mid =
      BodyPoint(mid_contact_point, plant.GetFrameByName("toe_left"));

  // Get CoM position
  VectorXd CoM(3);
  CoM = plant.CalcCenterOfMassPosition(*context);

  // Stance foot position
  VectorXd stance_foot_pos(3);
  plant.CalcPointsPositions(*context, left_toe_mid.second, left_toe_mid.first,
                            plant.world_frame(), &stance_foot_pos);
  VectorXd pos_st_to_CoM = CoM - stance_foot_pos;
  /*cout << "CoM = " << CoM.transpose() << endl;
  cout << "stance_foot_pos = " << stance_foot_pos.transpose() << endl;
  cout << "pos_st_to_CoM = " << pos_st_to_CoM.transpose() << endl;*/

  // Get CoM velocity
  Eigen::MatrixXd J_com(3, plant.num_velocities());
  plant.CalcJacobianCenterOfMassTranslationalVelocity(
      *context, JacobianWrtVariable::kV, plant.world_frame(),
      plant.world_frame(), &J_com);
  // Stance foot velocity
  Eigen::MatrixXd J_sf(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      *context, JacobianWrtVariable::kV, left_toe_mid.second,
      left_toe_mid.first, plant.world_frame(), plant.world_frame(), &J_sf);
  VectorXd CoM_vel = J_com * state.tail(plant.num_velocities());
  VectorXd stance_foot_vel = J_sf * state.tail(plant.num_velocities());
  VectorXd vel_st_to_CoM = CoM_vel - stance_foot_vel;
  /*cout << "CoM_vel= " << CoM_vel.transpose() << endl;
  cout << "stance_foot_vel= " << stance_foot_vel.transpose() << endl;
  cout << "vel_st_to_CoM= " << vel_st_to_CoM.transpose() << endl;*/
};

void CalcFeetPos(const drake::multibody::MultibodyPlant<double>& plant,
                 const VectorXd& state, Vector3d* left_foot_pos,
                 Vector3d* right_foot_pos) {
  // The position of the origin of the toe body frame

  auto context = plant.CreateDefaultContext();
  plant.SetPositions(context.get(), state.head(plant.num_positions()));

  auto left_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_left"));
  auto right_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_right"));

  Vector3d foot_pos = Vector3d::Zero();
  plant.CalcPointsPositions(*context, left_toe_origin.second,
                            left_toe_origin.first, plant.world_frame(),
                            &foot_pos);
  (*left_foot_pos) = foot_pos;
  //  cout << "left_foot_pos= " << left_foot_pos->transpose() << endl;
  plant.CalcPointsPositions(*context, right_toe_origin.second,
                            right_toe_origin.first, plant.world_frame(),
                            &foot_pos);
  (*right_foot_pos) = foot_pos;
  //  cout << "right_foot_pos= " << right_foot_pos->transpose() << endl;
}

void CalcFeetVel(const drake::multibody::MultibodyPlant<double>& plant,
                 const VectorXd& state, Vector3d* left_foot_vel,
                 Vector3d* right_foot_vel) {
  // The velocity of the origin of the toe body frame

  auto context = plant.CreateDefaultContext();
  plant.SetPositions(context.get(), state.head(plant.num_positions()));

  auto left_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_left"));
  auto right_toe_origin =
      BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_right"));

  // Left foot Vel
  Eigen::MatrixXd J(3, plant.num_velocities());
  plant.CalcJacobianTranslationalVelocity(
      *context, JacobianWrtVariable::kV, left_toe_origin.second,
      left_toe_origin.first, plant.world_frame(), plant.world_frame(), &J);
  (*left_foot_vel) = J * state.tail(plant.num_velocities());
  //  cout << "left_foot_vel= " << left_foot_vel->transpose() << endl;
  plant.CalcJacobianTranslationalVelocity(
      *context, JacobianWrtVariable::kV, right_toe_origin.second,
      right_toe_origin.first, plant.world_frame(), plant.world_frame(), &J);
  (*right_foot_vel) = J * state.tail(plant.num_velocities());
  //  cout << "right_foot_vel= " << right_foot_vel->transpose() << endl;
}

class FootVelConstraint : public solvers::NonlinearConstraint<double> {
 public:
  FootVelConstraint(const Vector3d& lf_vel, const Vector3d& rf_vel,
                    const drake::multibody::MultibodyPlant<double>& plant,
                    const VectorXd& v, const MatrixXd& J_lf,
                    const MatrixXd& J_rf, const std::vector<int>& idx_list,
                    const std::string& description = "")
      : NonlinearConstraint<double>(6, 10, VectorXd::Zero(6), VectorXd::Zero(6),
                                    description),
        plant_(plant),
        context_(plant.CreateDefaultContext()),
        lf_vel_(lf_vel),
        rf_vel_(rf_vel),
        v_(v),
        J_lf_(J_lf),
        J_rf_(J_rf),
        idx_list_(idx_list),
        left_toe_origin_(
            BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_left"))),
        right_toe_origin_(
            BodyPoint(Vector3d::Zero(), plant.GetFrameByName("toe_right"))){};

 private:
  void EvaluateConstraint(
      const Eigen::Ref<const drake::VectorX<double>>& knee_ankles_eps_,
      drake::VectorX<double>* output) const override {
    // Update vel
    VectorXd v = v_;
    for (int i = 0; i < 4; i++) {
      v(idx_list_.at(i)) = knee_ankles_eps_(i);
    }

    VectorXd feet_vel(6);
    feet_vel << lf_vel_ - J_lf_ * v + knee_ankles_eps_.segment<3>(4),
        rf_vel_ - J_rf_ * v + knee_ankles_eps_.segment<3>(7);

    // Impose constraint
    *output = feet_vel;
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  // Cannot use reference probably because you use Eigen's block operation to
  // pass y into the constructor
  const Vector3d lf_vel_;
  const Vector3d rf_vel_;
  const VectorXd v_;
  const MatrixXd J_lf_;
  const MatrixXd J_rf_;
  std::vector<int> idx_list_;

  const BodyPoint left_toe_origin_;
  const BodyPoint right_toe_origin_;
};

InitialStateForPlanner::InitialStateForPlanner(
    const drake::multibody::MultibodyPlant<double>& plant_feedback,
    const drake::multibody::MultibodyPlant<double>& plant_controls,
    double final_position_x, int n_step)
    : nq_(plant_controls.num_positions()),
      final_position_x_(final_position_x),
      n_step_(n_step),
      plant_feedback_(plant_feedback),
      plant_controls_(plant_controls) {
  // Input/Output Setup
  stance_foot_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  phase_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  fsm_and_lo_time_port_ =
      this->DeclareVectorInputPort(TimestampedVector<double>(2)).get_index();

  adjusted_state_port_ =
      this->DeclareVectorOutputPort(
              OutputVector<double>(plant_controls.num_positions(),
                                   plant_controls.num_velocities(),
                                   plant_controls.num_actuators()),
              &InitialStateForPlanner::CopyAdjustedState)
          .get_index();
  adjustment_port_ =
      this->DeclareVectorOutputPort(TimestampedVector<double>(7),
                                    &InitialStateForPlanner::CopyAdjustment)
          .get_index();

  // Discrete update
  DeclarePerStepDiscreteUpdateEvent(&InitialStateForPlanner::AdjustState);
  adjusted_state_idx_ = this->DeclareDiscreteState(
      plant_controls.num_positions() + plant_controls.num_velocities());
  quat_xyz_shift_idx_ = this->DeclareDiscreteState(7);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);

  // Create index maps
  pos_map_w_spr_ = multibody::makeNameToPositionsMap(plant_feedback);
  pos_map_wo_spr_ = multibody::makeNameToPositionsMap(plant_controls);
  vel_map_wo_spr_ = multibody::makeNameToVelocitiesMap(plant_controls);
}

EventStatus InitialStateForPlanner::AdjustState(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_original(map_position_from_spring_to_no_spring_.rows() +
                      map_velocity_from_spring_to_no_spring_.rows());
  x_original << map_position_from_spring_to_no_spring_ *
                    robot_output->GetPositions(),
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  // Get stance foot
  bool is_right_stance =
      (bool)this->EvalVectorInput(context, stance_foot_port_)->get_value()(0);
  bool is_left_stance = !is_right_stance;

  //    cout << "init_phase of the state we got = " << init_phase << endl;

  ///
  /// Adjust the knee/ankle joints to match the feet vel between the two models
  ///
  // Use IK to get a state of the model without springs so that the feet
  // velocity match well with the real robot's.
  // The max vel error after the adjustment seems to be always below 0.045 m/s.

  VectorXd x_w_spr = robot_output->GetState();
  Vector3d left_foot_pos_w_spr;
  Vector3d right_foot_pos_w_spr;
  Vector3d left_foot_vel_w_spr;
  Vector3d right_foot_vel_w_spr;
  CalcFeetPos(plant_feedback_, x_w_spr, &left_foot_pos_w_spr,
              &right_foot_pos_w_spr);
  CalcFeetVel(plant_feedback_, x_w_spr, &left_foot_vel_w_spr,
              &right_foot_vel_w_spr);

  cout << "\n================= Time = " +
              std::to_string(
                  static_cast<const TimestampedVector<double>*>(
                      this->EvalVectorInput(context, fsm_and_lo_time_port_))
                      ->get_timestamp()) +
              " =======================\n\n";
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
  /// Testing
  ///
  CheckAdjustemnt(x_w_spr, x_original, x_adjusted2, left_foot_pos_w_spr,
                  right_foot_pos_w_spr, left_foot_vel_w_spr,
                  right_foot_vel_w_spr);

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
  x_adjusted3(pos_map_wo_spr_.at("base_x")) =
      init_phase * final_position_x_ / n_step_;
  x_adjusted3(pos_map_wo_spr_.at("base_y")) = 0;
  // cout << "x(\"base_x\") = " << x_adjusted3(pos_map_wo_spr_.at("base_x")) <<
  // endl; cout << "x(\"base_y\") = " <<
  // x_adjusted3(pos_map_wo_spr_.at("base_y")) << endl;

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
  output->set_timestamp(
      static_cast<const TimestampedVector<double>*>(
          this->EvalVectorInput(context, fsm_and_lo_time_port_))
          ->get_timestamp());
}

void InitialStateForPlanner::CopyAdjustment(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const {
  output->SetDataVector(
      context.get_discrete_state(quat_xyz_shift_idx_).get_value());
  output->set_timestamp(
      static_cast<const TimestampedVector<double>*>(
          this->EvalVectorInput(context, fsm_and_lo_time_port_))
          ->get_timestamp());
}

void InitialStateForPlanner::AdjustKneeAndAnklePos(
    const VectorXd& x_w_spr, const Vector3d& left_foot_pos,
    const Vector3d& right_foot_pos, const VectorXd& x_init_original,
    VectorXd* x_init) const {
  // A rough adjusting. To make it more precise, we will need to change the
  // ankle joint as well.
  // Note that after the adjustment, the four bar linkage constraint is not
  // satisfied anymore.
  std::vector<int> idx_list_spring = {
      pos_map_w_spr_.at("knee_joint_left"),
      pos_map_w_spr_.at("knee_joint_right"),
      pos_map_w_spr_.at("ankle_spring_joint_left"),
      pos_map_w_spr_.at("ankle_spring_joint_right")};

  std::vector<int> idx_list = {pos_map_wo_spr_.at("knee_left"),
                               pos_map_wo_spr_.at("knee_right"),
                               pos_map_wo_spr_.at("ankle_joint_left"),
                               pos_map_wo_spr_.at("ankle_joint_right")};

  /// Assign
  x_init->segment<1>(idx_list[0]) += x_w_spr.segment<1>(idx_list_spring[0]);
  x_init->segment<1>(idx_list[1]) += x_w_spr.segment<1>(idx_list_spring[1]);
  //  x_init->segment<1>(idx_list[2]) -= x_w_spr.segment<1>(idx_list_spring[2]);
  //  x_init->segment<1>(idx_list[3]) -= x_w_spr.segment<1>(idx_list_spring[3]);
}

void InitialStateForPlanner::AdjustKneeAndAnkleVel(
    const Vector3d& left_foot_vel, const Vector3d& right_foot_vel,
    const VectorXd& x_init_original, VectorXd* x_init) const {
  auto start_build = std::chrono::high_resolution_clock::now();

  std::vector<int> idx_list = {vel_map_wo_spr_.at("knee_leftdot"),
                               vel_map_wo_spr_.at("knee_rightdot"),
                               vel_map_wo_spr_.at("ankle_joint_leftdot"),
                               vel_map_wo_spr_.at("ankle_joint_rightdot")};

  // Get Jacobian for the feet
  auto context_wo_spr = plant_controls_.CreateDefaultContext();
  plant_controls_.SetPositions(
      context_wo_spr.get(),
      x_init_original.head(plant_controls_.num_positions()));
  auto left_toe_origin =
      BodyPoint(Vector3d::Zero(), plant_controls_.GetFrameByName("toe_left"));
  auto right_toe_origin =
      BodyPoint(Vector3d::Zero(), plant_controls_.GetFrameByName("toe_right"));
  Eigen::MatrixXd J_lf_wo_spr(3, plant_controls_.num_velocities());
  plant_controls_.CalcJacobianTranslationalVelocity(
      *context_wo_spr, JacobianWrtVariable::kV, left_toe_origin.second,
      left_toe_origin.first, plant_controls_.world_frame(),
      plant_controls_.world_frame(), &J_lf_wo_spr);
  Eigen::MatrixXd J_rf_wo_spr(3, plant_controls_.num_velocities());
  plant_controls_.CalcJacobianTranslationalVelocity(
      *context_wo_spr, JacobianWrtVariable::kV, right_toe_origin.second,
      right_toe_origin.first, plant_controls_.world_frame(),
      plant_controls_.world_frame(), &J_rf_wo_spr);

  // Construct MP
  MathematicalProgram ik;
  auto knee_left = ik.NewContinuousVariables<1>("knee_left");
  auto knee_right = ik.NewContinuousVariables<1>("knee_right");
  auto ankle_joint_left = ik.NewContinuousVariables<1>("ankle_joint_left");
  auto ankle_joint_right = ik.NewContinuousVariables<1>("ankle_joint_right");

  bool constraint_version = false;
  int n_eps = constraint_version ? 6 : 0;
  VectorXDecisionVariable vel_eps =
      constraint_version ? ik.NewContinuousVariables(n_eps, "eps")
                         : ik.NewContinuousVariables(0, "eps");
  if (constraint_version) {
    // 1. Constraint version

    // Foot vel constraint
    auto foot_vel_constraint = std::make_shared<FootVelConstraint>(
        left_foot_vel, right_foot_vel, plant_controls_,
        x_init_original.tail(plant_controls_.num_velocities()), J_lf_wo_spr,
        J_rf_wo_spr, idx_list);
    ik.AddConstraint(
        foot_vel_constraint,
        {knee_left, knee_right, ankle_joint_left, ankle_joint_right, vel_eps});

    // Add cost
    ik.AddQuadraticCost(MatrixXd::Identity(6, 6), VectorXd::Zero(6), vel_eps);
  } else {
    // 2. Cost-only version

    // Update fill in zero vel for the variables
    VectorXd v = x_init_original.tail(plant_controls_.num_velocities());
    for (int i = 0; i < 4; i++) {
      v(idx_list.at(i)) = 0;
    }

    // Get matrix A
    // TODO: if you want to speed up the solve, you can make the A matrix 3x2.
    //  This is probably unnecessary because the current solve time is 1e-5
    //  seconds
    MatrixXd A_lf(3, 4);
    MatrixXd A_rf(3, 4);
    for (int i = 0; i < 4; i++) {
      A_lf.col(i) = J_lf_wo_spr.col(idx_list.at(i));
      A_rf.col(i) = J_rf_wo_spr.col(idx_list.at(i));
    }
    //    cout << "A_lf = \n" << A_lf << endl;
    //    cout << "A_rf = \n" << A_rf << endl;

    VectorXd b_lf = left_foot_vel - J_lf_wo_spr * v;
    VectorXd b_rf = right_foot_vel - J_rf_wo_spr * v;

    ik.AddL2NormCost(
        A_lf, b_lf,
        {knee_left, knee_right, ankle_joint_left, ankle_joint_right});
    ik.AddL2NormCost(
        A_rf, b_rf,
        {knee_left, knee_right, ankle_joint_left, ankle_joint_right});
  }

  // Initial guess
  ik.SetInitialGuess(knee_left, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(knee_right, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(ankle_joint_left, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(ankle_joint_right, 0.01 * VectorXd::Random(1));
  ik.SetInitialGuess(vel_eps, 0.01 * VectorXd::Random(n_eps));

  /// Solve
  //  if (debug_mode_) {
  /*if (true) {
    ik.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                       "../snopt_ik_for_feet.out");
  }
  ik.SetSolverOption(drake::solvers::SnoptSolver::id(),
                     "Major optimality tolerance",
                     ik_feas_tol_);  // target nonlinear constraint violation
  ik.SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Major feasibility tolerance",
      ik_opt_tol_);  // target complementarity gap
                     // TODO: can I move SnoptSolver outside to speed up?
  drake::solvers::SnoptSolver snopt_solver;*/
  auto start_solve = std::chrono::high_resolution_clock::now();
  //  const auto result = snopt_solver.Solve(ik, ik.initial_guess());
  const auto result = qp_solver_.Solve(ik, ik.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  SolutionResult solution_result = result.get_solution_result();
  cout << "Solver:" << result.get_solver_id().name() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  /// Get solution
  /*const auto q_sol = result.GetSolution(ik.q());
  VectorXd q_sol_normd(nq_);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(nq_ - 4);*/

  /*cout << "knee_left= " << result.GetSolution(knee_left) << endl;
  cout << "knee_right= " << result.GetSolution(knee_right) << endl;
  cout << "ankle_joint_left= " << result.GetSolution(ankle_joint_left) << endl;
  cout << "ankle_joint_right= " << result.GetSolution(ankle_joint_right)
       << endl;
  cout << "vel_eps= " << result.GetSolution(vel_eps) << endl;*/

  /// Assign
  x_init->segment<1>(plant_controls_.num_positions() + idx_list[0]) =
      result.GetSolution(knee_left);
  x_init->segment<1>(plant_controls_.num_positions() + idx_list[1]) =
      result.GetSolution(knee_right);
  x_init->segment<1>(plant_controls_.num_positions() + idx_list[2]) =
      result.GetSolution(ankle_joint_left);
  x_init->segment<1>(plant_controls_.num_positions() + idx_list[3]) =
      result.GetSolution(ankle_joint_right);
}

void InitialStateForPlanner::ZeroOutStanceFootVel(bool is_left_stance,
                                                  VectorXd* x_init) const {
  auto start_build = std::chrono::high_resolution_clock::now();

  // TODO: Clean up code to have only one bodypoint, one context

  int n_v = plant_controls_.num_velocities();

  VectorXd x_original = *x_init;

  std::string left_or_right = is_left_stance ? "left" : "right";

  BodyPoint toe_origin = BodyPoint(
      Vector3d::Zero(), plant_controls_.GetFrameByName("toe_" + left_or_right));
  // Get Jacobian for the feet
  auto context_wo_spr = plant_controls_.CreateDefaultContext();
  plant_controls_.SetPositions(
      context_wo_spr.get(), x_original.head(plant_controls_.num_positions()));
  Eigen::MatrixXd J(3, plant_controls_.num_velocities());
  plant_controls_.CalcJacobianTranslationalVelocity(
      *context_wo_spr, JacobianWrtVariable::kV, toe_origin.second,
      toe_origin.first, plant_controls_.world_frame(),
      plant_controls_.world_frame(), &J);

  // Construct MP
  MathematicalProgram ik;
  int n_eps = 6;
  auto v_eps = ik.NewContinuousVariables(n_eps, "v_eps");

  std::vector<int> idx_list = {
      vel_map_wo_spr_.at("hip_roll_" + left_or_right + "dot"),
      vel_map_wo_spr_.at("hip_yaw_" + left_or_right + "dot"),
      vel_map_wo_spr_.at("hip_pitch_" + left_or_right + "dot"),
      vel_map_wo_spr_.at("knee_" + left_or_right + "dot"),
      vel_map_wo_spr_.at("ankle_joint_" + left_or_right + "dot"),
      vel_map_wo_spr_.at("toe_" + left_or_right + "dot")};

  // Get matrix A
  MatrixXd A(3, n_eps);
  for (int i = 0; i < n_eps; i++) {
    A.col(i) = J.col(idx_list.at(i));
  }
  // Get vector b
  VectorXd b(3);
  b = -J * x_original.tail(n_v);

  ik.AddLinearEqualityConstraint(A, b, v_eps);
  ik.AddQuadraticErrorCost(MatrixXd::Identity(n_eps, n_eps),
                           VectorXd::Zero(n_eps), v_eps);
  //  ik.AddL2NormCost(A, b, v_eps);

  // Initial guess
  ik.SetInitialGuess(v_eps, 0.01 * VectorXd::Random(n_eps));

  /// Solve
  auto start_solve = std::chrono::high_resolution_clock::now();
  const auto result = qp_solver_.Solve(ik, ik.initial_guess());
  //  const auto result = Solve(ik, ik.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  SolutionResult solution_result = result.get_solution_result();
  /*cout << "Solver:" << result.get_solver_id().name() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";*/

  /// Assign
  auto v_eps_sol = result.GetSolution(v_eps);
  for (int i = 0; i < n_eps; i++) {
    x_init->segment<1>(plant_controls_.num_positions() + idx_list[i]) +=
        v_eps_sol.segment<1>(i);
  }
}

void InitialStateForPlanner::CheckAdjustemnt(
    const VectorXd& x_w_spr, const VectorXd& x_original,
    const VectorXd& x_adjusted2, const Vector3d& left_foot_pos_w_spr,
    const Vector3d& right_foot_pos_w_spr, const Vector3d& left_foot_vel_w_spr,
    const Vector3d& right_foot_vel_w_spr) const {
  // Testing -- check the model difference (springs vs no springs).
  // cout << "=== COM and stance foot ===\n";
  //  cout << "\ncassie without springs:\n";
  CalcCOM(plant_controls_, x_original);
  //  cout << "\ncassie with springs:\n";
  CalcCOM(plant_feedback_, x_w_spr);
  //  cout << "\ncassie without springs (after adjustment):\n";
  CalcCOM(plant_controls_, x_adjusted2);
  //  cout << "=== states ===\n\n";
  //  cout << "FOM state (without springs) = \n" << x_original << endl;
  //  cout << "FOM state (without springs, adjusted) = \n" << x_adjusted2 <<
  //  endl; cout << "FOM state (with springs) = \n" << x_w_spr
  //  <<
  //  "\n\n";

  // Comparing positions
  Vector3d left_foot_pos_wo_spr_original;
  Vector3d right_foot_pos_wo_spr_original;
  Vector3d left_foot_pos_wo_spr;
  Vector3d right_foot_pos_wo_spr;
  CalcFeetPos(plant_controls_, x_original, &left_foot_pos_wo_spr_original,
              &right_foot_pos_wo_spr_original);
  CalcFeetPos(plant_controls_, x_adjusted2, &left_foot_pos_wo_spr,
              &right_foot_pos_wo_spr);
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
  cout << "\n\n";

  // Comparing velocities
  Vector3d left_foot_vel_wo_spr_original;
  Vector3d right_foot_vel_wo_spr_original;
  Vector3d left_foot_vel_wo_spr_improved;
  Vector3d right_foot_vel_wo_spr_improved;
  CalcFeetVel(plant_controls_, x_original, &left_foot_vel_wo_spr_original,
              &right_foot_vel_wo_spr_original);
  CalcFeetVel(plant_controls_, x_adjusted2, &left_foot_vel_wo_spr_improved,
              &right_foot_vel_wo_spr_improved);
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
  bool left_vel_error_still_too_large = left_vel_error_improved > 0.02;
  bool right_vel_error_still_too_large = right_vel_error_improved > 0.02;
  bool stance_foot_vel_too_big = (left_foot_vel_wo_spr_improved.norm() > 0.2) &&
                                 (right_foot_vel_wo_spr_improved.norm() > 0.2);
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
