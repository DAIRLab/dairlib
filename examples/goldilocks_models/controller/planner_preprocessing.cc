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
using drake::systems::BasicVector;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;
using drake::trajectories::PiecewisePolynomial;

using dairlib::systems::OutputVector;

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
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();

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
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();

  this->DeclareVectorOutputPort(BasicVector<double>(1),
                                &PhaseInFirstMode::CalcPhase);
}

void PhaseInFirstMode::CalcPhase(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* init_phase_output) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  // Read in fsm state and lift-off time
  const BasicVector<double>* fsm_and_lo_time_port =
      this->EvalVectorInput(context, fsm_and_lo_time_port_);
  double lift_off_time = fsm_and_lo_time_port->get_value()(1);

  // Get time
  double timestamp = robot_output->get_timestamp();
  auto current_time = static_cast<double>(timestamp);

  double time_in_first_mode = current_time - lift_off_time;

  // Calc phase
  double init_phase = time_in_first_mode / stride_period_;
  if (init_phase >= 1) {
    cout << "WARNING: phase >= 1. There might be a bug somewhere, "
            "since we are using a time-based fsm\n";
    init_phase = 1 - 1e-8;
  }

  // Assign init_phase
  init_phase_output->get_mutable_value() = init_phase * VectorXd::Ones(1);

  //  cout << "time_in_first_mode = " << time_in_first_mode << endl;
  //  cout << "current_time=" << current_time << endl;
  //  cout << "lift_off_time=" << lift_off_time << endl;
  //  cout << "init_phase=" << init_phase << endl;
}

///
/// InitialStateForPlanner
///

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
  state_port_ = this->DeclareVectorInputPort(
                        OutputVector<double>(plant_feedback.num_positions(),
                                             plant_feedback.num_velocities(),
                                             plant_feedback.num_actuators()))
                    .get_index();
  phase_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  this->DeclareVectorOutputPort(
      OutputVector<double>(plant_controls.num_positions(),
                           plant_controls.num_velocities(),
                           plant_controls.num_actuators()),
      &InitialStateForPlanner::CalcState);

  // Initialize the mapping from spring to no spring
  map_position_from_spring_to_no_spring_ =
      systems::controllers::PositionMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);
  map_velocity_from_spring_to_no_spring_ =
      systems::controllers::VelocityMapFromSpringToNoSpring(plant_feedback,
                                                            plant_controls);

  // Create index maps
  pos_map_ = multibody::makeNameToPositionsMap(plant_controls);
  vel_map_ = multibody::makeNameToVelocitiesMap(plant_controls);
}

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
  cout << "CoM = " << CoM.transpose() << endl;
  cout << "stance_foot_pos = " << stance_foot_pos.transpose() << endl;
  cout << "pos_st_to_CoM = " << pos_st_to_CoM.transpose() << endl;

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
  cout << "CoM_vel= " << CoM_vel.transpose() << endl;
  cout << "stance_foot_vel= " << stance_foot_vel.transpose() << endl;
  cout << "vel_st_to_CoM= " << vel_st_to_CoM.transpose() << endl;
};

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
  cout << "left_foot_vel= " << left_foot_vel->transpose() << endl;
  plant.CalcJacobianTranslationalVelocity(
      *context, JacobianWrtVariable::kV, right_toe_origin.second,
      right_toe_origin.first, plant.world_frame(), plant.world_frame(), &J);
  (*right_foot_vel) = J * state.tail(plant.num_velocities());
  cout << "right_foot_vel= " << right_foot_vel->transpose() << endl;
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

void InitialStateForPlanner::CalcState(
    const drake::systems::Context<double>& context,
    OutputVector<double>* output) const {
  // Read in current robot state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x_init_original(map_position_from_spring_to_no_spring_.rows() +
                           map_velocity_from_spring_to_no_spring_.rows());
  x_init_original << map_position_from_spring_to_no_spring_ *
                         robot_output->GetPositions(),
      map_velocity_from_spring_to_no_spring_ * robot_output->GetVelocities();

  // Get phase in the first mode
  const BasicVector<double>* phase_port =
      this->EvalVectorInput(context, phase_port_);
  double init_phase = phase_port->get_value()(0);

  //    cout << "init_phase of the state we got = " << init_phase << endl;

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
  VectorXd x_init = x_init_original;
  Quaterniond quat(x_init(0), x_init(1), x_init(2), x_init(3));
  Vector3d pelvis_x = quat.toRotationMatrix().col(0);
  pelvis_x(2) = 0;
  Vector3d world_x(1, 0, 0);
  Quaterniond relative_qaut = Quaterniond::FromTwoVectors(pelvis_x, world_x);
  Quaterniond rotated_quat = relative_qaut * quat;
  x_init.head(4) << rotated_quat.w(), rotated_quat.vec();
  // cout << "pelvis_Rxyz = \n" << quat.toRotationMatrix() << endl;
  // cout << "rotated_pelvis_Rxyz = \n" << rotated_quat.toRotationMatrix() <<
  // endl;

  // Shift pelvis in x, y direction
  x_init(pos_map_.at("base_x")) = init_phase * final_position_x_ / n_step_;
  x_init(pos_map_.at("base_y")) = 0;
  // cout << "x(\"base_x\") = " << x_init(pos_map_.at("base_x")) << endl;
  // cout << "x(\"base_y\") = " << x_init(pos_map_.at("base_y")) << endl;

  // Also need to rotate floating base velocities (wrt global frame)
  x_init.segment<3>(nq_) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_);
  x_init.segment<3>(nq_ + 3) =
      relative_qaut.toRotationMatrix() * x_init.segment<3>(nq_ + 3);

  ///
  /// Assign
  ///
  output->SetState(x_init);
  output->set_timestamp(robot_output->get_timestamp());

  ///
  /// Testing
  ///

  // Testing -- check the model difference (springs vs no springs).
  cout << "\n================= Time = " +
              std::to_string(robot_output->get_timestamp()) +
              " =======================\n\n";
  cout << "=== COM and stance foot ===\n";
  cout << "\ncassie without springs:\n";
  CalcCOM(plant_controls_, x_init_original);
  cout << "\ncassie with springs:\n";
  CalcCOM(plant_feedback_, robot_output->GetState());
  cout << "=== states ===\n\n";
  cout << "FOM state (without springs) = \n" << x_init_original << endl;
  cout << "FOM state (with springs) = \n" << robot_output->GetState() << "\n\n";
  cout << "\n";

  // Testing -- use IK to get a state of the model without springs so that the
  //  feet velocity match well with the real robot's

  // TODO -- we can probably also have two stages of the IK. THe first one match
  //  the position and the second matches the vel.

  auto start_build = std::chrono::high_resolution_clock::now();

  VectorXd x_w_spr = robot_output->GetState();
  Vector3d left_foot_vel;
  Vector3d right_foot_vel;
  CalcFeetVel(plant_feedback_, x_w_spr, &left_foot_vel, &right_foot_vel);

  drake::multibody::InverseKinematics ik(plant_controls_);
  auto knee_left =
      ik.get_mutable_prog()->NewContinuousVariables<1>("knee_left");
  auto knee_right =
      ik.get_mutable_prog()->NewContinuousVariables<1>("knee_right");
  auto ankle_joint_left =
      ik.get_mutable_prog()->NewContinuousVariables<1>("ankle_joint_left");
  auto ankle_joint_right =
      ik.get_mutable_prog()->NewContinuousVariables<1>("ankle_joint_right");
  auto vel_eps = ik.get_mutable_prog()->NewContinuousVariables<6>("eps");

  // Configuration constraint
  // We try full position bounding box constraint for now
  ik.get_mutable_prog()->AddBoundingBoxConstraint(
      x_init_original.head(plant_controls_.num_positions()),
      x_init_original.head(plant_controls_.num_positions()), ik.q());

  // Four bar linkage constraint (without spring)
  // Skipped because we have configuration constraint

  // Foot vel constraint
  auto context_wo_spr = plant_controls_.CreateDefaultContext();
  plant_controls_.SetPositions(context_wo_spr.get(),
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
  std::vector<int> idx_list = {
      vel_map_.at("knee_leftdot"), vel_map_.at("knee_rightdot"),
      vel_map_.at("ankle_joint_leftdot"), vel_map_.at("ankle_joint_rightdot")};
  auto foot_vel_constraint = std::make_shared<FootVelConstraint>(
      left_foot_vel, right_foot_vel, plant_controls_,
      x_init_original.tail(plant_controls_.num_velocities()), J_lf_wo_spr, J_rf_wo_spr,
      idx_list);
  ik.get_mutable_prog()->AddConstraint(
      foot_vel_constraint,
      {knee_left, knee_right, ankle_joint_left, ankle_joint_right, vel_eps});

  // Add cost
  ik.get_mutable_prog()->AddQuadraticCost(MatrixXd::Identity(6, 6),
                                          VectorXd::Zero(6), vel_eps);
  // Initial guess
  ik.get_mutable_prog()->SetInitialGuess(knee_left, 0.01 * VectorXd::Random(1));
  ik.get_mutable_prog()->SetInitialGuess(knee_right,
                                         0.01 * VectorXd::Random(1));
  ik.get_mutable_prog()->SetInitialGuess(ankle_joint_left,
                                         0.01 * VectorXd::Random(1));
  ik.get_mutable_prog()->SetInitialGuess(ankle_joint_right,
                                         0.01 * VectorXd::Random(1));
  ik.get_mutable_prog()->SetInitialGuess(vel_eps, 0.01 * VectorXd::Random(6));

  /// Solve
  //  if (debug_mode_) {
  if (true) {
    ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(),
                                           "Print file",
                                           "../snopt_ik_for_feet.out");
  }
  ik.get_mutable_prog()->SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Major optimality tolerance",
      ik_feas_tol_);  // target nonlinear constraint violation
  ik.get_mutable_prog()->SetSolverOption(
      drake::solvers::SnoptSolver::id(), "Major feasibility tolerance",
      ik_opt_tol_);  // target complementarity gap
  // TODO: can I move SnoptSolver outside to speed up?
  drake::solvers::SnoptSolver snopt_solver;
  auto start_solve = std::chrono::high_resolution_clock::now();
  const auto result = snopt_solver.Solve(ik.prog(), ik.prog().initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed_build = start_solve - start_build;
  std::chrono::duration<double> elapsed_solve = finish - start_solve;
  cout << "  IK || Time of arrival: " << robot_output->get_timestamp() << " | ";
  cout << "Build time:" << elapsed_build.count() << " | ";
  cout << "Solve time:" << elapsed_solve.count() << " | ";
  SolutionResult solution_result = result.get_solution_result();
  cout << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << "\n";

  /// Get solution
  const auto q_sol = result.GetSolution(ik.q());
  VectorXd q_sol_normd(nq_);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(nq_ - 4);
  cout << "knee_left= " << result.GetSolution(knee_left) << endl;
  cout << "knee_right= " << result.GetSolution(knee_right) << endl;
  cout << "ankle_joint_left= " << result.GetSolution(ankle_joint_left) << endl;
  cout << "ankle_joint_right= " << result.GetSolution(ankle_joint_right)
       << endl;
  cout << "vel_eps= " << result.GetSolution(vel_eps) << endl;

  cout << "\ncassie without springs (after adjustment):\n";
  VectorXd x_init_adjusted = x_init;
  x_init_adjusted.segment<1>(plant_controls_.num_positions() + idx_list[0]) =
      result.GetSolution(knee_left);
  x_init_adjusted.segment<1>(plant_controls_.num_positions() + idx_list[1]) =
      result.GetSolution(knee_right);
  x_init_adjusted.segment<1>(plant_controls_.num_positions() + idx_list[2]) =
      result.GetSolution(ankle_joint_left);
  x_init_adjusted.segment<1>(plant_controls_.num_positions() + idx_list[3]) =
      result.GetSolution(ankle_joint_right);
  CalcCOM(plant_controls_, x_init_adjusted);
  cout << "\n\n";

  output->SetState(x_init_adjusted);
}

}  // namespace goldilocks_models
}  // namespace dairlib
