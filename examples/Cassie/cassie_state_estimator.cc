#include "examples/Cassie/cassie_state_estimator.h"

#include <math.h>
#include <chrono>
#include <fstream>
#include <utility>

#include "drake/math/orthonormal_basis.h"
#include "drake/solvers/equality_constrained_qp_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace systems {

using std::cout;
using std::endl;

using Eigen::Isometry3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AbstractValue;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using drake::systems::UnrestrictedUpdateEvent;

using multibody::KinematicEvaluatorSet;
using systems::OutputVector;

static const int SPACE_DIM = 3;

CassieStateEstimator::CassieStateEstimator(
    const MultibodyPlant<double>& plant,
    const KinematicEvaluatorSet<double>* fourbar_evaluator,
    const KinematicEvaluatorSet<double>* left_contact_evaluator,
    const KinematicEvaluatorSet<double>* right_contact_evaluator,
    bool test_with_ground_truth_state, bool print_info_to_terminal,
    int hardware_test_mode)
    : plant_(plant),
      fourbar_evaluator_(fourbar_evaluator),
      left_contact_evaluator_(left_contact_evaluator),
      right_contact_evaluator_(right_contact_evaluator),
      world_(plant_.world_frame()),
      is_floating_base_(multibody::isQuaternion(plant)),
      context_(plant_.CreateDefaultContext()),
      toe_frames_({&plant.GetFrameByName("toe_left"),
                   &plant.GetFrameByName("toe_right")}),
      pelvis_frame_(plant.GetFrameByName("pelvis")),
      pelvis_(plant.GetBodyByName("pelvis")),
      rod_on_thighs_({LeftRodOnThigh(plant), RightRodOnThigh(plant)}),
      rod_on_heel_springs_({LeftRodOnHeel(plant), RightRodOnHeel(plant)}),
      rod_length_(kCassieAchillesLength),
      context_gt_(plant_.CreateDefaultContext()),
      test_with_ground_truth_state_(test_with_ground_truth_state),
      print_info_to_terminal_(print_info_to_terminal),
      hardware_test_mode_(hardware_test_mode) {
  DRAKE_DEMAND(&fourbar_evaluator->plant() == &plant);
  DRAKE_DEMAND(&left_contact_evaluator->plant() == &plant);
  DRAKE_DEMAND(&right_contact_evaluator->plant() == &plant);

  n_q_ = plant.num_positions();
  n_v_ = plant.num_velocities();
  n_u_ = plant.num_actuators();

  // Declare input/output ports
  cassie_out_input_port_ = this->DeclareAbstractInputPort(
                                   "cassie_out_t", drake::Value<cassie_out_t>{})
                               .get_index();
  this->DeclareVectorOutputPort(OutputVector<double>(n_q_, n_v_, n_u_),
                                &CassieStateEstimator::CopyStateOut);

  // Initialize index maps
  actuator_idx_map_ = multibody::makeNameToActuatorsMap(plant);
  position_idx_map_ = multibody::makeNameToPositionsMap(plant);
  velocity_idx_map_ = multibody::makeNameToVelocitiesMap(plant);

  if (is_floating_base_) {
    // Middle point between the front and the rear contact points
    front_contact_disp_ = LeftToeFront(plant).first;
    rear_contact_disp_ = LeftToeRear(plant).first;
    mid_contact_disp_ = (front_contact_disp_ + rear_contact_disp_) / 2;

    // Declare input port receiving robot's state (simulation ground truth
    // state)
    if (test_with_ground_truth_state_) {
      state_input_port_ =
          this->DeclareVectorInputPort(OutputVector<double>(n_q_, n_v_, n_u_))
              .get_index();
    }

    // a state which stores previous timestamp
    time_idx_ = DeclareDiscreteState(VectorXd::Zero(1));

    // states related to EKF
    // 1. estimated floating base state (pelvis)
    VectorXd init_floating_base_state = VectorXd::Zero(7 + 6);
    init_floating_base_state(0) = 1;
    init_floating_base_state(6) = 1;
    fb_state_idx_ = DeclareDiscreteState(init_floating_base_state);

    // initialize ekf state mean and covariance
    inekf::RobotState initial_state;
    initial_state.setRotation(Matrix3d::Identity());
    initial_state.setVelocity(Vector3d::Zero());
    initial_state.setPosition(Vector3d::Zero());
    initial_state.setGyroscopeBias(Vector3d::Zero());
    initial_state.setAccelerometerBias(Vector3d::Zero());
    MatrixXd P = MatrixXd::Identity(15, 15);
    P.block<3, 3>(0, 0) = 0.0001 * MatrixXd::Identity(3, 3);  // rotation
    P.block<3, 3>(3, 3) = 0.01 * MatrixXd::Identity(3, 3);    // velocity
    P.block<3, 3>(6, 6) = 0.0001 * MatrixXd::Identity(3, 3);  // position
    P.block<3, 3>(9, 9) = 0.0001 * MatrixXd::Identity(3, 3);  // gyro bias
    P.block<3, 3>(12, 12) = 0.01 * MatrixXd::Identity(3, 3);  // accel bias
    initial_state.setP(P);
    // initialize ekf input noise
    cov_w_ = 0.000289 * Eigen::MatrixXd::Identity(16, 16);
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.002);
    noise_params.setAccelerometerNoise(0.04);
    noise_params.setGyroscopeBiasNoise(0.001);
    noise_params.setAccelerometerBiasNoise(0.001);
    noise_params.setContactNoise(0.05);
    // 2. estimated EKF state (imu frame)
    inekf::InEKF value(initial_state, noise_params);
    ekf_idx_ = DeclareAbstractState(AbstractValue::Make<inekf::InEKF>(value));

    // 3. state for previous imu value
    // Measured accelrometer should point toward positive z when the robot rests
    // on the ground.
    VectorXd init_prev_imu_value = VectorXd::Zero(6);
    init_prev_imu_value << 0, 0, 0, 0, 0, 9.81;
    prev_imu_idx_ = DeclareDiscreteState(init_prev_imu_value);

    // states related to contact estimation
    previous_velocity_idx_ = DeclareDiscreteState(VectorXd::Zero(n_v_, 1));

    filtered_residual_double_idx_ =
        DeclareDiscreteState(VectorXd::Zero(n_v_, 1));
    filtered_residual_left_idx_ = DeclareDiscreteState(VectorXd::Zero(n_v_, 1));
    filtered_residual_right_idx_ =
        DeclareDiscreteState(VectorXd::Zero(n_v_, 1));

    // Contact Estimation - Quadratic Programing
    n_b_ = fourbar_evaluator->count_full();
    n_cl_ = left_contact_evaluator->count_full();
    n_cl_active_ = left_contact_evaluator->count_active();
    n_cr_ = right_contact_evaluator->count_full();
    n_cr_active_ = right_contact_evaluator->count_active();
    // MathematicalProgram
    quadprog_ = std::make_unique<drake::solvers::MathematicalProgram>();
    // Add variables to the optimization program
    ddq_ = quadprog_->NewContinuousVariables(n_v_, "ddq");
    lambda_b_ = quadprog_->NewContinuousVariables(n_b_, "lambda_b");
    lambda_cl_ = quadprog_->NewContinuousVariables(n_cl_, "lambda_cl");
    lambda_cr_ = quadprog_->NewContinuousVariables(n_cr_, "lambda_cr");
    eps_cl_ = quadprog_->NewContinuousVariables(n_cl_active_, "eps_cl");
    eps_cr_ = quadprog_->NewContinuousVariables(n_cr_active_, "eps_cr");
    eps_imu_ = quadprog_->NewContinuousVariables(SPACE_DIM, "eps_imu");
    // Add equality constraints to the optimization program
    fourbar_constraint_ =
        quadprog_
            ->AddLinearEqualityConstraint(MatrixXd::Zero(n_b_, n_v_),
                                          VectorXd::Zero(n_b_), ddq_)
            .evaluator()
            .get();
    left_contact_constraint_ =
        quadprog_
            ->AddLinearEqualityConstraint(
                MatrixXd::Zero(n_cl_active_, n_v_ + n_cl_active_),
                VectorXd::Zero(n_cl_active_), {ddq_, eps_cl_})
            .evaluator()
            .get();
    right_contact_constraint_ =
        quadprog_
            ->AddLinearEqualityConstraint(
                MatrixXd::Zero(n_cr_active_, n_v_ + n_cr_active_),
                VectorXd::Zero(n_cr_active_), {ddq_, eps_cr_})
            .evaluator()
            .get();
    imu_accel_constraint_ = quadprog_
                                ->AddLinearEqualityConstraint(
                                    MatrixXd::Zero(SPACE_DIM, n_v_ + SPACE_DIM),
                                    VectorXd::Zero(SPACE_DIM), {ddq_, eps_imu_})
                                .evaluator()
                                .get();
    // Add costs to the optimization program
    int n_lambda = n_b_ + n_cl_ + n_cr_;
    quadcost_eom_ =
        quadprog_
            ->AddQuadraticCost(MatrixXd::Zero(n_v_ + n_lambda, n_v_ + n_lambda),
                               MatrixXd::Zero(n_v_ + n_lambda, 1),
                               {ddq_, lambda_b_, lambda_cl_, lambda_cr_})
            .evaluator()
            .get();
    quadcost_eps_cl_ =
        quadprog_
            ->AddQuadraticCost(MatrixXd::Zero(n_cl_active_, n_cl_active_),
                               VectorXd::Zero(n_cl_active_), eps_cl_)
            .evaluator()
            .get();
    quadcost_eps_cr_ =
        quadprog_
            ->AddQuadraticCost(MatrixXd::Zero(n_cr_active_, n_cr_active_),
                               VectorXd::Zero(n_cr_active_), eps_cr_)
            .evaluator()
            .get();
    quadcost_eps_imu_ =
        quadprog_
            ->AddQuadraticCost(
                w_soft_constraint_ * MatrixXd::Identity(SPACE_DIM, SPACE_DIM),
                VectorXd::Zero(SPACE_DIM), eps_imu_)
            .evaluator()
            .get();
  }
}

/// solveFourbarLinkage() calculates the angle of heel spring joints given the
/// configuration of Cassie which could be in either fixed-base or floating-
/// base.
///
/// Input:
///  - Generalize position of the robot `q`
///
/// Output:
///  - left heel spring angle `left_heel_spring`
///  - right heel spring angle `right_heel_spring`.
///
/// Assumptions:
///  The heel spring angle in `q` should be set to 0.
///
/// Algorithm:
///  We want to find where the achilles rod and the heel spring intersect.
///  The achilles rod is attached to the thigh with a ball joint, and the heel
///  spring is fixed to the heel. The heel spring (rotational spring) can
///  deflect in only one dimension, meaning it rotates around the spring base
///  where the spring is attached to the heel.
///  Let the ball joint position in the world frame to be r_ball_joint, and the
///  spring base position to be r_heel_spring_base.
///  Let the length of the rod to be rod_length_, and the spring length to be
///  spring_length.
///  We want to find the intersections of a sphere S_r (with origin r_ball_joint
///  and radius rod_length_) and a circle C_s (with origin r_heel_spring_base
///  and radius spring_length). The way we solve it is that we convert the 3D
///  problem into a 2D problem. Let PL_C_s to be the plane where C_s lies.
///  The intersection of S_r and PL_C_s is another circle. Let's call this
///  circle C_r. We can derived C_r's origin by projecting S_r's origin onto
///  PL_C_s, and we can derive C_r's radius by trigonometry.
///  There will be two intersections between C_r and C_s, and only one of the
///  two solutions is physically feasible for Cassie. Let's call this feasible
///  solution p.
///  Given p and the frame of the spring without deflection, we can calculate
///  the magnitude/direction of spring deflection.
///  One minor thing:
///   The connection point of the rod and the spring does not lie on the line
///   where the spring lies. Instead, there is a small offset.
///   We account for this offset by `spring_rest_offset`.
void CassieStateEstimator::solveFourbarLinkage(
    const VectorXd& q, double* left_heel_spring,
    double* right_heel_spring) const {
  // Get the spring length
  double spring_length = rod_on_heel_springs_[0].first.norm();
  // Spring rest angle offset
  double spring_rest_offset =
      atan(rod_on_heel_springs_[0].first(1) / rod_on_heel_springs_[0].first(0));

  plant_.SetPositions(context_.get(), q);

  for (int i = 0; i < 2; i++) {
    // Get thigh pose and heel spring pose
    auto thigh_pose = rod_on_thighs_[i].second.CalcPoseInWorld(*context_);
    const Vector3d& thigh_pos = thigh_pose.translation();
    const auto& thigh_rot_mat = thigh_pose.rotation();

    auto heel_spring_pose =
        rod_on_heel_springs_[i].second.CalcPoseInWorld(*context_);
    const Vector3d& r_heel_spring_base = heel_spring_pose.translation();
    const auto& heel_spring_rot_mat = heel_spring_pose.rotation();

    // Get r_heel_spring_base_to_thigh_ball_joint
    Vector3d r_ball_joint = thigh_pos + thigh_rot_mat * rod_on_thighs_[i].first;
    Vector3d r_heel_spring_base_to_thigh_ball_joint =
        r_ball_joint - r_heel_spring_base;
    Vector3d r_thigh_ball_joint_wrt_heel_spring_base =
        heel_spring_rot_mat.transpose() *
        r_heel_spring_base_to_thigh_ball_joint;

    // Get the projected rod length in the xy plane of heel spring base
    double projected_rod_length =
        sqrt(pow(rod_length_, 2) -
             pow(r_thigh_ball_joint_wrt_heel_spring_base(2), 2));

    // Get the vector of the deflected spring direction
    // Below solves for the intersections of two circles on a plane
    double x_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(0);
    double y_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(1);

    double k = -y_tbj_wrt_hb / x_tbj_wrt_hb;
    double c = (pow(spring_length, 2) - pow(projected_rod_length, 2) +
                pow(x_tbj_wrt_hb, 2) + pow(y_tbj_wrt_hb, 2)) /
               (2 * x_tbj_wrt_hb);

    double y_sol_1 =
        (-k * c + sqrt(pow(k * c, 2) -
                       (pow(k, 2) + 1) * (pow(c, 2) - pow(spring_length, 2)))) /
        (pow(k, 2) + 1);
    double y_sol_2 =
        (-k * c - sqrt(pow(k * c, 2) -
                       (pow(k, 2) + 1) * (pow(c, 2) - pow(spring_length, 2)))) /
        (pow(k, 2) + 1);
    double x_sol_1 = k * y_sol_1 + c;
    double x_sol_2 = k * y_sol_2 + c;

    Vector3d sol_1_wrt_heel_base(x_sol_1, y_sol_1, 0);
    Vector3d sol_2_wrt_heel_base(x_sol_2, y_sol_2, 0);
    Vector3d sol_1_cross_sol_2 = sol_1_wrt_heel_base.cross(sol_2_wrt_heel_base);

    // Pick the only physically feasible solution from the two intersections
    Vector3d r_sol_wrt_heel_base =
        (sol_1_cross_sol_2(2) >= 0) ? sol_2_wrt_heel_base : sol_1_wrt_heel_base;

    // Get the heel spring deflection direction and magnitude
    const Vector3d spring_rest_dir(1, 0, 0);
    double heel_spring_angle =
        acos(r_sol_wrt_heel_base.dot(spring_rest_dir) /
             (r_sol_wrt_heel_base.norm() * spring_rest_dir.norm()));
    Vector3d r_rest_dir_cross_r_hs_to_sol =
        spring_rest_dir.cross(r_sol_wrt_heel_base);
    int spring_deflect_sign = (r_rest_dir_cross_r_hs_to_sol(2) >= 0) ? 1 : -1;
    if (i == 0)
      *left_heel_spring =
          spring_deflect_sign * heel_spring_angle - spring_rest_offset;
    else
      *right_heel_spring =
          spring_deflect_sign * heel_spring_angle - spring_rest_offset;
  }  // end for
}

void CassieStateEstimator::AssignImuValueToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  const double* imu_d = cassie_out.pelvis.vectorNav.linearAcceleration;
  output->SetIMUAccelerationAtIndex(0, imu_d[0]);
  output->SetIMUAccelerationAtIndex(1, imu_d[1]);
  output->SetIMUAccelerationAtIndex(2, imu_d[2]);
}

void CassieStateEstimator::AssignActuationFeedbackToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  // Copy actuators
  output->SetEffortAtIndex(actuator_idx_map_.at("hip_roll_left_motor"),
                           cassie_out.leftLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("hip_yaw_left_motor"),
                           cassie_out.leftLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("hip_pitch_left_motor"),
                           cassie_out.leftLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("knee_left_motor"),
                           cassie_out.leftLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("toe_left_motor"),
                           cassie_out.leftLeg.footDrive.torque);

  output->SetEffortAtIndex(actuator_idx_map_.at("hip_roll_right_motor"),
                           cassie_out.rightLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("hip_yaw_right_motor"),
                           cassie_out.rightLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("hip_pitch_right_motor"),
                           cassie_out.rightLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("knee_right_motor"),
                           cassie_out.rightLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuator_idx_map_.at("toe_right_motor"),
                           cassie_out.rightLeg.footDrive.torque);
}

void CassieStateEstimator::AssignNonFloatingBaseStateToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  // Copy the robot state excluding floating base
  // TODO(yuming): check what cassie_out.leftLeg.footJoint.position is.
  // Similarly, the other leg and the velocity of these joints.
  output->SetPositionAtIndex(position_idx_map_.at("hip_roll_left"),
                             cassie_out.leftLeg.hipRollDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("hip_yaw_left"),
                             cassie_out.leftLeg.hipYawDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("hip_pitch_left"),
                             cassie_out.leftLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("knee_left"),
                             cassie_out.leftLeg.kneeDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("toe_left"),
                             cassie_out.leftLeg.footDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("knee_joint_left"),
                             cassie_out.leftLeg.shinJoint.position);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_joint_left"),
                             cassie_out.leftLeg.tarsusJoint.position);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_spring_joint_left"),
                             0.0);

  output->SetPositionAtIndex(position_idx_map_.at("hip_roll_right"),
                             cassie_out.rightLeg.hipRollDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("hip_yaw_right"),
                             cassie_out.rightLeg.hipYawDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("hip_pitch_right"),
                             cassie_out.rightLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("knee_right"),
                             cassie_out.rightLeg.kneeDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("toe_right"),
                             cassie_out.rightLeg.footDrive.position);
  output->SetPositionAtIndex(position_idx_map_.at("knee_joint_right"),
                             cassie_out.rightLeg.shinJoint.position);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_joint_right"),
                             cassie_out.rightLeg.tarsusJoint.position);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_spring_joint_right"),
                             0.0);

  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_roll_leftdot"),
                             cassie_out.leftLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_yaw_leftdot"),
                             cassie_out.leftLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_pitch_leftdot"),
                             cassie_out.leftLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("knee_leftdot"),
                             cassie_out.leftLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("toe_leftdot"),
                             cassie_out.leftLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("knee_joint_leftdot"),
                             cassie_out.leftLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("ankle_joint_leftdot"),
                             cassie_out.leftLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("ankle_spring_joint_leftdot"),
                             0.0);

  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_roll_rightdot"),
                             cassie_out.rightLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_yaw_rightdot"),
                             cassie_out.rightLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("hip_pitch_rightdot"),
                             cassie_out.rightLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("knee_rightdot"),
                             cassie_out.rightLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("toe_rightdot"),
                             cassie_out.rightLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("knee_joint_rightdot"),
                             cassie_out.rightLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocity_idx_map_.at("ankle_joint_rightdot"),
                             cassie_out.rightLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(
      velocity_idx_map_.at("ankle_spring_joint_rightdot"), 0.0);

  // Solve fourbar linkage for heel spring positions
  double left_heel_spring = 0;
  double right_heel_spring = 0;
  VectorXd q = output->GetMutablePositions();
  if (is_floating_base_) {
    // Floating-base state doesn't affect the spring values
    // We assign the floating base of q in case output's floating base is
    // not initialized.
    for (int i = 0; i < 7; i++) {
      q[i] = 0;
    }
    q[0] = 1;
  }
  solveFourbarLinkage(q, &left_heel_spring, &right_heel_spring);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_spring_joint_left"),
                             left_heel_spring);
  output->SetPositionAtIndex(position_idx_map_.at("ankle_spring_joint_right"),
                             right_heel_spring);
}

void CassieStateEstimator::AssignFloatingBaseStateToOutputVector(
    const VectorXd& est_fb_state, OutputVector<double>* output) const {
  output->SetPositionAtIndex(position_idx_map_.at("base_qw"), est_fb_state(0));
  output->SetPositionAtIndex(position_idx_map_.at("base_qx"), est_fb_state(1));
  output->SetPositionAtIndex(position_idx_map_.at("base_qy"), est_fb_state(2));
  output->SetPositionAtIndex(position_idx_map_.at("base_qz"), est_fb_state(3));
  output->SetPositionAtIndex(position_idx_map_.at("base_x"), est_fb_state(4));
  output->SetPositionAtIndex(position_idx_map_.at("base_y"), est_fb_state(5));
  output->SetPositionAtIndex(position_idx_map_.at("base_z"), est_fb_state(6));

  output->SetVelocityAtIndex(velocity_idx_map_.at("base_wx"), est_fb_state(7));
  output->SetVelocityAtIndex(velocity_idx_map_.at("base_wy"), est_fb_state(8));
  output->SetVelocityAtIndex(velocity_idx_map_.at("base_wz"), est_fb_state(9));
  output->SetVelocityAtIndex(velocity_idx_map_.at("base_vx"), est_fb_state(10));
  output->SetVelocityAtIndex(velocity_idx_map_.at("base_vy"), est_fb_state(11));
  output->SetVelocityAtIndex(velocity_idx_map_.at("base_vz"), est_fb_state(12));
}

/// UpdateContactEstimationCosts() updates the optimal costs of the quadratic
/// programs for contact estimations. There are three QPs in total which assume
/// double support, left support and right support in order.
/// The QP's are solved with the state/input feedback of the robot and the imu
/// linear acceleration.
///
/// Input:
///  - OutputVector `output` containing the state, input and imu acceleration of
///    the robot
///  - time `dt` elapsed between previous iteration and current iteration
///  - discretevalues `discrete_state` to store states related to contact
///    estimation
///
/// In each optimization, the residual of the
///  EoM is calculated based on the assumption of the stance. The assumption of
///  stance is done by imposing a constraint for the acceleration of the stance
///  foot. The acceleration of the pelvis is also constrained to match the imu
///  acceleration. The cost from the three optimizations are compared. In
///  general, the optimization with the least cost is assumed to be the actual
///  stance.
void CassieStateEstimator::UpdateContactEstimationCosts(
    const OutputVector<double>& output, const double& dt,
    DiscreteValues<double>* discrete_state,
    std::vector<double>* optimal_cost) const {
  plant_.SetPositionsAndVelocities(context_.get(), output.GetState());

  // M, C and B matrices
  MatrixXd M(n_v_, n_v_);
  plant_.CalcMassMatrix(*context_, &M);
  VectorXd C(n_v_);
  plant_.CalcBiasTerm(*context_, &C);
  C -= plant_.CalcGravityGeneralizedForces(*context_);
  drake::multibody::MultibodyForces<double> f_app(plant_);
  plant_.CalcForceElementsContribution(*context_, &f_app);
  C -= f_app.generalized_forces();
  C -= plant_.MakeActuationMatrix() * output.GetEfforts();

  // J_b - Jacobian for fourbar linkage
  MatrixXd J_b = fourbar_evaluator_->EvalFullJacobian(*context_);
  VectorXd JdotV_b =
      fourbar_evaluator_->EvalFullJacobianDotTimesV(*context_);

  // J_c{l, r} - contact Jacobians and JdotV
  // l - left; r - right
  MatrixXd J_cl = left_contact_evaluator_->EvalFullJacobian(*context_);
  MatrixXd J_cr = right_contact_evaluator_->EvalFullJacobian(*context_);
  MatrixXd J_cl_active = left_contact_evaluator_->EvalActiveJacobian(*context_);
  MatrixXd J_cr_active =
      right_contact_evaluator_->EvalActiveJacobian(*context_);
  VectorXd JdotV_cl_active =
      left_contact_evaluator_->EvalActiveJacobianDotTimesV(*context_);
  VectorXd JdotV_cr_active =
      right_contact_evaluator_->EvalActiveJacobianDotTimesV(*context_);

  // J_imu - Jacobian of the imu location
  MatrixXd J_imu(SPACE_DIM, n_v_);
  plant_.CalcJacobianTranslationalVelocity(*context_, JacobianWrtVariable::kV,
                                           pelvis_frame_, imu_pos_, world_,
                                           world_, &J_imu);
  VectorXd JdotV_imu = plant_.CalcBiasTranslationalAcceleration(
      *context_, JacobianWrtVariable::kV, pelvis_frame_, imu_pos_, world_,
      world_);

  // Get imu acceleration wrt world
  auto pelvis_pose = plant_.EvalBodyPoseInWorld(*context_, pelvis_);
  const auto& R_WB = pelvis_pose.rotation();
  Vector3d imu_accel_wrt_world = R_WB * output.GetIMUAccelerations() + gravity_;

  // Mathematical program - double contact
  // Equality constraint
  fourbar_constraint_->UpdateCoefficients(J_b, -1 * JdotV_b);

  MatrixXd IMU_coeff(SPACE_DIM, n_v_ + SPACE_DIM);
  IMU_coeff << J_imu, MatrixXd::Identity(SPACE_DIM, SPACE_DIM);
  imu_accel_constraint_->UpdateCoefficients(
      IMU_coeff, -1 * JdotV_imu + imu_accel_wrt_world);

  MatrixXd CL_coeff(n_cl_active_, n_v_ + n_cl_active_);
  CL_coeff << J_cl_active, MatrixXd::Identity(n_cl_active_, n_cl_active_);
  left_contact_constraint_->UpdateCoefficients(CL_coeff, -1 * JdotV_cl_active);

  MatrixXd CR_coeff(n_cr_active_, n_v_ + n_cr_active_);
  CR_coeff << J_cr_active, MatrixXd::Identity(n_cr_active_, n_cr_active_);
  right_contact_constraint_->UpdateCoefficients(CR_coeff, -1 * JdotV_cr_active);

  // Cost
  int A_cols = n_v_ + n_b_ + n_cl_ + n_cr_;
  MatrixXd A_dyn(n_v_, A_cols);
  A_dyn << M, -1 * J_b.transpose(), -1 * J_cl.transpose(),
      -1 * J_cr.transpose();
  VectorXd b_dyn(n_v_);
  b_dyn = -C;

  quadcost_eom_->UpdateCoefficients(
      2 * A_dyn.transpose() * A_dyn +
          eps_cost_ * MatrixXd::Identity(A_cols, A_cols),
      -2 * A_dyn.transpose() * b_dyn);
  quadcost_eps_cl_->UpdateCoefficients(
      w_soft_constraint_ * MatrixXd::Identity(n_cl_active_, n_cl_active_),
      VectorXd::Zero(n_cl_active_));
  quadcost_eps_cr_->UpdateCoefficients(
      w_soft_constraint_ * MatrixXd::Identity(n_cr_active_, n_cr_active_),
      VectorXd::Zero(n_cr_active_));

  // Initial guess
  // EqualityConstrainedQPSolver doesn't depend on the initial guess. There is a
  // closed form solution.

  // Solve the optimization problem
  drake::solvers::EqualityConstrainedQPSolver solver;
  drake::solvers::SolverOptions solver_options;
  solver_options.SetOption(drake::solvers::EqualityConstrainedQPSolver::id(),
                           "FeasibilityTol", 1e-6); // default 1e-12
  drake::solvers::MathematicalProgramResult result_double =
      solver.Solve(*quadprog_, {}, solver_options);

  if (!result_double.is_success()) {
    // If the optimization fails, push infinity into the optimal_cost vector
    optimal_cost->at(0) = std::numeric_limits<double>::infinity();

  } else {
    // Push the optimal cost to the optimal_cost vector
    optimal_cost->at(0) =
        result_double.get_optimal_cost() +
        b_dyn.transpose() * b_dyn;  // the second term is the cosntant term

    // Residual calculation
    // TODO(Nanda): Remove the residual calculation after testing on the real
    // robot
    VectorXd ddq_val = result_double.GetSolution(ddq_);
    VectorXd curr_residual = ddq_val * dt;
    curr_residual -=
        (output.GetVelocities() -
         discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_double =
        discrete_state->get_vector(filtered_residual_double_idx_).get_value();
    filtered_residual_double =
        filtered_residual_double +
        alpha_ * (curr_residual - filtered_residual_double);
    discrete_state->get_mutable_vector(filtered_residual_double_idx_)
            .get_mutable_value()
        << filtered_residual_double;
  }

  // Mathematical program - left contact
  // Equality constraints
  left_contact_constraint_->UpdateCoefficients(CL_coeff, -1 * JdotV_cl_active);
  right_contact_constraint_->UpdateCoefficients(
      MatrixXd::Zero(n_cr_active_, n_v_ + n_cr_active_),
      VectorXd::Zero(n_cr_active_));
  imu_accel_constraint_->UpdateCoefficients(
      IMU_coeff, -1 * JdotV_imu + imu_accel_wrt_world);

  // Cost
  A_dyn << M, -1 * J_b.transpose(), -1 * J_cl.transpose(),
      MatrixXd::Zero(n_v_, n_cr_);

  quadcost_eom_->UpdateCoefficients(
      2 * A_dyn.transpose() * A_dyn +
          eps_cost_ * MatrixXd::Identity(A_cols, A_cols),
      -2 * A_dyn.transpose() * b_dyn);
  quadcost_eps_cl_->UpdateCoefficients(
      w_soft_constraint_ * MatrixXd::Identity(n_cl_active_, n_cl_active_),
      VectorXd::Zero(n_cl_active_));
  quadcost_eps_cr_->UpdateCoefficients(
      MatrixXd::Zero(n_cr_active_, n_cr_active_), VectorXd::Zero(n_cr_active_));

  // Initial guess
  // EqualityConstrainedQPSolver doesn't depend on the initial guess. There is a
  // closed form solution.

  // Solve the optimization problem
  drake::solvers::MathematicalProgramResult result_left =
      solver.Solve(*quadprog_, {}, solver_options);

  if (!result_left.is_success()) {
    // Push infinity into optimal_cost vector if the optimization fails
    optimal_cost->at(1) = std::numeric_limits<double>::infinity();

  } else {
    // Push the optimal cost to the optimal_cost vector
    optimal_cost->at(1) =
        result_left.get_optimal_cost() + b_dyn.transpose() * b_dyn;

    // Residual calculation
    // TODO(Nanda): Remove the residual calculation after testing on the real
    // robot
    VectorXd ddq_val = result_left.GetSolution(ddq_);
    VectorXd curr_residual = ddq_val * dt;
    curr_residual -=
        (output.GetVelocities() -
         discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_left =
        discrete_state->get_vector(filtered_residual_left_idx_).get_value();
    filtered_residual_left = filtered_residual_left +
                             alpha_ * (curr_residual - filtered_residual_left);
    discrete_state->get_mutable_vector(filtered_residual_left_idx_)
            .get_mutable_value()
        << filtered_residual_left;
  }

  // Mathematical program - right contact
  // Equality constraint
  left_contact_constraint_->UpdateCoefficients(
      MatrixXd::Zero(n_cl_active_, n_v_ + n_cl_active_),
      VectorXd::Zero(n_cl_active_));
  right_contact_constraint_->UpdateCoefficients(CR_coeff, -1 * JdotV_cr_active);
  imu_accel_constraint_->UpdateCoefficients(
      IMU_coeff, -1 * JdotV_imu + imu_accel_wrt_world);

  // Cost
  A_dyn << M, -1 * J_b.transpose(), MatrixXd::Zero(n_v_, n_cl_),
      -1 * J_cr.transpose();

  quadcost_eom_->UpdateCoefficients(
      2 * A_dyn.transpose() * A_dyn +
          eps_cost_ * MatrixXd::Identity(A_cols, A_cols),
      -2 * A_dyn.transpose() * b_dyn);
  quadcost_eps_cl_->UpdateCoefficients(
      MatrixXd::Zero(n_cl_active_, n_cl_active_), VectorXd::Zero(n_cl_active_));
  quadcost_eps_cr_->UpdateCoefficients(
      w_soft_constraint_ * MatrixXd::Identity(n_cr_active_, n_cr_active_),
      VectorXd::Zero(n_cr_active_));

  // Initial guess
  // EqualityConstrainedQPSolver doesn't depend on the initial guess. There is a
  // closed form solution.

  // Solve the optimization problem
  drake::solvers::MathematicalProgramResult result_right =
      solver.Solve(*quadprog_, {}, solver_options);

  if (!result_right.is_success()) {
    // If the optimization fails, push infinity to the optimal_cost vector
    optimal_cost->at(2) = std::numeric_limits<double>::infinity();

  } else {
    // Push the optimal cost to optimal_cost vector
    optimal_cost->at(2) =
        result_right.get_optimal_cost() + b_dyn.transpose() * b_dyn;

    // Residual calculation
    // TODO(Nanda): Remove the residual calculation after testing on the real
    // robot
    VectorXd ddq_val = result_right.GetSolution(ddq_);
    VectorXd curr_residual = ddq_val * dt;
    curr_residual -=
        (output.GetVelocities() -
         discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_right =
        discrete_state->get_vector(filtered_residual_right_idx_).get_value();
    filtered_residual_right =
        filtered_residual_right +
        alpha_ * (curr_residual - filtered_residual_right);
    discrete_state->get_mutable_vector(filtered_residual_right_idx_)
            .get_mutable_value()
        << filtered_residual_right;
  }

  // Record previous velocity (used in acceleration residual)
  discrete_state->get_mutable_vector(previous_velocity_idx_).get_mutable_value()
      << output.GetVelocities();
}

/// EstimateContactForEkf(). Conservative estimation.
/// EKF is updated based on the assumption of stance foot being stationary.
/// The estimated state would get very inaccurate if the stance foot is moving.
///
/// Input:
///  - OutputVector `output` containing the state, input and imu acceleration of
///    the robot
/// Output: left contact `left_contact` and right contact `right_contact` that
///  indicate if the corresponding foot is in contact with the ground
///
/// Algorithm:
///  The contact is estimated based on
///   1. compression in the spring
///   2. three optimizations one for each double support, left support and
///   right support
///  If the compression in the left (right) heel/ankle spring is more than the
///  set threshold, the left (right) foot is estimated to be in contact with
///  the ground.
///  During impact, both the legs have some non-zero acceleration and hence the
///  optimal costs will all be high. This case is assumed to be *no* stance.
///
/// Observation:
///  Using both spring and Qp results in better prediction.
///   1. Around impact events, Qp's can sometimes predict correctly, whereas
///      the springs can not.
///   2. If the swing foot is not accelerating and stopping in the air, then QP
///      might predict double support. In this case, spring can predict it
///      better.
///  The above two cases are not comprehensive. E.g. there is another extreme
///  case where during single support phase, the double support cost suddenly
///  became the lowest cost among the three (and at the next time step, the cost
///  went back) Maybe it came from instability of the old walking controller.
///
/// Warning: UpdateContactEstimationCosts() should be called to update the costs
/// before calling EstimateContactForEkf().
void CassieStateEstimator::EstimateContactForEkf(
    const OutputVector<double>& output, const std::vector<double>& optimal_cost,
    int* left_contact, int* right_contact) const {
  // Initialize
  *left_contact = 0;
  *right_contact = 0;

  // Estimate contact based on optimization results
  // The vector optimal_cost has double support, left support and right support
  // costs in order. The corresponding indices are 0, 1, 2.
  // Here we get the index of min of left and right support costs.
  auto min_it =
      std::min_element(std::next(optimal_cost.begin(), 0), optimal_cost.end());
  int min_index = std::distance(optimal_cost.begin(), min_it);

  // If all three costs are high, we believe it's going through impact event (
  // big ground contact point acceleration),
  // and we assume there is no support legs because we don't want moving feet
  // to mess up EKF.
  bool qp_informative = !((optimal_cost.at(0) >= cost_threshold_ekf_) &&
                          (optimal_cost.at(1) >= cost_threshold_ekf_) &&
                          (optimal_cost.at(2) >= cost_threshold_ekf_));
  bool double_contact_qp = (min_index == 0);
  bool left_contact_qp = (min_index == 1);
  bool right_contact_qp = (min_index == 2);

  // Use spring as a necessary guard to determine the contact (that is, in the
  // case where QP says right stance but left spring deflection is not big
  // enough, the algorithm doesn't set the phase to be right support phase)
  // The reason is that the above case might happen when the rear contact
  // point is on the ground but not the front contact point.

  // We say a foot is in contact with the ground if knee and heel spring
  // deflections are *both* over some thresholds. We don't update anything
  // if it's under the threshold.
  const double& left_knee_spring =
      output.GetPositionAtIndex(position_idx_map_.at("knee_joint_left"));
  const double& right_knee_spring =
      output.GetPositionAtIndex(position_idx_map_.at("knee_joint_right"));
  const double& left_heel_spring = output.GetPositionAtIndex(
      position_idx_map_.at("ankle_spring_joint_left"));
  const double& right_heel_spring = output.GetPositionAtIndex(
      position_idx_map_.at("ankle_spring_joint_right"));
  bool left_contact_spring = (left_knee_spring < knee_spring_threshold_ekf_ &&
                              left_heel_spring < heel_spring_threshold_ekf_);
  bool right_contact_spring = (right_knee_spring < knee_spring_threshold_ekf_ &&
                               right_heel_spring < heel_spring_threshold_ekf_);

  // Determine contacts based on both spring deflation and QP cost
  if (qp_informative && (double_contact_qp || left_contact_qp) &&
      left_contact_spring) {
    *left_contact = 1;
  }
  if (qp_informative && (double_contact_qp || right_contact_qp) &&
      right_contact_spring) {
    *right_contact = 1;
  }

  if (print_info_to_terminal_) {
    cout << "optimal_cost[0][1][2], threshold = " << optimal_cost.at(0) << ", "
         << optimal_cost.at(1) << ", " << optimal_cost.at(2) << ", "
         << cost_threshold_ekf_ << endl;

    cout << "left/right knee spring, threshold = " << left_knee_spring << ", "
         << right_knee_spring << ", " << knee_spring_threshold_ekf_ << endl;
    cout << "left/right heel spring, threshold = " << left_heel_spring << ", "
         << right_heel_spring << ", " << heel_spring_threshold_ekf_ << endl;
    cout << "left/right contacts = " << *left_contact << ", " << *right_contact
         << endl;
  }
}

/// EstimateContactForController(). Less conservative.
///
/// Input:
///  - OutputVector `output` containing the state, input and imu acceleration of
///    the robot
/// Output: left contact `left_contact` and right contact `right_contact` that
///  indicate if the corresponding foot is in contact with the ground
///
/// Algorithm:
///  The contact is estimated based on
///   1. compression in the spring
///   2. three optimizations one for each double support, left support and
///   right support
///  If the compression in the left (right) heel/ankle spring is more than the
///  set threshold, the left (right) foot is estimated to be in contact with
///  the ground.
///  During impact, both the legs have some non-zero acceleration and hence the
///  optimal costs will all be high.
///
/// Warning: UpdateContactEstimationCosts() should be called to update the costs
/// before calling EstimateContactForController().
void CassieStateEstimator::EstimateContactForController(
    const OutputVector<double>& output, const std::vector<double>& optimal_cost,
    int* left_contact, int* right_contact) const {
  // Initialize
  *left_contact = 0;
  *right_contact = 0;

  // Estimate contact based on optimization results
  // The vector optimal_cost has double support, left support and right support
  // costs in order. The corresponding indices are 0, 1, 2.
  // Here we get the index of min of left and right support costs.
  auto min_it =
      std::min_element(std::next(optimal_cost.begin(), 0), optimal_cost.end());
  int min_index = std::distance(optimal_cost.begin(), min_it);

  // If all three costs are high, we believe it's going through impact event.
  // Since it's not very informative, we don't set any contact.
  bool qp_informative = !((optimal_cost.at(0) >= cost_threshold_ctrl_) &&
                          (optimal_cost.at(1) >= cost_threshold_ctrl_) &&
                          (optimal_cost.at(2) >= cost_threshold_ctrl_));
  bool double_contact_qp = (min_index == 0);
  bool left_contact_qp = (min_index == 1);
  bool right_contact_qp = (min_index == 2);

  // Contact estimation based on spring deflection information
  // We say a foot is in contact with the ground if either knee OR heel spring
  // deflection is over a threshold. We don't update anything if it's under
  // the threshold.
  const double& left_knee_spring =
      output.GetPositionAtIndex(position_idx_map_.at("knee_joint_left"));
  const double& right_knee_spring =
      output.GetPositionAtIndex(position_idx_map_.at("knee_joint_right"));
  const double& left_heel_spring = output.GetPositionAtIndex(
      position_idx_map_.at("ankle_spring_joint_left"));
  const double& right_heel_spring = output.GetPositionAtIndex(
      position_idx_map_.at("ankle_spring_joint_right"));
  bool left_contact_spring = (left_knee_spring < knee_spring_threshold_ctrl_ ||
                              left_heel_spring < heel_spring_threshold_ctrl_);
  bool right_contact_spring =
      (right_knee_spring < knee_spring_threshold_ctrl_ ||
       right_heel_spring < heel_spring_threshold_ctrl_);

  // Determine contacts based on both spring deflation and QP cost
  if ((qp_informative && (double_contact_qp || left_contact_qp)) ||
      left_contact_spring) {
    *left_contact = 1;
  }
  if ((qp_informative && (double_contact_qp || right_contact_qp)) ||
      right_contact_spring) {
    *right_contact = 1;
  }
}

EventStatus CassieStateEstimator::Update(
    const Context<double>& context,
    drake::systems::State<double>* state) const {
  // Get cassie output
  const auto& cassie_out =
      this->EvalAbstractInput(context, cassie_out_input_port_)
          ->get_value<cassie_out_t>();

  // Get current time and previous time
  double current_time = context.get_time();
  double prev_t =
      state->get_discrete_state().get_vector(time_idx_).get_value()(0);

  double dt = current_time - prev_t;
  if (print_info_to_terminal_) {
    cout << "current_time = " << current_time << endl;
    cout << "dt: " << dt << endl;
  }

  // Get ground truth information
  OutputVector<double> output_gt(n_q_, n_v_, n_u_);
  VectorXd imu_pos_wrt_world_gt(7);
  VectorXd imu_vel_wrt_world_gt(6);
  if (test_with_ground_truth_state_) {
    const OutputVector<double>* cassie_state =
        (OutputVector<double>*)this->EvalVectorInput(context,
                                                     state_input_port_);

    AssignImuValueToOutputVector(cassie_out, &output_gt);
    AssignActuationFeedbackToOutputVector(cassie_out, &output_gt);
    AssignNonFloatingBaseStateToOutputVector(cassie_out, &output_gt);
    VectorXd fb_state_gt(13);
    fb_state_gt.head(7) = cassie_state->GetPositions().head(7);
    fb_state_gt.tail(6) = cassie_state->GetVelocities().head(6);
    AssignFloatingBaseStateToOutputVector(fb_state_gt, &output_gt);

    // We get 0's cassie_state in the beginning because dispatcher_robot_out
    // is not triggered by CASSIE_STATE_SIMULATION message.
    // This wouldn't be an issue when you don't use ground truth state.
    if (output_gt.GetPositions().head(7).norm() == 0) {
      output_gt.SetPositionAtIndex(position_idx_map_.at("base_qw"), 1);
    }

    // Get kinematics cache for ground truth
    plant_.SetPositionsAndVelocities(context_gt_.get(), output_gt.GetState());
    // rotational position
    Eigen::Vector4d quat = output_gt.GetPositions().segment<4>(0);
    imu_pos_wrt_world_gt.head(4) = quat;
    // translational position
    VectorXd pos(3);
    plant_.CalcPointsPositions(*context_gt_, pelvis_frame_, imu_pos_, world_,
                               &pos);
    imu_pos_wrt_world_gt.tail(3) = pos;
    // rotational velocity
    imu_vel_wrt_world_gt.head(3) =
        Quaterniond(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix() *
        output_gt.GetVelocities().head(3);
    // translational velocity
    MatrixXd J(3, n_v_);
    plant_.CalcJacobianTranslationalVelocity(
        *context_gt_, JacobianWrtVariable::kV, pelvis_frame_, imu_pos_, world_,
        world_, &J);
    imu_vel_wrt_world_gt.tail(3) = J * output_gt.GetVelocities();
    if (print_info_to_terminal_) {
      // Print for debugging
      cout << "Ground Truth: " << endl;
      cout << "Positions: " << endl;
      cout << imu_pos_wrt_world_gt.transpose() << endl;
      cout << "Orientation (quaternion) : " << endl;
      cout << quat.transpose() << endl;
      cout << "Velocities: " << endl;
      cout << imu_vel_wrt_world_gt.transpose() << endl;

      // cout << "Leg positions: " << endl;
      // cout << plant_.transformPoints(
      //     cache_gt, Vector3d::Zero(), left_toe_idx_, 0).transpose() <<
      //     endl;
      // cout << plant_.transformPoints(
      //     cache_gt, Vector3d::Zero(), right_toe_idx_, 0).transpose() <<
      //     endl;
      // cout << endl;
    }
  }

  // Extract imu measurement
  VectorXd imu_measurement(6);
  const double* imu_linear_acceleration =
      cassie_out.pelvis.vectorNav.linearAcceleration;
  const double* imu_angular_velocity =
      cassie_out.pelvis.vectorNav.angularVelocity;
  imu_measurement << imu_angular_velocity[0], imu_angular_velocity[1],
      imu_angular_velocity[2], imu_linear_acceleration[0],
      imu_linear_acceleration[1], imu_linear_acceleration[2];
  if (print_info_to_terminal_) {
    // cout << "imu_measurement = " << imu_measurement.transpose() << endl;
  }

  // Perform State Estimation (in several steps)
  // Step 1 - Solve for the unknown joint angle
  // This step is done in AssignNonFloatingBaseStateToOutputVector()

  // Step 2 - EKF (Propagate step)
  auto& ekf = state->get_mutable_abstract_state<inekf::InEKF>(ekf_idx_);
  ekf.Propagate(context.get_discrete_state(prev_imu_idx_).get_value(), dt);

  // Print for debugging
  if (print_info_to_terminal_) {
    cout << "Prediction: " << endl;
    // cout << "Orientation (quaternion) : " << endl;
    // Quaterniond q_prop = Quaterniond(ekf.getState().getRotation());
    // q_prop.normalize();
    // cout << q_prop.w() << " ";
    // cout << q_prop.vec().transpose() << endl;
    cout << "Velocities: " << endl;
    cout << ekf.getState().getVelocity().transpose() << endl;
    cout << "Positions: " << endl;
    cout << ekf.getState().getPosition().transpose() << endl;
    // cout << "X: " << endl;
    // cout << ekf.getState().getX() << endl;
    // cout << "P: " << endl;
    // cout << ekf.getState().getP() << endl;
    if (test_with_ground_truth_state_) {
      cout << "z difference: "
           << ekf.getState().getPosition()[2] - imu_pos_wrt_world_gt[6] << endl;
    }
  }

  // Estimated floating base state (pelvis)
  VectorXd estimated_fb_state(13);
  Vector3d r_imu_to_pelvis_global = ekf.getState().getRotation() * (-imu_pos_);
  // Rotational position
  Quaterniond q(ekf.getState().getRotation());
  q.normalize();
  estimated_fb_state[0] = q.w();
  estimated_fb_state.segment<3>(1) = q.vec();
  // Translational position
  estimated_fb_state.segment<3>(4) =
      ekf.getState().getPosition() + r_imu_to_pelvis_global;
  // Rotational velocity
  Vector3d omega_global =
      ekf.getState().getRotation() * imu_measurement.head(3);
  estimated_fb_state.segment<3>(7) = omega_global;
  // Translational velocity
  estimated_fb_state.tail(3) =
      ekf.getState().getVelocity() + omega_global.cross(r_imu_to_pelvis_global);

  // Estimated robot output
  OutputVector<double> filtered_output(n_q_, n_v_, n_u_);
  AssignImuValueToOutputVector(cassie_out, &filtered_output);
  AssignActuationFeedbackToOutputVector(cassie_out, &filtered_output);
  AssignNonFloatingBaseStateToOutputVector(cassie_out, &filtered_output);
  AssignFloatingBaseStateToOutputVector(estimated_fb_state, &filtered_output);

  // Step 3 - Estimate which foot/feet are in contact with the ground
  // Estimate feet contacts
  int left_contact = 0;
  int right_contact = 0;
  std::vector<double> optimal_cost(3, 0.0);
  if (test_with_ground_truth_state_) {
    UpdateContactEstimationCosts(
        output_gt, dt, &(state->get_mutable_discrete_state()), &optimal_cost);
    EstimateContactForEkf(output_gt, optimal_cost, &left_contact,
                          &right_contact);
  } else {
    UpdateContactEstimationCosts(filtered_output, dt,
                                 &(state->get_mutable_discrete_state()),
                                 &optimal_cost);
    EstimateContactForEkf(filtered_output, optimal_cost, &left_contact,
                          &right_contact);
  }

  // Test mode needed for hardware experiment
  // mode #0 assumes the feet are always on the ground
  // mode #1 assumes the feet are always in the air
  if (hardware_test_mode_ == 0) {
    left_contact = 1;
    right_contact = 1;

    if ((*counter_for_testing_) % 5000 == 0) {
      cout << "pos = " << ekf.getState().getPosition().transpose() << endl;
    }
    *counter_for_testing_ = *counter_for_testing_ + 1;
  } else if (hardware_test_mode_ == 1) {
    left_contact = 0;
    right_contact = 0;
  }

  std::vector<std::pair<int, bool>> contacts;
  contacts.push_back(std::pair<int, bool>(0, left_contact));
  contacts.push_back(std::pair<int, bool>(1, right_contact));
  ekf.setContacts(contacts);

  // Step 4 - EKF (measurement step)
  plant_.SetPositionsAndVelocities(context_.get(), filtered_output.GetState());

  // rotation part of pose and covariance is unused in EKF
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> covariance = MatrixXd::Identity(6, 6);

  if (test_with_ground_truth_state_) {
    // Print for debugging
    if (print_info_to_terminal_) {
      cout << "Rotation differences: " << endl;
      cout << "Rotation matrix from EKF: " << endl;
      cout << ekf.getState().getRotation() << endl;
      cout << "Ground truth rotation: " << endl;
      Quaterniond q_real;
      q_real.w() = output_gt.GetPositions()[0];
      q_real.vec() = output_gt.GetPositions().segment<3>(1);
      MatrixXd R_actual = q_real.toRotationMatrix();
      cout << R_actual << endl;
    }
  }

  inekf::vectorKinematics measured_kinematics;
  Vector3d toe_pos = Vector3d::Zero();
  MatrixXd J = MatrixXd::Zero(3, n_v_);
  for (int i = 0; i < 2; i++) {
    plant_.CalcPointsPositions(*context_, *toe_frames_[i], rear_contact_disp_,
                               pelvis_frame_, &toe_pos);
    pose.block<3, 3>(0, 0) = Matrix3d::Identity();
    pose.block<3, 1>(0, 3) = toe_pos - imu_pos_;

    if (print_info_to_terminal_) {
      // Print for debugging
      // cout << "Pose: " << endl;
      // cout << pose.block<3, 1>(0, 3).transpose() << endl;
    }

    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, *toe_frames_[i], rear_contact_disp_,
        pelvis_frame_, pelvis_frame_, &J);
    MatrixXd J_wrt_joints = J.block(0, 6, 3, 16);
    covariance.block<3, 3>(3, 3) =
        J_wrt_joints * cov_w_ * J_wrt_joints.transpose();
    inekf::Kinematics frame(i, pose, covariance);
    measured_kinematics.push_back(frame);

    if (print_info_to_terminal_) {
      cout << "covariance.block<3, 3>(3, 3) = \n"
           << covariance.block<3, 3>(3, 3) << endl;
    }
  }
  ekf.CorrectKinematics(measured_kinematics);

  if (print_info_to_terminal_) {
    // Print for debugging
    q = Quaterniond(ekf.getState().getRotation()).normalized();
    cout << "Update: " << endl;
    // cout << "Orientation (quaternion) : " << endl;
    // cout << q.w() << " ";
    // cout << q.vec().transpose() << endl;
    cout << "Velocities: " << endl;
    cout << ekf.getState().getVelocity().transpose() << endl;
    cout << "Positions: " << endl;
    cout << ekf.getState().getPosition().transpose() << endl;
    // cout << "X: " << endl;
    // cout << ekf.getState().getX() << endl;
    // cout << "Theta: " << endl;
    // cout << ekf.getState().getTheta() << endl;
    // cout << "P: " << endl;
    // cout << ekf.getState().getP() << endl;
  }
  if (test_with_ground_truth_state_) {
    if (print_info_to_terminal_) {
      cout << "z difference: "
           << ekf.getState().getPosition()[2] - imu_pos_wrt_world_gt[6] << endl;
    }
  }
  if (print_info_to_terminal_) {
    cout << "------------------------------\n";
    cout << endl;
  }

  // Step 5 - Assign values to floating base state (pelvis)
  // We get the angular velocity directly from the IMU without filtering
  // because the magnitude of noise is about 2e-3.
  // Rotational position
  q = Quaterniond(ekf.getState().getRotation()).normalized();
  estimated_fb_state[0] = q.w();
  estimated_fb_state.segment<3>(1) = q.vec();
  // Translational position
  r_imu_to_pelvis_global = ekf.getState().getRotation() * (-imu_pos_);
  estimated_fb_state.segment<3>(4) =
      ekf.getState().getPosition() + r_imu_to_pelvis_global;
  // Rotational velocity
  omega_global = ekf.getState().getRotation() * imu_measurement.head(3);
  estimated_fb_state.segment<3>(7) = omega_global;
  // Translational velocity
  estimated_fb_state.tail(3) =
      ekf.getState().getVelocity() + omega_global.cross(r_imu_to_pelvis_global);
  state->get_mutable_discrete_state()
          .get_mutable_vector(fb_state_idx_)
          .get_mutable_value()
      << estimated_fb_state;

  // Store imu measurement
  state->get_mutable_discrete_state()
          .get_mutable_vector(prev_imu_idx_)
          .get_mutable_value()
      << imu_measurement;
  // Store current time
  state->get_mutable_discrete_state()
          .get_mutable_vector(time_idx_)
          .get_mutable_value()
      << current_time;
  return EventStatus::Succeeded();
}

/// Workhorse state estimation function. Given a `cassie_out_t`, compute the
/// estimated state as an OutputVector
/// Since it needs to map from a struct to a vector, and no assumptions on the
/// ordering of the vector are made, utilizes index maps to make this mapping.
void CassieStateEstimator::CopyStateOut(const Context<double>& context,
                                        OutputVector<double>* output) const {
  const auto& cassie_out =
      this->EvalAbstractInput(context, cassie_out_input_port_)
          ->get_value<cassie_out_t>();

  // Assign values robot output vector
  // Copy imu values and robot state excluding floating base
  AssignImuValueToOutputVector(cassie_out, output);
  AssignActuationFeedbackToOutputVector(cassie_out, output);
  AssignNonFloatingBaseStateToOutputVector(cassie_out, output);
  // Copy the floating base base state
  if (is_floating_base_) {
    AssignFloatingBaseStateToOutputVector(
        context.get_discrete_state(fb_state_idx_).get_value(), output);
    if (print_info_to_terminal_) {
      cout << "Assign floating base state of the pelvis. "
           << context.get_discrete_state(fb_state_idx_).get_value().transpose()
           << endl;
    }
  }
}

void CassieStateEstimator::setPreviousTime(Context<double>* context,
                                           double time) const {
  context->get_mutable_discrete_state(time_idx_).get_mutable_value() << time;
}
void CassieStateEstimator::setInitialPelvisPose(Context<double>* context,
                                                Eigen::Vector4d quat,
                                                Vector3d pelvis_pos) const {
  context->get_mutable_discrete_state(fb_state_idx_).get_mutable_value().head(7)
      << quat, pelvis_pos;

  // Update EKF state
  // The imu's and pelvis's frames share the same rotation.
  Matrix3d imu_rot_mat =
      Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
  Vector3d imu_position = pelvis_pos + imu_rot_mat * imu_pos_;
  auto& filter = context->get_mutable_abstract_state<inekf::InEKF>(ekf_idx_);
  auto state = filter.getState();
  state.setPosition(imu_position);
  state.setRotation(imu_rot_mat);
  filter.setState(state);
  cout << "Set initial IMU position to \n"
       << filter.getState().getPosition().transpose() << endl;
  cout << "Set initial IMU rotation to \n"
       << filter.getState().getRotation() << endl;
}
void CassieStateEstimator::setPreviousImuMeasurement(
    Context<double>* context, const VectorXd& imu_value) const {
  context->get_mutable_discrete_state(prev_imu_idx_).get_mutable_value()
      << imu_value;
}

void CassieStateEstimator::DoCalcNextUpdateTime(
    const Context<double>& context,
    drake::systems::CompositeEventCollection<double>* events,
    double* time) const {
  // Call Leafsystem's DoCalcNextUpdateTime to add events other than our own
  // kTimed events
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);

  // If `context.get_time() < next_message_time_ - eps_`, it means our callback
  // function hasn't been called in the Simulator loop, so we add/declare an
  // event. We set `next_message_time_ - eps_` to be the update time for our
  // callback function, because we want the event triggers before the publish
  // event at time `next_message_time_`.
  // Note that we don't need to worry about declaring multiple events at the
  // same message time, because the events that happen after the next update
  // time of the Simulator are cleared here:
  // https://github.com/RobotLocomotion/drake/blob/5f0ac26e7bf7dc6f86c773c77b2c9926fb67a9d5/systems/framework/diagram.cc#L850
  if (context.get_time() < next_message_time_ - eps_) {
    // Subtract a small epsilon value so this event triggers before the publish
    *time = next_message_time_ - eps_;

    UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
        [this](const Context<double>& c, const UnrestrictedUpdateEvent<double>&,
               drake::systems::State<double>* s) { this->Update(c, s); };

    auto& uu_events = events->get_mutable_unrestricted_update_events();
    uu_events.add_event(std::make_unique<UnrestrictedUpdateEvent<double>>(
        drake::systems::TriggerType::kTimed, callback));
  }
}

}  // namespace systems
}  // namespace dairlib
