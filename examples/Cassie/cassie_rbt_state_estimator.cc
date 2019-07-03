#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include <math.h>
#include <fstream>
#include <chrono>
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/math/orthonormal_basis.h"
#include "drake/multibody/rigid_body_plant/contact_resultant_force_calculator.h"

namespace dairlib {
namespace systems {

using std::cout;
using std::endl;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Isometry3d;

using drake::systems::Context;
using drake::systems::EventStatus;
using drake::systems::DiscreteValues;

using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;

using systems::OutputVector;
using multibody::GetBodyIndexFromName;

CassieRbtStateEstimator::CassieRbtStateEstimator(
    const RigidBodyTree<double>& tree, bool is_floating_base) :
    tree_(tree), is_floating_base_(is_floating_base) {
  // Declare input/output ports
  cassie_out_input_port_ = this->DeclareAbstractInputPort("cassie_out_t",
                           drake::Value<cassie_out_t> {}).get_index();
  this->DeclareVectorOutputPort(
      OutputVector<double>(tree.get_num_positions(),
                           tree.get_num_velocities(),
                           tree.get_num_actuators()),
      &CassieRbtStateEstimator::CopyStateOut);

  // Initialize index maps
  actuator_index_map_ = multibody::makeNameToActuatorsMap(tree);
  position_index_map_ = multibody::makeNameToPositionsMap(tree);
  velocity_index_map_ = multibody::makeNameToVelocitiesMap(tree);

  // Initialize body indices
  left_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_left");
  right_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_right");
  left_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_left");
  right_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_right");
  DRAKE_DEMAND(left_thigh_ind_ != -1 && right_thigh_ind_ != -1 &&
               left_heel_spring_ind_ != -1 && right_heel_spring_ind_ != -1);

  if (is_floating_base) {
    // Declare input port receiving robot's state (simulation ground truth state)
    // TODO(yminchen): delete this input port after finishing testing
    state_input_port_ = this->DeclareVectorInputPort(
                          OutputVector<double>(tree_.get_num_positions(),
                              tree_.get_num_velocities(),
                              tree_.get_num_actuators())).get_index();

    // Declare update event for EKF
    DeclarePerStepDiscreteUpdateEvent(&CassieRbtStateEstimator::Update);

    // a state which stores previous timestamp
    time_idx_ = DeclareDiscreteState(VectorXd::Zero(1));

    // states related to EKF
    VectorXd init_floating_base_state = VectorXd::Zero(7 + 6);
    init_floating_base_state(3) = 1;
    state_idx_ = DeclareDiscreteState(
        init_floating_base_state);  // estimated floating base
    ekf_X_idx_ = DeclareDiscreteState(27);  // estimated EKF state

    // states related to contact estimation
    previous_velocity_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));

    filtered_residual_double_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));
    filtered_residual_left_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));
    filtered_residual_right_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));

    ddq_double_init_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));
    ddq_left_init_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));
    ddq_right_init_idx_ = DeclareDiscreteState(
        VectorXd::Zero(tree_.get_num_velocities(), 1));
    lambda_b_double_init_idx_ = DeclareDiscreteState(VectorXd::Zero(2, 1));
    lambda_b_left_init_idx_ = DeclareDiscreteState(VectorXd::Zero(2, 1));
    lambda_b_right_init_idx_ = DeclareDiscreteState(VectorXd::Zero(2, 1));
    lambda_cl_double_init_idx_ = DeclareDiscreteState(VectorXd::Zero(6, 1));
    lambda_cl_left_init_idx_ = DeclareDiscreteState(VectorXd::Zero(6, 1));
    lambda_cr_double_init_idx_ = DeclareDiscreteState(VectorXd::Zero(6, 1));
    lambda_cr_right_init_idx_ = DeclareDiscreteState(VectorXd::Zero(6, 1));

    // Contact Estimation - Quadratic Programing
    // MathematicalProgram
    quadprog_ = std::make_unique<drake::solvers::MathematicalProgram>();
    // Add variables to the optimization program
    double n_v = tree_.get_num_velocities();
    ddq_ = quadprog_->NewContinuousVariables(n_v, "ddq");
    lambda_b_ = quadprog_->NewContinuousVariables(2, "lambda_b");
    lambda_cl_ = quadprog_->NewContinuousVariables(6, "lambda_cl");
    lambda_cr_ = quadprog_->NewContinuousVariables(6, "lambda_cr");
    eps_cl_ = quadprog_->NewContinuousVariables(6, "eps_cl");
    eps_cr_ = quadprog_->NewContinuousVariables(6, "eps_cr");
    eps_imu_ = quadprog_->NewContinuousVariables(3, "eps_imu");
    // Add equality constraints to the optimization program
    fourbar_constraint_ = quadprog_->AddLinearEqualityConstraint(
        MatrixXd::Zero(2, n_v), MatrixXd::Zero(2, 1), ddq_).
        evaluator().get();
    left_contact_constraint_ = quadprog_->AddLinearEqualityConstraint(
        MatrixXd::Zero(6, n_v + 6), MatrixXd::Zero(6, 1), {ddq_, eps_cl_}).
        evaluator().get();
    right_contact_constraint_ = quadprog_->AddLinearEqualityConstraint(
        MatrixXd::Zero(6, n_v + 6), MatrixXd::Zero(6, 1), {ddq_, eps_cr_}).
        evaluator().get();
    imu_accel_constraint_ = quadprog_->AddLinearEqualityConstraint(
        MatrixXd::Zero(3, n_v + 3), MatrixXd::Zero(3, 1), {ddq_, eps_imu_}).
        evaluator().get();
    // Add costs to the optimization program
    quadcost_eom_ = quadprog_->AddQuadraticCost(MatrixXd::Zero(n_v + 14, n_v + 14),
        MatrixXd::Zero(n_v + 14, 1), {ddq_, lambda_b_, lambda_cl_, lambda_cr_}).
        evaluator().get();
    quadcost_eps_cl_ = quadprog_->AddQuadraticCost(MatrixXd::Zero(6, 6),
        MatrixXd::Zero(6, 1), eps_cl_).
        evaluator().get();
    quadcost_eps_cr_ = quadprog_->AddQuadraticCost(MatrixXd::Zero(6, 6),
        MatrixXd::Zero(6, 1), eps_cr_).
        evaluator().get();
    quadcost_eps_imu_ = quadprog_->AddQuadraticCost(MatrixXd::Zero(3, 3),
        MatrixXd::Zero(3, 1), eps_imu_).
        evaluator().get();
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
///  The achilles rod is attched to the thigh with a ball joint, and the heel
///  spring is fixed to the heel. The heel spring (rotational spring) can
///  deflect in only one dimension, meaning it rotates around the spring base
///  where the spring is attched to the heel.
///  Let the ball joint position in the world frame to be r_ball_joint, and the
///  spring base position to be r_heel_spring_base.
///  Let the length of the rod to be rod_length_, and the spring length to be
///  spring_length.
///  We want to find the intersections of a sphere S_r (with origin r_ball_joint
///  and radius rod_length_) and a circle C_s (with origin r_heel_spring_base
///  and radius spring_length). The way we solve it is that we convert the 3D
///  problem into a 2D problem. Let PL_C_s to be the plane where C_s lies.
///  The interection of S_r and PL_C_s is another circle. Let's call this circle
///  C_r. We can derived C_r's origin by projecting S_r's origin onto PL_C_s,
///  and we can derive C_r's radius by trigonometry.
///  There will be two intersections between C_r and C_s, and only one of the
///  two soluations is physically feasible for Cassie. Let's call this feasible
///  solution p.
///  Given p and the frame of the spring without deflection, we can calculate
///  the magnitude/direction of spring deflection.
///  One minor thing:
///   The connection point of the rod and the spring does not lie on the line
///   where the spring lies. Instead, there is a small offset.
///   We account for this offset by `spring_rest_offset`.
void CassieRbtStateEstimator::solveFourbarLinkage(const VectorXd& q,
    double* left_heel_spring, double* right_heel_spring) const {
  // Get the spring length
  double spring_length = rod_on_heel_spring_.norm();
  // Spring rest angle offset
  double spring_rest_offset = atan(
                              rod_on_heel_spring_(1) / rod_on_heel_spring_(0));

  std::vector<Vector3d> rod_on_thigh{rod_on_thigh_left_, rod_on_thigh_right_};
  std::vector<int> thigh_ind{left_thigh_ind_, right_thigh_ind_};
  std::vector<int> heel_spring_ind{left_heel_spring_ind_,
                                   right_heel_spring_ind_};

  KinematicsCache<double> cache = tree_.doKinematics(q);

  for (int i = 0; i < 2; i++) {
    // Get thigh pose and heel spring pose
    const Isometry3d thigh_pose = tree_.CalcBodyPoseInWorldFrame(
                                    cache, tree_.get_body(thigh_ind[i]));
    const Vector3d thigh_pos = thigh_pose.translation();
    const MatrixXd thigh_rot_mat = thigh_pose.linear();

    const Isometry3d heel_spring_pose = tree_.CalcBodyPoseInWorldFrame(cache,
                                        tree_.get_body(heel_spring_ind[i]));
    const Vector3d r_heel_spring_base = heel_spring_pose.translation();
    const MatrixXd heel_spring_rot_mat = heel_spring_pose.linear();

    // Get r_heel_spring_base_to_thigh_ball_joint
    Vector3d r_ball_joint = thigh_pos + thigh_rot_mat * rod_on_thigh[i];
    Vector3d r_heel_spring_base_to_thigh_ball_joint =
        r_ball_joint - r_heel_spring_base;
    Vector3d r_thigh_ball_joint_wrt_heel_spring_base =
        heel_spring_rot_mat.transpose() * r_heel_spring_base_to_thigh_ball_joint;

    // Get the projected rod length in the xy plane of heel spring base
    double projected_rod_length = sqrt(pow(rod_length_, 2) -
        pow(r_thigh_ball_joint_wrt_heel_spring_base(2), 2));

    // Get the vector of the deflected spring direction
    // Below solves for the intersections of two circles on a plane
    double x_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(0);
    double y_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(1);

    double k = -y_tbj_wrt_hb / x_tbj_wrt_hb;
    double c =
        (pow(spring_length, 2) - pow(projected_rod_length, 2) +
         pow(x_tbj_wrt_hb, 2) + pow(y_tbj_wrt_hb, 2)) / (2 * x_tbj_wrt_hb);

    double y_sol_1 =
        (-k * c + sqrt(pow(k * c, 2) - (pow(k, 2) + 1) *
                     (pow(c, 2) - pow(spring_length, 2)))) / (pow(k, 2) + 1);
    double y_sol_2 =
        (-k * c - sqrt(pow(k * c, 2) - (pow(k, 2) + 1) *
                     (pow(c, 2) - pow(spring_length, 2)))) / (pow(k, 2) + 1);
    double x_sol_1 = k * y_sol_1 + c;
    double x_sol_2 = k * y_sol_2 + c;

    Vector3d sol_1_wrt_heel_base(x_sol_1, y_sol_1, 0);
    Vector3d sol_2_wrt_heel_base(x_sol_2, y_sol_2, 0);
    Vector3d sol_1_cross_sol_2 = sol_1_wrt_heel_base.cross(sol_2_wrt_heel_base);

    // Pick the only physically feasible solution from the two intersections
    Vector3d r_sol_wrt_heel_base = (sol_1_cross_sol_2(2) >= 0) ?
                                   sol_2_wrt_heel_base : sol_1_wrt_heel_base;

    // Get the heel spring deflection direction and magnitude
    const Vector3d spring_rest_dir(1, 0, 0);
    double heel_spring_angle =
        acos(r_sol_wrt_heel_base.dot(spring_rest_dir) /
            (r_sol_wrt_heel_base.norm() * spring_rest_dir.norm()));
    Vector3d r_rest_dir_cross_r_hs_to_sol = spring_rest_dir.cross(
        r_sol_wrt_heel_base);
    int spring_deflect_sign = (r_rest_dir_cross_r_hs_to_sol(2) >= 0) ? 1 : -1;
    if (i == 0)
      *left_heel_spring = spring_deflect_sign * heel_spring_angle
                         - spring_rest_offset;
    else
      *right_heel_spring = spring_deflect_sign * heel_spring_angle
                          - spring_rest_offset;
  }  // end for
}


void CassieRbtStateEstimator::AssignImuValueToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  const double* imu_d = cassie_out.pelvis.vectorNav.linearAcceleration;
  output->SetIMUAccelerationAtIndex(0, imu_d[0]);
  output->SetIMUAccelerationAtIndex(1, imu_d[1]);
  output->SetIMUAccelerationAtIndex(2, imu_d[2]);
}

void CassieRbtStateEstimator::AssignActuationFeedbackToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  // Copy actuators
  output->SetEffortAtIndex(actuator_index_map_.at("hip_roll_left_motor"),
                           cassie_out.leftLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("hip_yaw_left_motor"),
                           cassie_out.leftLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("hip_pitch_left_motor"),
                           cassie_out.leftLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("knee_left_motor"),
                           cassie_out.leftLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("toe_left_motor"),
                           cassie_out.leftLeg.footDrive.torque);

  output->SetEffortAtIndex(actuator_index_map_.at("hip_roll_right_motor"),
                           cassie_out.rightLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("hip_yaw_right_motor"),
                           cassie_out.rightLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("hip_pitch_right_motor"),
                           cassie_out.rightLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("knee_right_motor"),
                           cassie_out.rightLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuator_index_map_.at("toe_right_motor"),
                           cassie_out.rightLeg.footDrive.torque);
}

void CassieRbtStateEstimator::AssignNonFloatingBaseStateToOutputVector(
    const cassie_out_t& cassie_out, OutputVector<double>* output) const {
  // Copy the robot state excluding floating base
  // TODO(yuming): check what cassie_out.leftLeg.footJoint.position is.
  // Similarly, the other leg and the velocity of these joints.
  output->SetPositionAtIndex(position_index_map_.at("hip_roll_left"),
                             cassie_out.leftLeg.hipRollDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("hip_yaw_left"),
                             cassie_out.leftLeg.hipYawDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("hip_pitch_left"),
                             cassie_out.leftLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("knee_left"),
                             cassie_out.leftLeg.kneeDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("toe_left"),
                             cassie_out.leftLeg.footDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("knee_joint_left"),
                             cassie_out.leftLeg.shinJoint.position);
  output->SetPositionAtIndex(position_index_map_.at("ankle_joint_left"),
                             cassie_out.leftLeg.tarsusJoint.position);
  output->SetPositionAtIndex(position_index_map_.at("ankle_spring_joint_left"),
                             0.0);

  output->SetPositionAtIndex(position_index_map_.at("hip_roll_right"),
                             cassie_out.rightLeg.hipRollDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("hip_yaw_right"),
                             cassie_out.rightLeg.hipYawDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("hip_pitch_right"),
                             cassie_out.rightLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("knee_right"),
                             cassie_out.rightLeg.kneeDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("toe_right"),
                             cassie_out.rightLeg.footDrive.position);
  output->SetPositionAtIndex(position_index_map_.at("knee_joint_right"),
                             cassie_out.rightLeg.shinJoint.position);
  output->SetPositionAtIndex(position_index_map_.at("ankle_joint_right"),
                             cassie_out.rightLeg.tarsusJoint.position);
  output->SetPositionAtIndex(position_index_map_.at("ankle_spring_joint_right"),
                             0.0);

  output->SetVelocityAtIndex(velocity_index_map_.at("hip_roll_leftdot"),
                             cassie_out.leftLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("hip_yaw_leftdot"),
                             cassie_out.leftLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("hip_pitch_leftdot"),
                             cassie_out.leftLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("knee_leftdot"),
                             cassie_out.leftLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("toe_leftdot"),
                             cassie_out.leftLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("knee_joint_leftdot"),
                             cassie_out.leftLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("ankle_joint_leftdot"),
                             cassie_out.leftLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("ankle_spring_joint_leftdot"),
                             0.0);

  output->SetVelocityAtIndex(velocity_index_map_.at("hip_roll_rightdot"),
                             cassie_out.rightLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("hip_yaw_rightdot"),
                             cassie_out.rightLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("hip_pitch_rightdot"),
                             cassie_out.rightLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("knee_rightdot"),
                             cassie_out.rightLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("toe_rightdot"),
                             cassie_out.rightLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("knee_joint_rightdot"),
                             cassie_out.rightLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("ankle_joint_rightdot"),
                             cassie_out.rightLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocity_index_map_.at("ankle_spring_joint_rightdot"),
                             0.0);

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
    q[3] =  1;
  }
  solveFourbarLinkage(q, &left_heel_spring, &right_heel_spring);
  output->SetPositionAtIndex(position_index_map_.at("ankle_spring_joint_left"),
                             left_heel_spring);
  output->SetPositionAtIndex(position_index_map_.at("ankle_spring_joint_right"),
                             right_heel_spring);
}


void CassieRbtStateEstimator::AssignFloatingBaseStateToOutputVector(
    const VectorXd& est_fb_state, OutputVector<double>* output) const {
  // TODO(yminchen): Joints names need to be changed when we move to MBP
  output->SetPositionAtIndex(position_index_map_.at("base_x"), est_fb_state(0));
  output->SetPositionAtIndex(position_index_map_.at("base_y"), est_fb_state(1));
  output->SetPositionAtIndex(position_index_map_.at("base_z"), est_fb_state(2));
  output->SetPositionAtIndex(position_index_map_.at("base_qw"), est_fb_state(3));
  output->SetPositionAtIndex(position_index_map_.at("base_qx"), est_fb_state(4));
  output->SetPositionAtIndex(position_index_map_.at("base_qy"), est_fb_state(5));
  output->SetPositionAtIndex(position_index_map_.at("base_qz"), est_fb_state(6));

  output->SetVelocityAtIndex(velocity_index_map_.at("base_wx"), est_fb_state(7));
  output->SetVelocityAtIndex(velocity_index_map_.at("base_wy"), est_fb_state(8));
  output->SetVelocityAtIndex(velocity_index_map_.at("base_wz"), est_fb_state(9));
  output->SetVelocityAtIndex(velocity_index_map_.at("base_vx"), est_fb_state(10));
  output->SetVelocityAtIndex(velocity_index_map_.at("base_vy"), est_fb_state(11));
  output->SetVelocityAtIndex(velocity_index_map_.at("base_vz"), est_fb_state(12));
}


/// contactEstimation() determines which foot is in contact with the ground
/// based on the state/input feedback of the robot and the imu acceleration
///
/// Input:
///  - OutputVector `output` containing the positions, velocities and
///    actuator torques of the robot
///  - time `dt` elapsed between previous iteration and current iteration
///  - discretevalues `discrete_state` to store states related to contact
///    estimation
/// Ouput: left contact `left_contact` and right contact `right_contact` that
///  indicate if the corresponding foot is in contact with the ground
///
/// Assumptions:
///  1. the swing leg doesn't stop during single support
///  2. flight mode is not present during the gait
///
/// Algorithm:
///  The contact is estimated based on
///   1. compression in the spring
///   2. three optimizations one for each double support, left support and
///   right support
///  If the compression in the left (right) heel/ankle spring is more than the
///  set threshold, the left (right) foot is estimated to be in contact with
///  the ground.
///  Additionally, the optimal cost from the optimization is used to augument
///  the estimation from the spring. In each optimization, the residual of the
///  EoM is calculated based on the assumption of the stance. the assumption of
///  stance is done by imposing a constraint for the acceleration of the stance
///  foot. The acceleration of the pelvis is also constrained to match the imu
///  acceleration. The cost from the three optimizations are compared. The
///  optimization with the least cost is assumed to be the actual stance.
///  During impact, both the legs have some non-zero acceleration and hence the
///  optimal costs will all be high. This case is assumed to be double stance.
void CassieRbtStateEstimator::contactEstimation(
    const OutputVector<double>& output, const double& dt,
    DiscreteValues<double>* discrete_state,
    int* left_contact, int* right_contact) const {
  const int n_v = tree_.get_num_velocities();

  // Indices
  const int left_toe_ind = GetBodyIndexFromName(tree_, "toe_left");
  const int right_toe_ind = GetBodyIndexFromName(tree_, "toe_right");
  const int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");

  // Cache
  KinematicsCache<double> cache = tree_.doKinematics(output.GetPositions(),
      output.GetVelocities(), true);

  // M, C and B matrices
  MatrixXd M = tree_.massMatrix(cache);
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  MatrixXd C = tree_.dynamicsBiasTerm(cache, no_external_wrenches);
  MatrixXd B = tree_.B;

  // control input - obtained from cassie_input temporarily
  VectorXd u = output.GetEfforts();

  // Jb - Jacobian for fourbar linkage
  MatrixXd Jb = tree_.positionConstraintsJacobian(cache, false);
  VectorXd Jb_dot_times_v = tree_.positionConstraintsJacDotTimesV(cache);

  /** Jc{l, r}{f, r} - contact Jacobians **/
  // l - left; r - right; f - front; r - rear
  MatrixXd Jclf = tree_.transformPointsJacobian(cache,
      front_contact_disp_, left_toe_ind, 0, false);
  MatrixXd Jclr = tree_.transformPointsJacobian(cache,
      rear_contact_disp_, left_toe_ind, 0, false);

  MatrixXd Jcl(3*2, Jclf.cols());
  Jcl.block(0, 0, 3, Jclf.cols()) = Jclf;
  Jcl.block(3, 0, 3, Jclr.cols()) = Jclr;

  MatrixXd Jcrf = tree_.transformPointsJacobian(cache,
      front_contact_disp_, right_toe_ind, 0, false);
  MatrixXd Jcrr = tree_.transformPointsJacobian(cache,
      rear_contact_disp_, right_toe_ind, 0, false);

  MatrixXd Jcr(3*2, Jcrf.cols());
  Jcr.block(0, 0, 3, Jcrf.cols()) = Jcrf;
  Jcr.block(3, 0, 3, Jcrr.cols()) = Jcrr;

  // Contact jacobian dot times v
  VectorXd Jclf_dot_times_v = tree_.transformPointsJacobianDotTimesV(
      cache, front_contact_disp_, left_toe_ind, 0);
  VectorXd Jclr_dot_times_v = tree_.transformPointsJacobianDotTimesV(
      cache, rear_contact_disp_, left_toe_ind, 0);

  VectorXd Jcl_dot_times_v(6);
  Jcl_dot_times_v.head(3) = Jclf_dot_times_v;
  Jcl_dot_times_v.tail(3) = Jclr_dot_times_v;

  VectorXd Jcrf_dot_times_v = tree_.transformPointsJacobianDotTimesV(
      cache, front_contact_disp_, right_toe_ind, 0);
  VectorXd Jcrr_dot_times_v = tree_.transformPointsJacobianDotTimesV(
      cache, rear_contact_disp_, right_toe_ind, 0);

  VectorXd Jcr_dot_times_v(6);
  Jcr_dot_times_v.head(3) = Jcrf_dot_times_v;
  Jcr_dot_times_v.tail(3) = Jcrr_dot_times_v;

  // double left_leg_velocity = (Jcl*output.GetVelocities()).norm();
  // double right_leg_velocity = (Jcr*output.GetVelocities()).norm();

  MatrixXd J_imu = tree_.transformPointsJacobian(cache,
      imu_pos_, pelvis_ind, 0, false);
  VectorXd J_imu_dot_times_v = tree_.transformPointsJacobianDotTimesV(
      cache, imu_pos_, pelvis_ind, 0);

  Vector3d alpha_imu = output.GetIMUAccelerations();

  RigidBody<double>* pelvis_body = tree_.FindBody("pelvis");
  Isometry3d pelvis_pose = tree_.CalcBodyPoseInWorldFrame(cache,
      *pelvis_body);
  MatrixXd R_WB = pelvis_pose.linear();
  Vector3d gravity_in_pelvis_frame = R_WB.transpose()*gravity_;
  alpha_imu -= gravity_in_pelvis_frame;

  std::vector<double> optimal_cost;

  // Mathematical program - double contact
  // Equality constraint
  fourbar_constraint_->UpdateCoefficients(Jb, -1*Jb_dot_times_v);

  MatrixXd CL_coeff(Jcl.rows(), Jcl.cols() + 6);
  CL_coeff << Jcl, MatrixXd::Identity(6, 6);
  left_contact_constraint_->UpdateCoefficients(CL_coeff, -1*Jcl_dot_times_v);

  MatrixXd CR_coeff(Jcr.rows(), Jcr.cols() + 6);
  CR_coeff << Jcr, MatrixXd::Identity(6, 6);
  right_contact_constraint_->UpdateCoefficients(CR_coeff, -1*Jcr_dot_times_v);

  MatrixXd IMU_coeff(J_imu.rows(), J_imu.cols() + 3);
  IMU_coeff << J_imu, MatrixXd::Identity(3, 3);
  imu_accel_constraint_->UpdateCoefficients(IMU_coeff,
      -1*J_imu_dot_times_v + alpha_imu);

  // Cost
  int A_cols = n_v + Jb.rows() + Jcl.rows() + Jcr.rows();
  MatrixXd cost_A(n_v, A_cols);
  cost_A << M, -1*Jb.transpose(), -1*Jcl.transpose(), -1*Jcr.transpose();
  VectorXd cost_b(n_v);
  cost_b = B*u - C;

  quadcost_eom_->UpdateCoefficients(2*cost_A.transpose()*cost_A +
      eps_cost_*MatrixXd::Identity(A_cols, A_cols),
      -2*cost_A.transpose()*cost_b);
  quadcost_eps_cl_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(6, 6),
      VectorXd::Zero(6, 1));
  quadcost_eps_cr_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(6, 6),
      VectorXd::Zero(6, 1));
  quadcost_eps_imu_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(3, 3),
      VectorXd::Zero(3, 1));

  // Initial guess
  quadprog_->SetInitialGuess(ddq_,
      discrete_state->get_vector(ddq_double_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_b_,
      discrete_state->get_vector(lambda_b_double_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_cl_,
      discrete_state->get_vector(lambda_cl_double_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_cr_,
      discrete_state->get_vector(lambda_cr_double_init_idx_).get_value());

  // Solve the optimization problem
  const drake::solvers::MathematicalProgramResult result_double =
      drake::solvers::Solve(*quadprog_);

  if (!result_double.is_success()) {
    // If the optimization fails, push infinity into the optimal_cost vector
    optimal_cost.push_back(std::numeric_limits<double>::infinity());

    // Initialize the optimization at the next time step with zeros
    discrete_state->get_mutable_vector(
        ddq_double_init_idx_).get_mutable_value() <<
        VectorXd::Zero(n_v, 1);
    discrete_state->get_mutable_vector(
        lambda_b_double_init_idx_).get_mutable_value() <<
        VectorXd::Zero(2, 1);
    discrete_state->get_mutable_vector(
        lambda_cl_double_init_idx_).get_mutable_value() <<
        VectorXd::Zero(6, 1);
    discrete_state->get_mutable_vector(
        lambda_cr_double_init_idx_).get_mutable_value() <<
        VectorXd::Zero(6, 1);
  } else {
    // Push the optimal cost to the optimal_cost vector
    optimal_cost.push_back(result_double.get_optimal_cost() +
        cost_b.transpose()*cost_b);

    VectorXd ddq_val = result_double.GetSolution(ddq_);
    VectorXd left_force = result_double.GetSolution(lambda_cl_);
    VectorXd right_force = result_double.GetSolution(lambda_cr_);

    // Save current estimate for initial guess in the next iteration
    discrete_state->get_mutable_vector(
        ddq_double_init_idx_).get_mutable_value() <<
        ddq_val;
    discrete_state->get_mutable_vector(
        lambda_b_double_init_idx_).get_mutable_value() <<
        result_double.GetSolution(lambda_b_);
    discrete_state->get_mutable_vector(
        lambda_cl_double_init_idx_).get_mutable_value() <<
        left_force;
    discrete_state->get_mutable_vector(
        lambda_cr_double_init_idx_).get_mutable_value() <<
        right_force;

    // Residual calculation
    VectorXd curr_residual = ddq_val*dt;
    curr_residual -= (output.GetVelocities() -
        discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_double = discrete_state->get_vector(
        filtered_residual_double_idx_).get_value();
    filtered_residual_double = filtered_residual_double +
      alpha_*(curr_residual - filtered_residual_double);
    discrete_state->get_mutable_vector(
        filtered_residual_double_idx_).get_mutable_value() <<
        filtered_residual_double;
  }

  // Mathematical program - left contact
  // Equality constraints
  fourbar_constraint_->UpdateCoefficients(Jb, -1*Jb_dot_times_v);
  left_contact_constraint_->UpdateCoefficients(CL_coeff, -1*Jcl_dot_times_v);
  right_contact_constraint_->UpdateCoefficients(MatrixXd::Zero(6, n_v + 6),
      VectorXd::Zero(6, 1));
  imu_accel_constraint_->UpdateCoefficients(IMU_coeff,
      -1*J_imu_dot_times_v + alpha_imu);

  // Cost
  cost_A << M, -1*Jb.transpose(), -1*Jcl.transpose(), MatrixXd::Zero(n_v, 6);

  quadcost_eom_->UpdateCoefficients(2*cost_A.transpose()*cost_A +
      eps_cost_*MatrixXd::Identity(A_cols, A_cols),
      -2*cost_A.transpose()*cost_b);
  quadcost_eps_cl_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(6, 6),
      VectorXd::Zero(6, 1));
  quadcost_eps_cr_->UpdateCoefficients(
      MatrixXd::Zero(6, 6), VectorXd::Zero(6, 1));
  quadcost_eps_imu_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(3, 3),
      VectorXd::Zero(3, 1));

  // Initial guess
  quadprog_->SetInitialGuess(ddq_,
      discrete_state->get_vector(ddq_left_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_b_,
      discrete_state->get_vector(lambda_b_left_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_cl_,
      discrete_state->get_vector(lambda_cl_left_init_idx_).get_value());

  // Solve the optimization problem
  const drake::solvers::MathematicalProgramResult result_left =
      drake::solvers::Solve(*quadprog_);

  if (!result_left.is_success()) {
    // Push infinity into optimal_costv vector if the optimization fails
    optimal_cost.push_back(std::numeric_limits<double>::infinity());

    // Initialize the optimization with zero at the next time step
    discrete_state->get_mutable_vector(
        ddq_left_init_idx_).get_mutable_value() <<
        VectorXd::Zero(n_v, 1);
    discrete_state->get_mutable_vector(
        lambda_b_left_init_idx_).get_mutable_value() <<
        VectorXd::Zero(2, 1);
    discrete_state->get_mutable_vector(
        lambda_cl_left_init_idx_).get_mutable_value() <<
        VectorXd::Zero(6, 1);
  } else {
    // Push the optimal cost to the optimal_cost vector
    optimal_cost.push_back(result_left.get_optimal_cost() +
        cost_b.transpose()*cost_b);

    VectorXd ddq_val = result_left.GetSolution(ddq_);
    VectorXd left_force = result_left.GetSolution(lambda_cl_);

    // Save current estimate for initial guess in the next iteration
    discrete_state->get_mutable_vector(
        ddq_left_init_idx_).get_mutable_value() <<
        ddq_val;
    discrete_state->get_mutable_vector(
        lambda_b_left_init_idx_).get_mutable_value() <<
        result_left.GetSolution(lambda_b_);
    discrete_state->get_mutable_vector(
        lambda_cl_left_init_idx_).get_mutable_value() <<
        left_force;

    // Residual calculation
    VectorXd curr_residual = ddq_val*dt;
    curr_residual -= (output.GetVelocities() -
        discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_left = discrete_state->get_vector(
        filtered_residual_left_idx_).get_value();
    filtered_residual_left = filtered_residual_left +
        alpha_*(curr_residual - filtered_residual_left);
    discrete_state->get_mutable_vector(
        filtered_residual_left_idx_).get_mutable_value() <<
        filtered_residual_left;
  }

  // Mathematical program - right contact
  // Equality constrtaint
  fourbar_constraint_->UpdateCoefficients(Jb, -1*Jb_dot_times_v);
  left_contact_constraint_->UpdateCoefficients(
      MatrixXd::Zero(6, n_v + 6), VectorXd::Zero(6, 1));
  right_contact_constraint_->UpdateCoefficients(
      CR_coeff, -1*Jcr_dot_times_v);
  imu_accel_constraint_->UpdateCoefficients(IMU_coeff,
      -1*J_imu_dot_times_v + alpha_imu);

  // Cost
  cost_A << M, -1*Jb.transpose(), MatrixXd::Zero(n_v, 6), -1*Jcr.transpose();

  quadcost_eom_->UpdateCoefficients(2*cost_A.transpose()*cost_A +
      eps_cost_*MatrixXd::Identity(A_cols, A_cols),
      -2*cost_A.transpose()*cost_b);
  quadcost_eps_cl_->UpdateCoefficients(
      MatrixXd::Zero(6, 6), VectorXd::Zero(6, 1));
  quadcost_eps_cr_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(6, 6),
      VectorXd::Zero(6, 1));
  quadcost_eps_imu_->UpdateCoefficients(
      w_soft_constraint_*MatrixXd::Identity(3, 3),
      VectorXd::Zero(3, 1));

  // Initial guess
  quadprog_->SetInitialGuess(ddq_,
      discrete_state->get_vector(ddq_right_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_b_,
      discrete_state->get_vector(lambda_b_right_init_idx_).get_value());
  quadprog_->SetInitialGuess(lambda_cr_,
      discrete_state->get_vector(lambda_cr_right_init_idx_).get_value());

  // Solve the optimization problem
  const drake::solvers::MathematicalProgramResult result_right =
      drake::solvers::Solve(*quadprog_);

  if (!result_right.is_success()) {
    // If the optimization fails, push infinity to the optimal_cost vector
    optimal_cost.push_back(std::numeric_limits<double>::infinity());

    // Initialize the optimization with zero at the next time step
    discrete_state->get_mutable_vector(
        ddq_right_init_idx_).get_mutable_value() <<
        VectorXd::Zero(n_v, 1);
    discrete_state->get_mutable_vector(
        lambda_b_right_init_idx_).get_mutable_value() <<
        VectorXd::Zero(2, 1);
    discrete_state->get_mutable_vector(
        lambda_cr_right_init_idx_).get_mutable_value() <<
        VectorXd::Zero(6 ,1);
  } else {
    // Push the optimal cost to optimal_cost vector
    optimal_cost.push_back(result_right.get_optimal_cost() +
        cost_b.transpose()*cost_b);

    VectorXd ddq_val = result_right.GetSolution(ddq_);
    VectorXd right_force = result_right.GetSolution(lambda_cr_);

    // Save current estimate for initial guess in the next iteration
    discrete_state->get_mutable_vector(
        ddq_right_init_idx_).get_mutable_value() <<
        ddq_val;
    discrete_state->get_mutable_vector(
        lambda_b_right_init_idx_).get_mutable_value() <<
        result_right.GetSolution(lambda_b_);
    discrete_state->get_mutable_vector(
        lambda_cr_right_init_idx_).get_mutable_value() <<
        right_force;

    // Residual calculation
    VectorXd curr_residual = ddq_val*dt;
    curr_residual -= (output.GetVelocities() -
        discrete_state->get_vector(previous_velocity_idx_).get_value());
    VectorXd filtered_residual_right = discrete_state->get_vector(
        filtered_residual_right_idx_).get_value();
    filtered_residual_right = filtered_residual_right +
      alpha_*(curr_residual - filtered_residual_right);
    discrete_state->get_mutable_vector(
        filtered_residual_right_idx_).get_mutable_value() <<
      filtered_residual_right;
  }

  // Estimate contact based on optimization results
  // The vector optimal_cost has double support, left support and right support
  // costs in order. The corresponding indices are 0, 1, 2.
  // Here we get the index of min of left and right support costs.
  auto min_it = std::min_element(std::next(optimal_cost.begin(), 1),
      optimal_cost.end());
  int min_index = std::distance(optimal_cost.begin(), min_it);

  // If all three costs are high, we believe it's going through impact event,
  // and we assume it's double support. (Therefore, it won't predict the case
  // where the robot transition from flight phase to single support. It'd say
  // it's double support.)
  if ((optimal_cost[0] >= cost_threshold_ &&
        optimal_cost[1] >= cost_threshold_ &&
        optimal_cost[2] >= cost_threshold_)) {
    *left_contact = 1;
    *right_contact = 1;
  } else if (min_index == 1) {
    *left_contact = 1;
  } else if (min_index == 2) {
    *right_contact = 1;
  }

  // Update contact estimation based on spring deflection information
  // We say a foot is in contact with the ground if spring deflection is over
  // a threshold. We don't update anything if it's under the threshold.
  double left_knee_spring = output.GetPositionAtIndex(
      position_index_map_.at("knee_joint_left"));
  double right_knee_spring = output.GetPositionAtIndex(
      position_index_map_.at("knee_joint_right"));
  double left_heel_spring = output.GetPositionAtIndex(
      position_index_map_.at("ankle_spring_joint_left"));
  double right_heel_spring = output.GetPositionAtIndex(
      position_index_map_.at("ankle_spring_joint_right"));
  if (left_knee_spring < knee_spring_threshold_ ||
        left_heel_spring < heel_spring_threshold_) {
    *left_contact = 1;
  }
  if (right_knee_spring < knee_spring_threshold_ ||
        right_heel_spring < heel_spring_threshold_) {
    *right_contact = 1;
  }

  // Record previous velocity (used in acceleration residual)
  discrete_state->get_mutable_vector(
      previous_velocity_idx_).get_mutable_value() << output.GetVelocities();

  // Ground truth of contact
  /*
  RigidBodyTree<double>* tree_with_ground = nullptr;
  tree_with_ground = const_cast<RigidBodyTree<double>*>(&tree_);
  std::vector<drake::multibody::collision::PointPair<double>> pairs;
  pairs = tree_with_ground->ComputeMaximumDepthCollisionPoints(cache,
      true, false);
  int gtl = 0;
  int gtr = 0;
  for (const auto& pair: pairs) {
    if (pair.distance < 0.0) {
      if (pair.elementA->get_body()->get_body_index() == 18) {
        gtl += 1;
      }
      if (pair.elementA->get_body()->get_body_index() == 20) {
        gtr += 1;
      }
    }
  }
  */
}


EventStatus CassieRbtStateEstimator::Update(const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // TODO(yminchen): delete the testing code when you fix the time delay issue
  // Testing
  // const auto& cassie_out =
  //     this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
  // cout << "\nIn per-step update: lcm_time = " <<
  //      cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  // cout << "In per-step update: context_time = " << context.get_time() << endl;

  // Get current time and previous time
  double current_time = context.get_time();
  double prev_t = discrete_state->get_vector(time_idx_).get_value()(0);

  // TODO(yminchen): delete the testing code when you fix the time delay issue
  // Testing
  // current_time = cassie_out.pelvis.targetPc.taskExecutionTime;

  if (current_time > prev_t) {
    double dt = current_time - prev_t;

    // Perform State Estimation (in several steps)
    // Step 1 - Solve for the unknown joint angle
    // This step is done in AssignNonFloatingBaseStateToOutputVector()

    // Step 2 - EKF (update step)

    // Step 3 - Estimate which foot/feet are in contact with the ground
    // Create robot output vector for contact estimation
    OutputVector<double> output(tree_.get_num_positions(),
                                tree_.get_num_velocities(),
                                tree_.get_num_actuators());
    // Assign values to robot output vector
    const auto& cassie_out =
        this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
    AssignImuValueToOutputVector(cassie_out, &output);
    AssignActuationFeedbackToOutputVector(cassie_out, &output);
    AssignNonFloatingBaseStateToOutputVector(cassie_out, &output);
    AssignFloatingBaseStateToOutputVector(
        context.get_discrete_state(state_idx_).get_value(), &output);
    // Since we don't have EKF yet, we get the floating base state
    // from ground truth (CASSIE_STATE)
    // TODO(yminchen): delete this later
    const OutputVector<double>* cassie_state = (OutputVector<double>*)
        this->EvalVectorInput(context, state_input_port_);
    output.SetPositionAtIndex(position_index_map_.at("base_x"),
                              cassie_state->GetPositions()[0]);
    output.SetPositionAtIndex(position_index_map_.at("base_y"),
                              cassie_state->GetPositions()[1]);
    output.SetPositionAtIndex(position_index_map_.at("base_z"),
                              cassie_state->GetPositions()[2]);
    output.SetPositionAtIndex(position_index_map_.at("base_qw"),
                              cassie_state->GetPositions()[3]);
    output.SetPositionAtIndex(position_index_map_.at("base_qx"),
                              cassie_state->GetPositions()[4]);
    output.SetPositionAtIndex(position_index_map_.at("base_qy"),
                              cassie_state->GetPositions()[5]);
    output.SetPositionAtIndex(position_index_map_.at("base_qz"),
                              cassie_state->GetPositions()[6]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_wx"),
                              cassie_state->GetVelocities()[0]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_wy"),
                              cassie_state->GetVelocities()[1]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_wz"),
                              cassie_state->GetVelocities()[2]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_vx"),
                              cassie_state->GetVelocities()[3]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_vy"),
                              cassie_state->GetVelocities()[4]);
    output.SetVelocityAtIndex(velocity_index_map_.at("base_vz"),
                              cassie_state->GetVelocities()[5]);
    // We get 0's cassie_state in the beginning because dispatcher_robot_out is
    // not triggerred by CASSIE_STATE message.
    // This wouldn't be an issue when you don't use ground truth state.
    if (output.GetPositions().head(7).norm() == 0){
      output.SetPositionAtIndex(position_index_map_.at("base_qw"), 1);
    }

    // Estimate feet contacts
    int left_contact = 0;
    int right_contact = 0;
    contactEstimation(output, dt,
        discrete_state, &left_contact, &right_contact);

    // Step 4 - EKF (measurement step)


    // Step 5 - Assign values to states
    // Below is how you should assign the state at the end of this Update
    // discrete_state->get_mutable_vector(ekf_X_idx_).get_mutable_value() = ...;
    // discrete_state->get_mutable_vector(time_idx_).get_mutable_value() = ...;

    // You can convert a rotational matrix to quaternion using Eigen
    // https://stackoverflow.com/questions/21761909/eigen-convert-matrix3d-rotation-to-quaternion
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

    // Then convert Eigen::Quaterion to (w,x,y,z) by drake's QuaternionToVectorWxyz()
    // https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1multibody.html#ad1b559878de179a7e363846fa67f58c0


    // Question: Do we need to filter the gyro value?
    // We will get the bias (parameter) from EKF

    discrete_state->get_mutable_vector(time_idx_).get_mutable_value() <<
      current_time;
  }
  return EventStatus::Succeeded();
}


/// Workhorse state estimation function. Given a `cassie_out_t`, compute the
/// estimated state as an OutputVector
/// Since it needs to map from a struct to a vector, and no assumptions on the
/// ordering of the vector are made, utilizies index maps to make this mapping.
void CassieRbtStateEstimator::CopyStateOut(
    const Context<double>& context, OutputVector<double>* output) const {
  const auto& cassie_out =
      this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
  // There might be a better way to initialize?
  auto data = output->get_mutable_data();  // This doesn't affect timestamp value
  data = VectorXd::Zero(data.size());

  // Assign values robot output vector
  // Copy imu values and robot state excluding floating base
  AssignImuValueToOutputVector(cassie_out, output);
  AssignActuationFeedbackToOutputVector(cassie_out, output);
  AssignNonFloatingBaseStateToOutputVector(cassie_out, output);
  // Copy the floating base base state
  if (is_floating_base_) {
    AssignFloatingBaseStateToOutputVector(
        context.get_discrete_state(state_idx_).get_value(), output);

    // Since we don't have EKF yet, we get the floating base state
    // from ground truth (CASSIE_STATE)
    // TODO(yminchen): delete this later
    const OutputVector<double>* cassie_state = (OutputVector<double>*)
        this->EvalVectorInput(context, state_input_port_);
    output->SetPositionAtIndex(position_index_map_.at("base_x"),
                              cassie_state->GetPositions()[0]);
    output->SetPositionAtIndex(position_index_map_.at("base_y"),
                              cassie_state->GetPositions()[1]);
    output->SetPositionAtIndex(position_index_map_.at("base_z"),
                              cassie_state->GetPositions()[2]);
    output->SetPositionAtIndex(position_index_map_.at("base_qw"),
                              cassie_state->GetPositions()[3]);
    output->SetPositionAtIndex(position_index_map_.at("base_qx"),
                              cassie_state->GetPositions()[4]);
    output->SetPositionAtIndex(position_index_map_.at("base_qy"),
                              cassie_state->GetPositions()[5]);
    output->SetPositionAtIndex(position_index_map_.at("base_qz"),
                              cassie_state->GetPositions()[6]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_wx"),
                              cassie_state->GetVelocities()[0]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_wy"),
                              cassie_state->GetVelocities()[1]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_wz"),
                              cassie_state->GetVelocities()[2]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_vx"),
                              cassie_state->GetVelocities()[3]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_vy"),
                              cassie_state->GetVelocities()[4]);
    output->SetVelocityAtIndex(velocity_index_map_.at("base_vz"),
                              cassie_state->GetVelocities()[5]);
    // We get 0's cassie_state in the beginning because dispatcher_robot_out is
    // not triggerred by CASSIE_STATE message.
    // This wouldn't be an issue when you don't use ground truth state.
    if (output->GetPositions().head(7).norm()==0){
      output->SetPositionAtIndex(position_index_map_.at("base_qw"), 1);
    }
  }

  // TODO(yminchen): delete the testing code when you fix the time delay issue
  // Testing
  // auto state_time = context.get_discrete_state(time_idx_).get_value();
  // cout << "  In copyStateOut: lcm_time = " << cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  // cout << "  In copyStateOut: state_time = " << state_time << endl;
  // cout << "  In copyStateOut: context_time = " << context.get_time() << endl;
}

}  // namespace systems
}  // namespace dairlib
