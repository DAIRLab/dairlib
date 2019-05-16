#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include <math.h>

namespace dairlib {
namespace systems {

using std::cout;
using std::endl;
using std::string;
using drake::systems::Context;
using drake::systems::LeafSystem;
using multibody::GetBodyIndexFromName;
using drake::solvers::MathematicalProgram;
using drake::solvers::Solve;

CassieRbtStateEstimator::CassieRbtStateEstimator(
  const RigidBodyTree<double>& tree, bool is_floating_base) :
  tree_(tree), is_floating_base_(is_floating_base) {
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);
  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);

  cassie_out_input_port_ = this->DeclareAbstractInputPort("cassie_out_t",
                                 drake::Value<cassie_out_t> {}).get_index();

  state_input_port_ = this->DeclareVectorInputPort(
      OutputVector<double>(tree_.get_num_positions(),
                           tree_.get_num_velocities(), tree_.get_num_actuators())).get_index();

  command_input_port_ = this->DeclareVectorInputPort(
      TimestampedVector<double>(tree_.get_num_actuators())).get_index();

  this->DeclareVectorOutputPort(
    OutputVector<double>(tree.get_num_positions(),
                         tree.get_num_velocities(), tree.get_num_actuators()),
    &CassieRbtStateEstimator::CopyStateOut);

  // if (is_floating_base) {
    DeclarePerStepDiscreteUpdateEvent(&CassieRbtStateEstimator::Update);
    state_idx_ = DeclareDiscreteState(7 + 6); // estimated floating base
    ekf_X_idx_ = DeclareDiscreteState(27); // estimated EKF state
    time_idx_ = DeclareDiscreteState(VectorXd::Zero(1)); // previous time
  // }

  // Initialize body indices
  left_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_left");
  right_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_right");
  left_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_left");
  right_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_right");
  if (left_thigh_ind_ == -1 || right_thigh_ind_ == -1 ||
      left_heel_spring_ind_ == -1 || right_heel_spring_ind_ == -1 )
    std::cout << "In cassie_rbt_state_estimator.cc,"
              " body indices were not set correctly.\n";

}

void CassieRbtStateEstimator::solveFourbarLinkage(
  VectorXd q_init,
  double & left_heel_spring, double & right_heel_spring) const {

  // TODO(yminchen): get the numbers below from tree
  // Get the rod length
  Vector3d rod_on_heel_spring(.11877, -.01, 0.0);
  double spring_length = rod_on_heel_spring.norm();
  // Spring rest angle offset
  double spring_rest_offset =
    atan(rod_on_heel_spring(1) / rod_on_heel_spring(0));

  // Get the rod length projected to thigh-shin plane
  double rod_length = 0.5012;          // from cassie_utils
  Vector3d rod_on_thigh_left(0.0, 0.0, 0.045);
  Vector3d rod_on_thigh_right(0.0, 0.0, -0.045);

  std::vector<Vector3d> rod_on_thigh{rod_on_thigh_left, rod_on_thigh_right};
  std::vector<int> thigh_ind{left_thigh_ind_, right_thigh_ind_};
  std::vector<int> heel_spring_ind{
    left_heel_spring_ind_, right_heel_spring_ind_};

  KinematicsCache<double> cache = tree_.doKinematics(q_init);
  for (int i = 0; i < 2; i++) {
    // Get thigh pose and heel spring pose
    const Isometry3d thigh_pose =
      tree_.CalcBodyPoseInWorldFrame(cache, tree_.get_body(thigh_ind[i]));
    const Vector3d thigh_pos = thigh_pose.translation();
    const MatrixXd thigh_rot_mat = thigh_pose.linear();

    const Isometry3d heel_spring_pose =
      tree_.CalcBodyPoseInWorldFrame(cache,
                                     tree_.get_body(heel_spring_ind[i]));
    const Vector3d heel_spring_pos = heel_spring_pose.translation();
    const MatrixXd heel_spring_rot_mat = heel_spring_pose.linear();

    // Get r_heel_spring_base_to_thigh_ball_joint
    Vector3d r_though_ball_joint = thigh_pos + thigh_rot_mat * rod_on_thigh[i];
    Vector3d r_heel_spring_base_to_thigh_ball_joint =
      r_though_ball_joint - heel_spring_pos;
    Vector3d r_thigh_ball_joint_wrt_heel_spring_base =
      heel_spring_rot_mat.transpose() * r_heel_spring_base_to_thigh_ball_joint;

    // Get the projected rod length in the xy plane of heel spring base
    double projected_rod_length =
      sqrt(pow(rod_length, 2) -
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
      left_heel_spring = spring_deflect_sign * heel_spring_angle
                         - spring_rest_offset;
    else
      right_heel_spring = spring_deflect_sign * heel_spring_angle
                          - spring_rest_offset;
  }  // end for
}


void CassieRbtStateEstimator::AssignNonFloatingBaseToOutputVector(
  OutputVector<double>* output, const cassie_out_t& cassie_out) const {
  // Copy the robot state excluding floating base
  // TODO(yuming): check what cassie_out.leftLeg.footJoint.position is.
  // Similarly, the other leg and the velocity of these joints.
  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_left"),
                             cassie_out.leftLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_left"),
                             cassie_out.leftLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_left"),
                             cassie_out.leftLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_left"),
                             cassie_out.leftLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_left"),
                             cassie_out.leftLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_left"),
                             cassie_out.leftLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_left"),
                             cassie_out.leftLeg.tarsusJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_left")
                             , 0.0);

  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_right"),
                             cassie_out.rightLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_right"),
                             cassie_out.rightLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_right"),
                             cassie_out.rightLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_right"),
                             cassie_out.rightLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_right"),
                             cassie_out.rightLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_right"),
                             cassie_out.rightLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_right"),
                             cassie_out.rightLeg.tarsusJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_right")
                             , 0.0);

  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_leftdot"),
                             cassie_out.leftLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_leftdot"),
                             cassie_out.leftLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_leftdot"),
                             cassie_out.leftLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_leftdot"),
                             cassie_out.leftLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_leftdot"),
                             cassie_out.leftLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_leftdot"),
                             cassie_out.leftLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_leftdot"),
                             cassie_out.leftLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_leftdot")
                             , 0.0);

  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_rightdot"),
                             cassie_out.rightLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_rightdot"),
                             cassie_out.rightLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_rightdot"),
                             cassie_out.rightLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_rightdot"),
                             cassie_out.rightLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_rightdot"),
                             cassie_out.rightLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_rightdot"),
                             cassie_out.rightLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_rightdot"),
                             cassie_out.rightLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_rightdot")
                             , 0.0);

  // Copy actuators
  // We don't need to copy the value of motor torque, since we are not passing
  // it to the controller. However, we can pass it if we are doing torque
  // control, for example.
  /*output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_left_motor"),
                           cassie_out.leftLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_left_motor"),
                           cassie_out.leftLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_left_motor"),
                           cassie_out.leftLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_left_motor"),
                           cassie_out.leftLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_left_motor"),
                           cassie_out.leftLeg.footDrive.torque);

  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_right_motor"),
                           cassie_out.rightLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_right_motor"),
                           cassie_out.rightLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_right_motor"),
                           cassie_out.rightLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_right_motor"),
                           cassie_out.rightLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_right_motor"),
                           cassie_out.rightLeg.footDrive.torque);*/
}



EventStatus CassieRbtStateEstimator::Update(const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  // Testing
  const auto& cassie_out =
    this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
  cout << "\nIn per-step update: lcm_time = " << cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  cout << "In per-step update: context_time = " << context.get_time() << endl;

  // Get current time and previous time
  double current_time = context.get_time();
  double prev_t = discrete_state->get_mutable_vector(time_idx_).get_value()(0);

  // Testing
  // current_time = cassie_out.pelvis.targetPc.taskExecutionTime;

  if (current_time > prev_t) {
    double dt = current_time - prev_t;
    discrete_state->get_mutable_vector(time_idx_).get_mutable_value() <<
        current_time;

    // Testing
    cout << "In per-step update: updated state_time = " <<
         discrete_state->get_mutable_vector(time_idx_).get_mutable_value() << endl;

    // Perform State Estimation (in several steps)

    // Step 1 - Solve for the unknown joint angle
    //TODO(yminchen): Is there a way to avoid copying state two times?
                    //Currently, we copy in Update() and CopyStateOut().
    const auto& cassie_out =
      this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
    OutputVector<double> output(tree_.get_num_positions(),
                                tree_.get_num_velocities(),
                                tree_.get_num_actuators());
    AssignNonFloatingBaseToOutputVector(&output, cassie_out);
    double left_heel_spring = 0;
    double right_heel_spring = 0;
    solveFourbarLinkage(output.GetPositions(),
                        left_heel_spring, right_heel_spring);

    // TODO(yminchen):
    // You can test the contact force estimator here using fixed based.
    // You can implement step 3 independently of the EKF.

    // The concern when moving to floating based simulation:
    // The simulatino update rate is about 30-60 Hz.


    // Step 2 - EKF (update step)


    // Step 3 - Estimate which foot/feet are in contact with the ground
    const OutputVector<double>* cassie_state = (OutputVector<double>*)
      this->EvalVectorInput(context, state_input_port_);
    const TimestampedVector<double>* cassie_command = (TimestampedVector<double>*)
      this->EvalVectorInput(context, command_input_port_);

    const int num_positions = tree_.get_num_positions();
    const int num_velocities = tree_.get_num_velocities();

    /* Indices */
    const int world_ind = GetBodyIndexFromName(tree_, "world");
    const int left_toe_ind = GetBodyIndexFromName(tree_, "toe_left");
    const int right_toe_ind = GetBodyIndexFromName(tree_, "toe_right");
    const int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");

    /* Cache */
    KinematicsCache<double> cache = tree_.doKinematics(cassie_state->GetPositions(),
        cassie_state->GetVelocities(),
        true);

    /* M, C and B matrices */
    Eigen::MatrixXd M = tree_.massMatrix(cache);
    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
    Eigen::MatrixXd C = tree_.dynamicsBiasTerm(cache, no_external_wrenches);
    Eigen::MatrixXd B = tree_.B;

    /* Control input  */
    Eigen::VectorXd u = cassie_command->get_data();

    /** Jb - Jacobian for fourbar linkage (2 rows) **/
    Eigen::MatrixXd Jb = tree_.positionConstraintsJacobian(cache, false);
    Eigen::VectorXd Jb_dot_times_v = tree_.positionConstraintsJacDotTimesV(cache);

    /** Jc{l, r}{f, r} - contact Jacobians **/
    /* l - left; r - right; f - front; r - rear */
    Eigen::Vector3d front_contact_disp;
    front_contact_disp << -0.0457, 0.112, 0;
    Eigen::Vector3d rear_contact_disp;
    rear_contact_disp << 0.088, 0, 0;

    Eigen::MatrixXd Jclf = tree_.transformPointsJacobian(cache,
        front_contact_disp, left_toe_ind, 0, false);
    Eigen::MatrixXd Jclr = tree_.transformPointsJacobian(cache,
        rear_contact_disp, left_toe_ind, 0, false);

    Eigen::MatrixXd Jcl(3*2, Jclf.cols());
    Jcl.block(0, 0, 3, Jclf.cols()) = Jclf;
    Jcl.block(3, 0, 3, Jclr.cols()) = Jclr;

    Eigen::MatrixXd Jcrf = tree_.transformPointsJacobian(cache,
        front_contact_disp, right_toe_ind, 0, false);
    Eigen::MatrixXd Jcrr = tree_.transformPointsJacobian(cache,
        rear_contact_disp, right_toe_ind, 0, false);

    Eigen::MatrixXd Jcr(3*2, Jcrf.cols());
    Jcr.block(0, 0, 3, Jcrf.cols()) = Jcrf;
    Jcr.block(3, 0, 3, Jcrr.cols()) = Jcrr;

    /* Contact jacobian dot times v */
    Eigen::VectorXd Jclf_dot_times_v = tree_.transformPointsJacobianDotTimesV(cache,
        front_contact_disp, left_toe_ind, 0);
    Eigen::VectorXd Jclr_dot_times_v = tree_.transformPointsJacobianDotTimesV(cache,
        rear_contact_disp, left_toe_ind, 0);

    Eigen::VectorXd Jcl_dot_times_v(6);
    Jcl_dot_times_v.head(3) = Jclf_dot_times_v;
    Jcl_dot_times_v.tail(3) = Jclr_dot_times_v;

    Eigen::VectorXd Jcrf_dot_times_v = tree_.transformPointsJacobianDotTimesV(cache,
        front_contact_disp, right_toe_ind, 0);
    Eigen::VectorXd Jcrr_dot_times_v = tree_.transformPointsJacobianDotTimesV(cache,
        rear_contact_disp, right_toe_ind, 0);

    Eigen::VectorXd Jcr_dot_times_v(6);
    Jcr_dot_times_v.head(3) = Jcrf_dot_times_v;
    Jcr_dot_times_v.tail(3) = Jcrr_dot_times_v;

    Eigen::Vector3d imu_pos;
    // TODO: change IMU file to include this as a variable
    imu_pos << 0.03155, 0, -0.07996; // IMU location wrt pelvis.
    Eigen::MatrixXd J_imu = tree_.transformPointsJacobian(cache,
        imu_pos, pelvis_ind, 0, false);
    Eigen::VectorXd J_imu_dot_times_v = tree_.transformPointsJacobianDotTimesV(cache,
        imu_pos, pelvis_ind, 0);

    const double* imu_d = cassie_out.pelvis.vectorNav.linearAcceleration;
    Eigen::Vector3d alpha_imu;
    alpha_imu << imu_d[0], imu_d[1], imu_d[2];

    const double EPS = 0.0000001;
    /* Mathematical program - double contact */
    MathematicalProgram quadprog;
    auto ddq = quadprog.NewContinuousVariables(M.rows(), "ddq");
    auto lambda_b = quadprog.NewContinuousVariables(2, "lambda_b");
    auto lambda_cl = quadprog.NewContinuousVariables(6, "lambda_cl");
    auto lambda_cr = quadprog.NewContinuousVariables(6, "lambda_cr");
    auto eps_cl = quadprog.NewContinuousVariables(6, "eps_cl");
    auto eps_cr = quadprog.NewContinuousVariables(6, "eps_cr");
    auto eps_imu = quadprog.NewContinuousVariables(3, "eps_imu");

    /* constraints */
    quadprog.AddLinearEqualityConstraint(Jb, -1*Jb_dot_times_v, ddq);
    Eigen::MatrixXd CL_coeff(Jcl.rows(), Jcl.cols() + 6);
    CL_coeff << Jcl, Eigen::MatrixXd::Identity(6, 6);
    
    quadprog.AddLinearEqualityConstraint(CL_coeff, -1*Jcl_dot_times_v, {ddq, eps_cl});
    Eigen::MatrixXd CR_coeff(Jcr.rows(), Jcr.cols() + 6);
    CR_coeff << Jcr, Eigen::MatrixXd::Identity(6, 6);
    quadprog.AddLinearEqualityConstraint(CR_coeff, -1*Jcr_dot_times_v, {ddq, eps_cr});
    
    Eigen::MatrixXd IMU_coeff(J_imu.rows(), J_imu.cols() + 3);
    IMU_coeff << J_imu, Eigen::MatrixXd::Identity(3, 3);
    quadprog.AddLinearEqualityConstraint(IMU_coeff, -1*J_imu_dot_times_v + alpha_imu, {ddq, eps_imu});

    quadprog.AddLinearConstraint(Eigen::MatrixXd::Identity(6, 6), -EPS, EPS, eps_cl);
    quadprog.AddLinearConstraint(Eigen::MatrixXd::Identity(6, 6), -EPS, EPS, eps_cr);
    quadprog.AddLinearConstraint(Eigen::MatrixXd::Identity(3, 3), -EPS, EPS, eps_imu);

    /* cost */
    int A_cols = M.cols() + Jb.rows() + Jcl.rows() + Jcr.rows();
    Eigen::MatrixXd cost_A(M.rows(), A_cols);
    cost_A << M, -1*Jb.transpose(), -1*Jcl.transpose(), -1*Jcr.transpose();
    Eigen::VectorXd cost_b(M.rows());
    cost_b = B*u - C;

    /* Debug info */
    /* Using eigen values for debug - remove later */
    Eigen::MatrixXd aa = 2*cost_A.transpose()*cost_A + 0.00001*Eigen::MatrixXd::Identity(A_cols, A_cols);
    Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(aa);
    if (eigensolver.info() != Eigen::Success) {
      cout << "Error in eigen solver!" << endl;
      cout << "cassie_state->positions: \n" << cassie_state->GetPositions() << endl;
      cout << "cassie_state->velocities: \n" << cassie_state->GetVelocities() << endl;
      KinematicsCache<double> k_cache = tree_.doKinematics(cassie_state->GetPositions(), cassie_state->GetVelocities(), true);
      cout << "M: \n" << M << endl;
      cout << "M_old: \n" << tree_.massMatrix(k_cache) << endl;
      return EventStatus::Succeeded();
    }
    quadprog.AddQuadraticCost(2*cost_A.transpose()*cost_A + 0.00001*Eigen::MatrixXd::Identity(A_cols, A_cols),
        -2*cost_A.transpose()*cost_b, {ddq, lambda_b, lambda_cl, lambda_cr});

    const drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(quadprog);
    if (!result.is_success()) {
      std::cout << "Error (double stance): " << result.get_solution_result() << endl;
    }
    cout << "Optimal cost: " << result.get_optimal_cost() + cost_b.transpose()*cost_b << endl;
    /* for (int i = 0; i < quadprog.num_vars(); ++i) { */
    /*   std::cout << quadprog.decision_variable(i).get_name() << " = " */
    /*     << result.GetSolution(quadprog.decision_variable(i)) << "\n"; */
    /* } */

    /* Mathematical program - left contact*/
    MathematicalProgram quadprog_left;
    ddq = quadprog_left.NewContinuousVariables(M.rows(), "ddq");
    lambda_b = quadprog_left.NewContinuousVariables(2, "lambda_b");
    lambda_cl = quadprog_left.NewContinuousVariables(6, "lambda_cl");
    eps_cl = quadprog_left.NewContinuousVariables(6, "eps_cl");
    eps_imu = quadprog_left.NewContinuousVariables(3, "eps_imu");

    /* constraints */
    quadprog_left.AddLinearEqualityConstraint(Jb, -1*Jb_dot_times_v, ddq);
    quadprog_left.AddLinearEqualityConstraint(CL_coeff, -1*Jcl_dot_times_v, {ddq, eps_cl});
    quadprog_left.AddLinearEqualityConstraint(IMU_coeff, -1*J_imu_dot_times_v + alpha_imu, {ddq, eps_imu});

    quadprog_left.AddLinearConstraint(Eigen::MatrixXd::Identity(6, 6), -EPS, EPS, eps_cl);
    quadprog_left.AddLinearConstraint(Eigen::MatrixXd::Identity(3, 3), -EPS, EPS, eps_imu);

    /* cost */
    A_cols = M.cols() + Jb.rows() + Jcl.rows();
    Eigen::MatrixXd cost_A_left(M.rows(), A_cols);
    cost_A_left << M, -1*Jb.transpose(), -1*Jcl.transpose();
    cost_b = B*u - C;

    quadprog_left.AddQuadraticCost(2*cost_A_left.transpose()*cost_A_left + 0.00001*Eigen::MatrixXd::Identity(A_cols, A_cols),
        -2*cost_A_left.transpose()*cost_b, {ddq, lambda_b, lambda_cl});

    const drake::solvers::MathematicalProgramResult result_left = drake::solvers::Solve(quadprog_left);
    if (!result_left.is_success()) {
      std::cout << "Error (left stance): " << result_left.get_solution_result() << endl;
    }
    cout << "Optimal cost: " << result_left.get_optimal_cost() + cost_b.transpose()*cost_b << endl;
    /* for (int i = 0; i < quadprog_left.num_vars(); ++i) { */
    /*   std::cout << quadprog_left.decision_variable(i).get_name() << " = " */
    /*     << result_left.GetSolution(quadprog_left.decision_variable(i)) << "\n"; */
    /* } */

    /* Mathematical program - right contact */
    MathematicalProgram quadprog_right;
    ddq = quadprog_right.NewContinuousVariables(M.rows(), "ddq");
    lambda_b = quadprog_right.NewContinuousVariables(2, "lambda_b");
    lambda_cr = quadprog_right.NewContinuousVariables(6, "lambda_cr");
    eps_cr = quadprog_right.NewContinuousVariables(6, "eps_cr");
    eps_imu = quadprog_right.NewContinuousVariables(3, "eps_imu");

    /* constraints */
    quadprog_right.AddLinearEqualityConstraint(Jb, -1*Jb_dot_times_v, ddq);
    quadprog_right.AddLinearEqualityConstraint(CR_coeff, -1*Jcr_dot_times_v, {ddq, eps_cr});
    quadprog_right.AddLinearEqualityConstraint(IMU_coeff, -1*J_imu_dot_times_v + alpha_imu, {ddq, eps_imu});

    quadprog_right.AddLinearConstraint(Eigen::MatrixXd::Identity(6, 6), -EPS, EPS, eps_cr);
    quadprog_right.AddLinearConstraint(Eigen::MatrixXd::Identity(3, 3), -EPS, EPS, eps_imu);

    /* cost */
    A_cols = M.cols() + Jb.rows() + Jcr.rows();
    Eigen::MatrixXd cost_A_right(M.rows(), A_cols);
    cost_A_right << M, -1*Jb.transpose(), -1*Jcr.transpose();
    cost_b = B*u - C;

    quadprog_right.AddQuadraticCost(2*cost_A_right.transpose()*cost_A_right + 0.00001*Eigen::MatrixXd::Identity(A_cols, A_cols),
        -2*cost_A_right.transpose()*cost_b, {ddq, lambda_b, lambda_cr});

    const drake::solvers::MathematicalProgramResult result_right = drake::solvers::Solve(quadprog_right);
    if (!result_right.is_success()) {
      std::cout << "Error (double stance): " << result_right.get_solution_result() << endl;
    }
    cout << "Optimal cost: " << result_right.get_optimal_cost() + cost_b.transpose()*cost_b << endl;
    /* for (int i = 0; i < quadprog_right.num_vars(); ++i) { */
    /*   std::cout << quadprog_right.decision_variable(i).get_name() << " = " */
    /*     << result_right.GetSolution(quadprog_right.decision_variable(i)) << "\n"; */
    /* } */


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

    // discrete_state->get_mutable_vector(time_idx_).get_mutable_value() << ...
  }


  return EventStatus::Succeeded();
}


/// Workhorse state estimation function. Given a `cassie_out_t`, compute the
/// esitmated state as an OutputVector
/// Since it needs to map from a struct to a vector, and no assumptions on the
/// ordering of the vector are made, utilizies index maps to make this mapping.
void CassieRbtStateEstimator::CopyStateOut(
  const Context<double>& context, OutputVector<double>* output) const {
  const auto& cassie_out =
    this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();

  // It's necessary for initialization. Might be a better way to initialize?
  auto data = output->get_mutable_data(); // This doesn't affect timestamp value
  data = Eigen::VectorXd::Zero(data.size());

  // Assign the values
  // Copy the robot state excluding floating base
  AssignNonFloatingBaseToOutputVector(output, cassie_out);

  // Floating base coordinates
  if (is_floating_base_) {
    // Assign the values
    auto state_est = context.get_discrete_state(state_idx_).get_value();

    // TODO(yminchen): The name of the joitn name need to be change when we move to MBP
    output->SetPositionAtIndex(positionIndexMap_.at("base_x"),
                               state_est(0));
    output->SetPositionAtIndex(positionIndexMap_.at("base_y"),
                               state_est(1));
    output->SetPositionAtIndex(positionIndexMap_.at("base_z"),
                               state_est(2));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qw"),
                               state_est(3));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qx"),
                               state_est(4));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qy"),
                               state_est(5));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qz"),
                               state_est(6));

    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wx"),
                               state_est(7));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wy"),
                               state_est(8));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wz"),
                               state_est(9));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vx"),
                               state_est(10));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vy"),
                               state_est(11));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vz"),
                               state_est(12));
  }  //  end if(is_floating_base)




  // Testing
  auto state_time = context.get_discrete_state(time_idx_).get_value();
  cout << "  In copyStateOut: lcm_time = " << cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  cout << "  In copyStateOut: state_time = " << state_time << endl;
  cout << "  In copyStateOut: context_time = " << context.get_time() << endl;

  // Testing
  // cout << endl << "****bodies****" << endl;
  // for (int i = 0; i < tree_.get_num_bodies(); i++)
  //   cout << tree_.getBodyOrFrameName(i) << endl;
  // cout << endl << "****actuators****" << endl;
  // for (int i = 0; i < tree_.get_num_actuators(); i++)
  //   cout << tree_.actuators[i].name_ << endl;
  // cout << endl << "****positions****" << endl;
  // for (int i = 0; i < tree_.get_num_positions(); i++)
  //   cout << tree_.get_position_name(i) << endl;
  // cout << endl << "****velocities****" << endl;
  // for (int i = 0; i < tree_.get_num_velocities(); i++)
  // cout << tree_.get_velocity_name(i) << endl;




}

}  // namespace systems
}  // namespace dairlib
