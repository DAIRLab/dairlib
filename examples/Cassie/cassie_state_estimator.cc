#include "examples/Cassie/cassie_state_estimator.h"

#include <math.h>

#include <chrono>
#include <fstream>
#include <utility>

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
using multibody::MakeJointPositionOffsetFromMap;
using systems::OutputVector;

static const int SPACE_DIM = 3;

CassieStateEstimator::CassieStateEstimator(
    const MultibodyPlant<double>& plant,
    const KinematicEvaluatorSet<double>* fourbar_evaluator,
    const KinematicEvaluatorSet<double>* left_contact_evaluator,
    const KinematicEvaluatorSet<double>* right_contact_evaluator,
    std::map<std::string, double> joint_offset_map,
    bool test_with_ground_truth_state, bool print_info_to_terminal,
    int hardware_test_mode, double contact_force_threshold)
    : plant_(plant),
      fourbar_evaluator_(fourbar_evaluator),
      left_contact_evaluator_(left_contact_evaluator),
      right_contact_evaluator_(right_contact_evaluator),
      world_(plant_.world_frame()),
      is_floating_base_(multibody::HasQuaternion(plant)),
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
      hardware_test_mode_(hardware_test_mode),
      contact_force_threshold_(contact_force_threshold),
      joint_offsets_(MakeJointPositionOffsetFromMap(plant, joint_offset_map)){
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
  estimated_state_output_port_ =
      this->DeclareVectorOutputPort("x, u, t",
                                    OutputVector<double>(n_q_, n_v_, n_u_),
                                    &CassieStateEstimator::CopyStateOut)
          .get_index();


  // Initialize index maps
  actuator_idx_map_ = multibody::MakeNameToActuatorsMap(plant);
  position_idx_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_idx_map_ = multibody::MakeNameToVelocitiesMap(plant);

  if (is_floating_base_) {
    contact_output_port_ =
        this->DeclareAbstractOutputPort("lcmt_contact",
                                        &CassieStateEstimator::CopyContact)
            .get_index();
    contact_forces_output_port_ =
        this->DeclareAbstractOutputPort(
                "lcmt_contact_results_for_viz",
                &CassieStateEstimator::CopyEstimatedContactForces)
            .get_index();

    // Middle point between the front and the rear contact points
    front_contact_disp_ = LeftToeFront(plant).first;
    rear_contact_disp_ = LeftToeRear(plant).first;
    mid_contact_disp_ = (front_contact_disp_ + rear_contact_disp_) / 2;

    // Declare input port receiving robot's state (simulation ground truth
    // state)
    if (test_with_ground_truth_state_) {
      state_input_port_ =
          this->DeclareVectorInputPort("x, u, t",
                                       OutputVector<double>(n_q_, n_v_, n_u_))
              .get_index();
    }

    pose_covariance_output_port_ = this->DeclareVectorOutputPort(
            "cov", 36, &CassieStateEstimator::CopyPoseCovarianceOut)
        .get_index();

    // a state which stores previous timestamp
    time_idx_ = DeclareDiscreteState(VectorXd::Zero(1));

    // Joint selection matrices initialization
    joint_selection_matrices.emplace_back(MatrixXd::Zero(n_v_, n_v_));
    joint_selection_matrices.emplace_back(MatrixXd::Zero(n_v_, n_v_));
    vector<string> leg_names = {"left, right"};
    for (const auto& joint_name : velocity_idx_map_) {
      if (joint_name.first.find("left") != std::string::npos) {
        joint_selection_matrices[0](joint_name.second, joint_name.second) = 1;
      }
      if (joint_name.first.find("right") != std::string::npos) {
        joint_selection_matrices[1](joint_name.second, joint_name.second) = 1;
      }
    }

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
    ekf_idx_ = DeclareAbstractState(*AbstractValue::Make<inekf::InEKF>(value));

    // 3. state for previous imu value
    // Measured accelrometer should point toward positive z when the robot rests
    // on the ground.
    VectorXd init_prev_imu_value = VectorXd::Zero(6);
    init_prev_imu_value << 0, 0, 0, 0, 0, 9.81;
    prev_imu_idx_ = DeclareDiscreteState(init_prev_imu_value);

    // states related to contact estimation
    contact_idx_ = DeclareDiscreteState(VectorXd::Zero(num_contacts_));
    contact_forces_idx_ =
        DeclareDiscreteState(VectorXd::Zero(num_contacts_ * SPACE_DIM));

    previous_velocity_idx_ = DeclareDiscreteState(VectorXd::Zero(n_v_, 1));
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
      atan2(rod_on_heel_springs_[0].first(1), rod_on_heel_springs_[0].first(0));
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
    // We pick it by checking the x component of the solutions. The correct one
    // could be closer to 1 (the rest spring is (1,0,0) in local frame)
    Vector3d r_sol_wrt_heel_base =
        (sol_2_wrt_heel_base(0) >= sol_1_wrt_heel_base(0))
            ? sol_2_wrt_heel_base
            : sol_1_wrt_heel_base;
    // Get the heel spring deflection direction and magnitude
    const Vector3d spring_rest_dir_wrt_spring_base(1, 0, 0);
    double heel_spring_angle = acos(
        r_sol_wrt_heel_base.dot(spring_rest_dir_wrt_spring_base) /
        (r_sol_wrt_heel_base.norm() * spring_rest_dir_wrt_spring_base.norm()));
    Vector3d r_rest_dir_cross_r_hs_to_sol =
        spring_rest_dir_wrt_spring_base.cross(r_sol_wrt_heel_base);
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
  VectorXd q = output->GetPositions() + joint_offsets_;
  output->SetPositions(q);

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

/// EstimateContactFromSprings(). Conservative estimation.
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
    const systems::OutputVector<double>& output, int* left_contact,
    int* right_contact) const {
  // Initialize
  *left_contact = 0;
  *right_contact = 0;

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
                              left_heel_spring < ankle_spring_threshold_ekf_);
  bool right_contact_spring = (right_knee_spring < knee_spring_threshold_ekf_ &&
                               right_heel_spring < ankle_spring_threshold_ekf_);

  // conflicting contact measurements can cause the state estimator to jump
  // due to kinematic differences, backlash, etc, so we only want to trust
  // one at a time
  /*
  if (left_contact_spring && right_contact_spring) {
    left_contact_spring = left_knee_spring < right_knee_spring;
    right_contact_spring = !left_contact_spring;
  } */

  // Determine contacts based on both spring deflation and QP cost
  if (left_contact_spring) {
    *left_contact = 1;
  }
  if (right_contact_spring) {
    *right_contact = 1;
  }

  if (print_info_to_terminal_) {
    cout << "left/right knee spring, threshold = " << left_knee_spring << ", "
         << right_knee_spring << ", " << knee_spring_threshold_ekf_ << endl;
    cout << "left/right heel spring, threshold = " << left_heel_spring << ", "
         << right_heel_spring << ", " << ankle_spring_threshold_ekf_ << endl;
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
    const systems::OutputVector<double>& output, int* left_contact,
    int* right_contact) const {
  // Initialize
  *left_contact = 0;
  *right_contact = 0;

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
                              left_heel_spring < ankle_spring_threshold_ctrl_);
  bool right_contact_spring =
      (right_knee_spring < knee_spring_threshold_ctrl_ ||
       right_heel_spring < ankle_spring_threshold_ctrl_);

  // Determine contacts based on both spring deflation and QP cost
  if (left_contact_spring) {
    *left_contact = 1;
  }
  if (right_contact_spring) {
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
  const auto& ekf_state = ekf.getState();
  const auto& R_WB = ekf_state.getRotation();

  VectorXd estimated_fb_state(13);
  Vector3d r_imu_to_pelvis_global = R_WB * (-imu_pos_);
  // Rotational position
  Quaterniond q(R_WB);
  q.normalize();
  estimated_fb_state[0] = q.w();
  estimated_fb_state.segment<3>(1) = q.vec();
  // Translational position
  estimated_fb_state.segment<3>(4) =
     ekf_state.getPosition() + r_imu_to_pelvis_global;
  // Rotational velocity
  Vector3d omega_global =
     R_WB * imu_measurement.head(3);
  estimated_fb_state.segment<3>(7) = omega_global;
  // Translational velocity
  estimated_fb_state.tail(3) =
      ekf_state.getVelocity() + omega_global.cross(r_imu_to_pelvis_global);

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

  VectorXd lambda_est = VectorXd::Zero(num_contacts_ * 3);
  if (test_with_ground_truth_state_) {
    EstimateContactForEkf(output_gt, &left_contact, &right_contact);
  } else {
    EstimateContactForEkf(filtered_output, &left_contact, &right_contact);
    // EstimateContactForces(context, filtered_output, lambda_est, left_contact,
    //                       right_contact);
  }
  state->get_mutable_discrete_state(contact_forces_idx_).get_mutable_value()
      << lambda_est;

  // Override hardware_test_mode_ if test mode is 2 and we detect contact
  // Useful for preventing drift when the feet are not fully in contact - i.e
  // when running the PD controller with external support
  if (left_contact && right_contact && hardware_test_mode_ == 2) {
    hardware_test_mode_ = -1;
    if (print_info_to_terminal_) {
      cout << "Switch to test_mode -1 \n";
    }
  }

  // Test mode needed for hardware experiment
  // mode #0 assumes the feet are always on the ground
  // mode #1 assumes the feet are always in the air
  if (hardware_test_mode_ == 0 || hardware_test_mode_ == 2) {
    left_contact = 1;
    right_contact = 1;

    if ((*counter_for_testing_) % 5000 == 0 and print_info_to_terminal_) {
      cout << "pos = " << ekf.getState().getPosition().transpose() << endl;
    }
    *counter_for_testing_ = *counter_for_testing_ + 1;
  } else if (hardware_test_mode_ == 1) {
    left_contact = 0;
    right_contact = 0;
  }

  // Assign contacts
  state->get_mutable_discrete_state()
          .get_mutable_vector(contact_idx_)
          .get_mutable_value()
      << left_contact,
      right_contact;

  std::vector<std::pair<int, bool>> contacts;
  // TODO(yangwill): Decide whether to use both contacts per foot or just one.
  // Possibly leave it as an option to the state estimator
  contacts.push_back(std::pair<int, bool>(0, left_contact));
//  contacts.push_back(std::pair<int, bool>(1, left_contact));
  contacts.push_back(std::pair<int, bool>(2, right_contact));
//  contacts.push_back(std::pair<int, bool>(3, right_contact));
  ekf.setContacts(contacts);

  // Step 4 - EKF (measurement step)
  plant_.SetPositionsAndVelocities(context_.get(), filtered_output.GetState());

  // rotation part of pose and covariance is unused in EKF
  Eigen::Matrix4d rear_toe_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d front_toe_pose = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 6> rear_covariance = MatrixXd::Identity(6, 6);
  Eigen::Matrix<double, 6, 6> front_covariance = MatrixXd::Identity(6, 6);

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
    rear_toe_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
    rear_toe_pose.block<3, 1>(0, 3) = toe_pos - imu_pos_;
    plant_.CalcPointsPositions(*context_, *toe_frames_[i], front_contact_disp_,
                               pelvis_frame_, &toe_pos);
    front_toe_pose.block<3, 3>(0, 0) = Matrix3d::Identity();
    front_toe_pose.block<3, 1>(0, 3) = toe_pos - imu_pos_;

    if (print_info_to_terminal_) {
      // Print for debugging
      // cout << "Pose: " << endl;
      // cout << rear_toe_pose.block<3, 1>(0, 3).transpose() << endl;
    }

    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, *toe_frames_[i], rear_contact_disp_,
        pelvis_frame_, pelvis_frame_, &J);
    MatrixXd J_wrt_joints = J.block(0, 6, 3, 16);
    rear_covariance.block<3, 3>(3, 3) =
        J_wrt_joints * cov_w_ * J_wrt_joints.transpose();
    inekf::Kinematics rear_frame(2 * i, rear_toe_pose, rear_covariance);
    measured_kinematics.push_back(rear_frame);
    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, *toe_frames_[i],
        front_contact_disp_, pelvis_frame_, pelvis_frame_, &J);
    J_wrt_joints = J.block(0, 6, 3, 16);
    front_covariance.block<3, 3>(3, 3) =
        J_wrt_joints * cov_w_ * J_wrt_joints.transpose();
    inekf::Kinematics front_frame(2 * i + 1, front_toe_pose, front_covariance);
    measured_kinematics.push_back(front_frame);

    if (print_info_to_terminal_) {
      cout << "covariance.block<3, 3>(3, 3) = \n"
           << rear_covariance.block<3, 3>(3, 3) << endl;
    }
  }
  //  std::ofstream outfile;
  //  outfile.open("../ekf_error.txt", std::ios_base::app);
  //  outfile << current_time << ", ";
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

void CassieStateEstimator::CopyContact(
    const Context<double>& context, dairlib::lcmt_contact* contact_msg) const {
  contact_msg->utime = context.get_time() * 1e6;
  contact_msg->num_contacts = num_contacts_;
  contact_msg->contact_names.resize(num_contacts_);
  contact_msg->contact.resize(num_contacts_);
  for (int i = 0; i < num_contacts_; i++) {
    contact_msg->contact_names[i] = contact_names_[i];
    contact_msg->contact[i] =
        context.get_discrete_state(contact_idx_).get_value()[i];
  }
}

void CassieStateEstimator::CopyEstimatedContactForces(
    const Context<double>& context,
    drake::lcmt_contact_results_for_viz* contact_msg) const {
  // TODO (yangwill) fuse residual based contact estimation with heel spring
  // deflection and phase of gait cycle
  contact_msg->timestamp = context.get_time() * 1e6;
  contact_msg->num_point_pair_contacts = num_contacts_;
  contact_msg->num_hydroelastic_contacts = 0;
  contact_msg->point_pair_contact_info.clear();
  for (int i = 0; i < num_contacts_; i++) {
    auto contact_info = drake::lcmt_point_pair_contact_info_for_viz();
    contact_info.timestamp = contact_msg->timestamp;
    contact_info.body1_name = toe_frames_[i]->body().name();
    contact_info.body2_name = world_.name();
    memcpy(contact_info.contact_force,
           context.get_discrete_state(contact_forces_idx_)
               .get_value()
               .segment(i * SPACE_DIM, (i + 1) * SPACE_DIM)
               .data(),
           SPACE_DIM * sizeof(double));
    contact_msg->point_pair_contact_info.push_back(contact_info);
  }
}

void CassieStateEstimator::CopyPoseCovarianceOut(
    const Context<double> &context, BasicVector<double> *cov) const {
  const auto& ekf = context.get_abstract_state<inekf::InEKF>(ekf_idx_);
  Eigen::Matrix<double, 6, 6> pos_rot_cov = Eigen::Matrix<double, 6, 6>::Zero();
  const auto& P = ekf.getState().getP();
  pos_rot_cov.bottomRightCorner<3,3>() = P.topRightCorner<3,3>();
  pos_rot_cov.topRightCorner<3,3>() = P.block<3,3>(0, 6).transpose();
  pos_rot_cov.topLeftCorner<3,3>() = P.block<3,3>(6,6);
  pos_rot_cov.bottomLeftCorner<3,3>() = P.block<3,3>(0, 6);
  cov->get_mutable_value() =
      Eigen::Map<VectorXd>(pos_rot_cov.data(), pos_rot_cov.size());
}

void CassieStateEstimator::setPreviousTime(Context<double>* context,
                                           double time) const {
  context->get_mutable_discrete_state(time_idx_).get_mutable_value() << time;
}
void CassieStateEstimator::setInitialPelvisPose(Context<double>* context,
                                                Eigen::Vector4d quat,
                                                Vector3d pelvis_pos,
                                                Vector3d pelvis_vel) const {
  context->get_mutable_discrete_state(fb_state_idx_).get_mutable_value().head(7)
      << quat,
      pelvis_pos;

  // Update EKF state
  // The imu's and pelvis's frames share the same rotation.
  Matrix3d imu_rot_mat =
      Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();
  Vector3d imu_position = pelvis_pos + imu_rot_mat * imu_pos_;
  Vector3d imu_velocity = pelvis_vel;  // assuming omega = 0
  auto& filter = context->get_mutable_abstract_state<inekf::InEKF>(ekf_idx_);
  auto state = filter.getState();
  state.setPosition(imu_position);
  state.setRotation(imu_rot_mat);
  state.setVelocity(imu_velocity);
  filter.setState(state);
  if (print_info_to_terminal_) {
    cout << "Set initial IMU position to \n"
         << filter.getState().getPosition().transpose() << endl;
    cout << "Set initial IMU rotation to \n"
         << filter.getState().getRotation() << endl;
  }
}
void CassieStateEstimator::setPreviousImuMeasurement(
    Context<double>* context, const VectorXd& imu_value) const {
  context->get_mutable_discrete_state(prev_imu_idx_).get_mutable_value()
      << imu_value;
}
void CassieStateEstimator::EstimateContactForces(
    const Context<double>& context, const systems::OutputVector<double>& output,
    VectorXd& lambda, int& left_contact, int& right_contact) const {

  // TODO(yangwill) add a discrete time filter to the force estimate
  VectorXd v_prev =
      context.get_discrete_state(previous_velocity_idx_).get_value();
  plant_.SetPositionsAndVelocities(context_.get(), output.GetState());
  MatrixXd M = MatrixXd(n_v_, n_v_);
  plant_.CalcMassMatrix(*context_, &M);
  VectorXd C(n_v_);
  plant_.CalcBiasTerm(*context_, &C);
  MatrixXd B = plant_.MakeActuationMatrix();
  drake::multibody::MultibodyForces<double> f_app(plant_);
  plant_.CalcForceElementsContribution(*context_, &f_app);
  double beta = 0.01;
  double gamma = 0.01;

  VectorXd v = output.GetVelocities();
  VectorXd g = plant_.CalcGravityGeneralizedForces(*context_);
  VectorXd tau_d = gamma * beta * M * v_prev -
                   (1 - gamma) * (beta * M * v + B * output.GetEfforts() + C -
                                  g + f_app.generalized_forces());

  // Simplifying to 2 feet contacts, might need to change it to two contacts per
  // foot and sum them up
  for (int leg = 0; leg < num_contacts_; ++leg) {
    MatrixXd J_contact(3, n_v_);
    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV, *toe_frames_[leg],
        VectorXd::Zero(3), world_, world_, &J_contact);
    lambda.segment(3 * leg, SPACE_DIM) =
        (joint_selection_matrices[leg] * J_contact.transpose())
            .colPivHouseholderQr()
            .solve(joint_selection_matrices[leg] * tau_d)
            .transpose();
  }
  double contact_force_threshold = 70;
  if (!(lambda[2] > 2 * contact_force_threshold) !=
      !(lambda[5] > 2 * contact_force_threshold)) {
    left_contact = lambda[2] / (2 * contact_force_threshold);
    right_contact = lambda[5] / (2 * contact_force_threshold);
  } else {
    left_contact = lambda[2] / contact_force_threshold;
    right_contact = lambda[5] / contact_force_threshold;
  }
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

    if (is_floating_base_) {
      UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback callback =
          [this](const Context<double>& c,
                 const UnrestrictedUpdateEvent<double>&,
                 drake::systems::State<double>* s) { this->Update(c, s); };

      auto& uu_events = events->get_mutable_unrestricted_update_events();
      uu_events.AddEvent(UnrestrictedUpdateEvent<double>(
          drake::systems::TriggerType::kTimed, callback));
    } else {
      *time = INFINITY;
    }
  }
}

}  // namespace systems
}  // namespace dairlib
