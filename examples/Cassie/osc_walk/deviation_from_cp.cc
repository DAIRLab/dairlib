#include "examples/Cassie/osc_walk/deviation_from_cp.h"

#include <math.h>
#include <string>

#include "examples/Cassie/osc_walk/cp_control_common_func.h"
#include "attic/multibody/rigidbody_utils.h"

#include "drake/math/quaternion.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Quaterniond;

using dairlib::systems::OutputVector;

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace cassie {
namespace osc_walk {

DeviationFromCapturePoint::DeviationFromCapturePoint(
    const RigidBodyTree<double>& tree, int pelvis_idx,
    Vector2d global_target_position, Eigen::Vector2d params_of_no_turning) :
        tree_(tree),
        pelvis_idx_(pelvis_idx),
        global_target_position_(global_target_position),
        params_of_no_turning_(params_of_no_turning) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(2),
                                &DeviationFromCapturePoint::CalcFootPlacement);

  // Foot placement control (Sagital) parameters
  kp_pos_sagital_ = 1.0;
  kd_pos_sagital_ = 0.2;
  vel_max_sagital_ = 1;
  vel_min_sagital_ = -1;  // TODO(yminchen): need to test this

  k_fp_ff_sagital_ = 0.16;  // TODO(yminchen): these are for going forward.
                            // Should have parameters for going backward
  k_fp_fb_sagital_ = 0.04;
  target_pos_offset_ = -0.16;  // Due to steady state error

  // Foot placement control (Lateral) parameters
  kp_pos_lateral_ = 0.5;
  kd_pos_lateral_ = 0.1;
  vel_max_lateral_ = 0.5;
  vel_min_lateral_ = -0.5;

  k_fp_ff_lateral_ = 0.08;
  k_fp_fb_lateral_ = 0.02;
}

void DeviationFromCapturePoint::CalcFootPlacement(const Context<double>& context,
    BasicVector<double>* output) const {
  // Read in current state
  const OutputVector<double>* robot_output = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  // Get center of mass position and velocity
  Vector3d com_pos = tree_.centerOfMass(cache);
  MatrixXd J = tree_.centerOfMassJacobian(cache);
  Vector3d com_vel = J * v;

  // Get proximated heading angle of pelvis
  Vector3d pelvis_heading_vec = tree_.CalcBodyPoseInWorldFrame(
      cache, tree_.get_body(pelvis_idx_)).linear().col(0);
  double approx_pelvis_yaw = atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desired heading direction
  Vector2d global_com_pos_to_target_pos =
      global_target_position_ - com_pos.segment(0, 2);
  double desired_yaw = GetDesiredYawAngle(approx_pelvis_yaw,
      global_com_pos_to_target_pos, params_of_no_turning_);

  // Calculate the current-desired yaw angle difference
  double heading_error = desired_yaw - approx_pelvis_yaw;
  bool pause_walking_position_control =
      (heading_error > M_PI / 2 || heading_error < -M_PI / 2) ? true : false;

  Vector3d delta_CP_sagital_3D_global(0, 0, 0);
  Vector3d delta_CP_lateral_3D_global(0, 0, 0);

  // Apply walking speed control only when the robot is facing the target
  // position.
  if (!pause_walking_position_control) {
    // Extract quaternion from floating base position
    Quaterniond Quat(q(3), q(4), q(5), q(6));
    Quaterniond Quat_conj = Quat.conjugate();
    Vector4d quat(q.segment(3,4));
    Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(),
                       Quat_conj.y(), Quat_conj.z());

    // Calculate local target position and com velocity
    Vector3d global_com_pos_to_target_pos_3d;
    global_com_pos_to_target_pos_3d << global_com_pos_to_target_pos, 0;
    Vector3d local_com_pos_to_target_pos =
        drake::math::quatRotateVec(quad_conj, global_com_pos_to_target_pos_3d);
    Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

    //////////////////// Sagital ////////////////////
    // Position Control
    double com_vel_sagital = local_com_vel(0);
    double des_sagital_vel =
        kp_pos_sagital_ * (local_com_pos_to_target_pos(0) + target_pos_offset_) +
        kd_pos_sagital_ * (-com_vel_sagital);
    des_sagital_vel = std::min(vel_max_sagital_,
                               std::max(vel_min_sagital_, des_sagital_vel));
    // Velocity control
    double delta_CP_sagital =
        - k_fp_ff_sagital_ * des_sagital_vel
        - k_fp_fb_sagital_ * (des_sagital_vel - com_vel_sagital);
    Vector3d delta_CP_sagital_3D_local(delta_CP_sagital, 0, 0);
    delta_CP_sagital_3D_global = drake::math::quatRotateVec(
                                   quat, delta_CP_sagital_3D_local);

    //////////////////// Lateral ////////////////////  TODO(yminchen): tune this
    // Position Control
    double com_vel_lateral = local_com_vel(1);
    double des_lateral_vel = kp_pos_lateral_ * (local_com_pos_to_target_pos(1)) +
                             kd_pos_lateral_ * (-com_vel_lateral);
    des_lateral_vel = std::min(vel_max_lateral_,
                               std::max(vel_min_lateral_, des_lateral_vel));
    // Velocity control
    double delta_CP_lateral =
        -k_fp_ff_lateral_ * des_lateral_vel
        - k_fp_fb_lateral_ * (des_lateral_vel - com_vel_lateral);
    Vector3d delta_CP_lateral_3D_local(0, delta_CP_lateral, 0);
    delta_CP_lateral_3D_global = drake::math::quatRotateVec(
                                   quat, delta_CP_lateral_3D_local);
  }

  // Assign foot placement
  output->get_mutable_value() =
      (delta_CP_sagital_3D_global + delta_CP_lateral_3D_global).head(2);
}

}  // namespace osc_walk
}  // namespace cassie
}  // namespace dairlib


