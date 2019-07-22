#include "examples/Cassie/osc_walking_control/foot_placement_control.h"

#include <math.h>
#include <string>

#include "examples/Cassie/osc_walking_control/control_utils.h"
#include "examples/Cassie/osc_walking_control/cp_control_common_func.h"
#include "common/math_utils.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using dairlib::systems::OutputVector;

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib {
namespace cassie {
namespace osc_walking_control {

FootPlacementControl::FootPlacementControl(RigidBodyTree<double> * tree,
    int pelvis_idx,
    Vector2d global_target_position, double circle_radius_of_no_turning) :
  tree_(tree),
  pelvis_idx_(pelvis_idx),
  global_target_position_(global_target_position),
  circle_radius_of_no_turning_(circle_radius_of_no_turning) {
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree->get_num_positions(),
                  tree->get_num_velocities(),
                  tree->get_num_actuators())).get_index();
  this->DeclareVectorOutputPort(BasicVector<double>(2),
                                &FootPlacementControl::CalcFootPlacement);

  // Foot placement control (Sagital) parameters
  kp_pos_sagital_ = 1.0;
  kd_pos_sagital_ = 0.2;
  vel_max_sagital_ = 1;
  vel_min_sagital_ = -1;  // TODO(yminchen): need to test this

  k_fp_ff_sagital_ = 0.16;  // TODO(yminchen): these are for going forward.
                            // Should have parameters for going backward
  k_fp_fb_sagital_ = 0.04;
  target_position_offset_ = -0.16;  // Due to steady state error

  // Foot placement control (Lateral) parameters
  kp_pos_lateral_ = 0.5;
  kd_pos_lateral_ = 0.1;
  vel_max_lateral_ = 0.5;
  vel_min_lateral_ = -0.5;

  k_fp_ff_lateral_ = 0.08;
  k_fp_fb_lateral_ = 0.02;
}

void FootPlacementControl::CalcFootPlacement(
    const Context<double>& context,
    BasicVector<double>* output) const {
  // Read in current state
  const OutputVector<double>* robotOutput = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  q.segment(3, 4) = NormalizeQuaternion(q.segment(3, 4));
  cache.initialize(q);
  tree_->doKinematics(cache);

  // Get center of mass position and velocity
  Vector3d CoM = tree_->centerOfMass(cache);
  MatrixXd J = tree_->centerOfMassJacobian(cache);
  Vector3d dCoM = J * v;

  // Extract quaternion from floating base position
  Eigen::Quaterniond floating_base_quat(q(3), q(4), q(5), q(6));

  // Get proximated heading angle of pelvis
  Vector3d pelvis_heading_vec = tree_->CalcBodyPoseInWorldFrame(
      cache, tree_->get_body(pelvis_idx_)).linear().col(0);
  double approx_pelvis_yaw = atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desried heading direction
  Vector2d global_CoM_to_target_pos =
      global_target_position_ - CoM.segment(0, 2);
  double desried_heading_pos = GetDesiredHeadingPos(approx_pelvis_yaw,
      global_CoM_to_target_pos, circle_radius_of_no_turning_);

  // Walking control
  double heading_error = desried_heading_pos - approx_pelvis_yaw;
  bool isPauseWalkingPositionControl =
      (heading_error > M_PI / 2 || heading_error < -M_PI / 2) ? true : false;

  Vector3d delta_CP_sagital_3D_global(0, 0, 0);
  Vector3d delta_CP_lateral_3D_global(0, 0, 0);
  if (!isPauseWalkingPositionControl) {

    Vector3d global_CoM_to_target_pos_3d;
    global_CoM_to_target_pos_3d << global_CoM_to_target_pos, 0;
    Vector3d local_CoM_to_target_pos = RotateVecFromGlobalToLocalByQuaternion(
                                         floating_base_quat,
                                         global_CoM_to_target_pos_3d);
    Vector3d local_dCoM = RotateVecFromGlobalToLocalByQuaternion(
                            floating_base_quat, dCoM);
    //////////////////// Sagital ////////////////////
    // Position Control
    double dCoM_sagital = local_dCoM(0);
    double des_sagital_vel =
        kp_pos_sagital_ * (local_CoM_to_target_pos(0) + target_position_offset_) +
        kd_pos_sagital_ * (-dCoM_sagital);
    des_sagital_vel = std::min(vel_max_sagital_,
                               std::max(vel_min_sagital_, des_sagital_vel));
    // Velocity control
    double delta_CP_sagital =
        - k_fp_ff_sagital_ * des_sagital_vel
        - k_fp_fb_sagital_ * (des_sagital_vel - dCoM_sagital);
    Vector3d delta_CP_sagital_3D_local(delta_CP_sagital, 0, 0);
    delta_CP_sagital_3D_global = RotateVecFromLocalToGlobalByQuaternion(
                                   floating_base_quat,
                                   delta_CP_sagital_3D_local);

    //////////////////// Lateral ////////////////////  TODO(yminchen): tune this
    // Position Control
    double dCoM_lateral = local_dCoM(1);
    double des_lateral_vel = kp_pos_lateral_ * (local_CoM_to_target_pos(1)) +
                             kd_pos_lateral_ * (-dCoM_lateral);
    des_lateral_vel = std::min(vel_max_lateral_,
                               std::max(vel_min_lateral_, des_lateral_vel));
    // Velocity control
    double delta_CP_lateral =
        -k_fp_ff_lateral_ * des_lateral_vel
        - k_fp_fb_lateral_ * (des_lateral_vel - dCoM_lateral);
    Vector3d delta_CP_lateral_3D_local(0, delta_CP_lateral, 0);
    delta_CP_lateral_3D_global = RotateVecFromLocalToGlobalByQuaternion(
                                   floating_base_quat,
                                   delta_CP_lateral_3D_local);
  }

  // Assign foot placement
  Vector2d global_delta_CP_sagital_and_lateral(
      delta_CP_sagital_3D_global(0) + delta_CP_lateral_3D_global(0),
      delta_CP_sagital_3D_global(1) + delta_CP_lateral_3D_global(1));
  output->get_mutable_value() = global_delta_CP_sagital_and_lateral;
}

}  // namespace osc_walking_control
}  // namespace cassie
}  // namespace dairlib


