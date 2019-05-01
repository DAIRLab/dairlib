#define WORLDFRAME_ID 0

#include "systems/controllers/endeffector_position_controller.h"

namespace dairlib{
namespace systems{

 // Remember to use std::move on the rigid body tree argument.
EndEffectorPositionController::EndEffectorPositionController(
    RigidBodyTree<double>& tree, int ee_frame_id,
    Eigen::Vector3d ee_contact_frame, int num_joints, int k_p, int k_omega)
    : tree_local(tree){
 	// Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
 	joint_position_measured_port = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints)).get_index();
 	endpoint_position_commanded_port = this->DeclareVectorInputPort(
      "endpoint_position_commanded", BasicVector<double>(3)).get_index();
  endpoint_orientation_commanded_port = this->DeclareVectorInputPort(
      "endpoint_orientation_commanded", BasicVector<double>(4)).get_index();

  endpoint_position_cmd_output_port = this->DeclareVectorOutputPort(
      BasicVector<double>(6), &EndEffectorPositionController::CalcOutputTwist).get_index();

  // The coordinates for the end effector with respect to the last joint.
  // Eventually passed into transformPointsJacobian()
  this->ee_contact_frame = ee_contact_frame;
  // frame ID of the joint connected to the end effector
  this->ee_frame_id = ee_frame_id;
  this->k_p = k_p;
  this->k_omega = k_omega;
}

void EndEffectorPositionController::CalcOutputTwist(
    const Context<double> &context, BasicVector<double>* output) const {
  VectorX<double> q_actual = this->
                             EvalVectorInput(context, joint_position_measured_port)->
                             CopyToVector();

  VectorX<double> x_desired = this->
                              EvalVectorInput(context, endpoint_position_commanded_port)->
                              CopyToVector();

  VectorX<double> orientation_desired = this->
                                        EvalVectorInput(context, endpoint_orientation_commanded_port)->
                                        CopyToVector();

  KinematicsCache<double> cache = tree_local.doKinematics(q_actual);

  MatrixXd x_actual = tree_local.transformPoints(cache, ee_contact_frame,
                                                  ee_frame_id, WORLDFRAME_ID);
  MatrixXd diff = k_p * (x_desired - x_actual);

  // Quaternion for rotation from base to end effector
  MatrixXd quat_tmp= tree_local.relativeQuaternion(cache, ee_frame_id, WORLDFRAME_ID);

  Eigen::Quaternion<double> quat_n_a = Eigen::Quaternion<double>(
      quat_tmp(0),quat_tmp(1),quat_tmp(2),quat_tmp(3));

  // Quaternion for rotation from world frame to desired end effector attitude. This is constant,
  // since we don't want the orientation of the end effector to move.
  Eigen::Quaternion<double> quat_n_a_des = Eigen::Quaternion<double>(
      orientation_desired(0), orientation_desired(1), orientation_desired(2),
      orientation_desired(3));

  // Quaternion for rotation from end effector attitude to desired end effector attitude.
  Eigen::Quaternion<double> quat_a_a_des = quat_n_a.conjugate().operator*(quat_n_a_des);

  // Angle Axis Representation for the given quaternion
  Eigen::AngleAxis<double> angleaxis_a_a_des = Eigen::AngleAxis<double>(quat_a_a_des);
  MatrixXd axis = angleaxis_a_a_des.axis();
  MatrixXd angularVelocity = k_omega * axis * angleaxis_a_a_des.angle();

  // Transforming angular velocity from joint frame to world frame
  MatrixXd angularVelocityWF = tree_local.relativeTransform(cache, WORLDFRAME_ID, ee_frame_id).linear() * angularVelocity;

  MatrixXd twist(6, 1);
  twist << angularVelocityWF, diff;
  output->set_value(twist);
}

} // namespace systems
} // namespace dairlib
