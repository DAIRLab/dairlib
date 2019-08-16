#include "systems/controllers/endeffector_position_controller.h"
#include <math.h>

namespace dairlib{
namespace systems{

EndEffectorPositionController::EndEffectorPositionController(
	const MultibodyPlant<double>& plant, std::string ee_frame_name,
	Eigen::Vector3d ee_contact_frame, double k_p, double k_omega,
    double max_linear_vel, double max_angular_vel)
	: plant_(plant), plant_world_frame_(plant_.world_frame()),
	ee_contact_frame_(ee_contact_frame),
	ee_joint_frame_(plant_.GetFrameByName(ee_frame_name)){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port_ = this->DeclareVectorInputPort(
	  "joint_position_measured", BasicVector<double>(plant_.num_positions())).get_index();
  endpoint_position_commanded_port_ = this->DeclareVectorInputPort(
	  "endpoint_position_commanded", BasicVector<double>(3)).get_index();
  endpoint_velocity_commanded_port_ = this->DeclareVectorInputPort(
      "endpoint_velocity_commanded", BasicVector<double>(3)).get_index();
  endpoint_orientation_commanded_port_ = this->DeclareVectorInputPort(
	  "endpoint_orientation_commanded", BasicVector<double>(4)).get_index();
  endpoint_twist_cmd_output_port_ = this->DeclareVectorOutputPort(
	  BasicVector<double>(6), &EndEffectorPositionController::CalcOutputTwist).get_index();

  // The coordinates for the end effector with respect to the last joint.
  // Eventually passed into transformPointsJacobian()
  k_p_ = k_p;
  k_omega_ = k_omega;
  max_linear_vel_ = max_linear_vel;
  max_angular_vel_ = max_angular_vel;
}

void EndEffectorPositionController::CalcOutputTwist(
	const Context<double> &context, BasicVector<double>* output) const {

  VectorX<double> q_actual = this->EvalVectorInput(context,
	  joint_position_measured_port_)->CopyToVector();

  VectorX<double> x_desired = this->EvalVectorInput(context,
	  endpoint_position_commanded_port_)->CopyToVector();

  VectorX<double> xdot_desired = this->EvalVectorInput(context,
      endpoint_velocity_commanded_port_)->CopyToVector();

  VectorX<double> orientation_desired = this->EvalVectorInput(context,
	  endpoint_orientation_commanded_port_)->CopyToVector();


  Eigen::Vector3d x_actual;
  const std::unique_ptr<Context<double>> plant_context =
      plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q_actual);
  plant_.CalcPointsPositions(*plant_context, ee_joint_frame_, ee_contact_frame_,
	                         plant_world_frame_, &x_actual);

  VectorXd diff = k_p_ * (x_desired - x_actual) + xdot_desired;

  std::cout << "desired:\n" << x_desired << std::endl;
	std::cout << "actual:\n" << x_actual << std::endl;

  // Rotation Matrix for rotation from base to end effector
  drake::Matrix3<double> rot_n_a = plant_.CalcRelativeTransform(
	  *plant_context, plant_world_frame_, ee_joint_frame_).rotation().matrix();

  // Quaternion for rotation from world frame to desired end effector attitude.
  drake::Matrix3<double> rot_n_a_des = Eigen::Quaternion<double>(
      orientation_desired(0), orientation_desired(1), orientation_desired(2),
	  orientation_desired(3)).toRotationMatrix();

  // Rotation Matrix from end effector attitude to desired end effector attitude.
  drake::Matrix3<double> rot_a_a_des = rot_n_a.transpose() * rot_n_a_des ;
  rot_a_a_des = 0.5 * (rot_a_a_des - rot_a_a_des.transpose());
  // std::cout << "rot: " << std::endl;
  // std::cout << rot_a_a_des << std::endl;
  Eigen::Vector3d skewInverse;
  skewInverse << rot_a_a_des(2, 1), rot_a_a_des(0, 2), rot_a_a_des(1, 0);
  std::cout << rot_a_a_des(2, 1) << std::endl;


  // Angle Axis Representation for the given quaternion
  // Eigen::AngleAxis<double> angleaxis_a_a_des =
  //     Eigen::AngleAxis<double>(quat_a_a_des);
  // MatrixXd axis = angleaxis_a_a_des.axis();
  MatrixXd angularVelocity = k_omega_ * skewInverse;

  std::cout << "angvel: " << std::endl;
  std::cout << angularVelocity << std::endl;

  // Transforming angular velocity from joint frame to world frame
  VectorXd angularVelocityWF = plant_.CalcRelativeTransform(
	  *plant_context, ee_joint_frame_, plant_world_frame_).rotation() * angularVelocity;

  Eigen::Matrix3d m;
  m << 1, 0, 0, 0, 0, -1, 0, 1, 0;

  // std::cout << "Euler angles: " << std::endl;

  MatrixXd rot_4_7 = m*(plant_.GetBodyByName("iiwa_link_4").EvalPoseInWorld(*plant_context).rotation().matrix().inverse() * rot_n_a_des);

  Eigen::Vector3d eulerangles = (m*(plant_.GetBodyByName("iiwa_link_4").EvalPoseInWorld(*plant_context).rotation().matrix().inverse() * rot_n_a_des)).eulerAngles(2,1,2);
  // std::cout << (plant_.CalcRelativeTransform(
	//   *plant_context, plant_.GetFrameByName("iiwa_link_0"), plant_.GetFrameByName("iiwa_link_4")).rotation() * rot_n_a_des).eulerAngles(2, 1, 2) << std::endl;

  std::cout << "Matrix from 0 to 4" << std::endl;
  std::cout << plant_.GetBodyByName("iiwa_link_0").EvalPoseInWorld(*plant_context).rotation().matrix().inverse() * plant_.GetBodyByName("iiwa_link_4").EvalPoseInWorld(*plant_context).rotation().matrix();

  std::cout << "wanted angles: " << std::endl;
  std::cout << atan2(rot_4_7(1, 2), rot_4_7(0, 2)) << std::endl;
  std::cout << acos(rot_4_7(2, 2)) << std::endl;
  std::cout << atan2(rot_4_7(2, 1), -1*rot_4_7(2, 0)) << std::endl;

  std::cout << "eulerangles" << std::endl;
  std::cout << eulerangles << std::endl;

  std::cout << "last three angles: " << std::endl;
  std::cout << q_actual[4] << std::endl;
  std::cout << q_actual[5] << std::endl;
  std::cout << q_actual[6] << std::endl;
  // Limit maximum commanded linear velocity
  double currVel = diff.norm();

  if (currVel > max_linear_vel_) {
      diff = diff * (max_linear_vel_/currVel);
      std::cout << "Warning: desired end effector velocity: " << currVel;
      std::cout << " exceeded limit of " << max_linear_vel_ << std::endl;
      currVel = diff.norm();
      std::cout << "Set end effector velocity to " << currVel << std::endl;
  }

  // Limit maximum commanded angular velocity
  double currAngVel = angularVelocityWF.norm();
  if (currAngVel > max_angular_vel_) {
      angularVelocityWF = angularVelocityWF * (max_angular_vel_/currAngVel);
      std::cout << "Warning: desired end effector velocity: " << currAngVel;
      std::cout << " exceeded limit of " << max_angular_vel_ << std::endl;
      currAngVel = angularVelocityWF.norm();
      std::cout << "Set end effector angular velocity to " << currAngVel;
      std::cout << std::endl;
  }

  MatrixXd twist(6, 1);
  twist << eulerangles, diff;
  std::cout << "angularvel" << std::endl;
  std::cout << angularVelocityWF << std::endl;
  output->set_value(twist);
}

} // namespace systems
} // namespace dairlib
