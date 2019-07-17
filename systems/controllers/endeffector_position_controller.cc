#include "systems/controllers/endeffector_position_controller.h"

namespace dairlib{
namespace systems{

EndEffectorPositionController::EndEffectorPositionController(
	const MultibodyPlant<double>& plant, std::string ee_frame_name,
	Eigen::Vector3d ee_contact_frame, double k_p, double k_omega)
	: plant_(plant), plant_world_frame_(plant_.world_frame()),
	ee_contact_frame_(ee_contact_frame),
	ee_joint_frame_(plant_.GetFrameByName(ee_frame_name)){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port_ = this->DeclareVectorInputPort(
	  "joint_position_measured", BasicVector<double>(plant_.num_positions())).get_index();
  endpoint_position_commanded_port_ = this->DeclareVectorInputPort(
	  "endpoint_position_commanded", BasicVector<double>(3)).get_index();
  endpoint_orientation_commanded_port_ = this->DeclareVectorInputPort(
	  "endpoint_orientation_commanded", BasicVector<double>(4)).get_index();
  endpoint_twist_cmd_output_port_ = this->DeclareVectorOutputPort(
	  BasicVector<double>(6), &EndEffectorPositionController::CalcOutputTwist).get_index();

  // The coordinates for the end effector with respect to the last joint.
  // Eventually passed into transformPointsJacobian()
  k_p_ = k_p;
  k_omega_ = k_omega;
}

void EndEffectorPositionController::CalcOutputTwist(
	const Context<double> &context, BasicVector<double>* output) const {

  VectorX<double> q_actual = this->EvalVectorInput(context,
	  joint_position_measured_port_)->CopyToVector();

  VectorX<double> x_desired = this->EvalVectorInput(context,
	  endpoint_position_commanded_port_)->CopyToVector();

  VectorX<double> orientation_desired = this->EvalVectorInput(context,
	  endpoint_orientation_commanded_port_)->CopyToVector();


  Eigen::Vector3d x_actual;
  const std::unique_ptr<Context<double>> plant_context =
      plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q_actual);
  plant_.CalcPointsPositions(*plant_context, ee_joint_frame_, ee_contact_frame_,
	                         plant_world_frame_, &x_actual);

  VectorXd diff = k_p_ * (x_desired - x_actual);

  // Quaternion for rotation from base to end effector
  Eigen::Quaternion<double> quat_n_a = plant_.CalcRelativeTransform(
	  *plant_context, plant_world_frame_, ee_joint_frame_).rotation().ToQuaternion();

  // Quaternion for rotation from world frame to desired end effector attitude.
  Eigen::Quaternion<double> quat_n_a_des = Eigen::Quaternion<double>(
	  orientation_desired(0), orientation_desired(1), orientation_desired(2),
	  orientation_desired(3));

  // Quaternion for rotation
  // from end effector attitude to desired end effector attitude.
  Eigen::Quaternion<double> quat_a_a_des =
      quat_n_a.conjugate().operator*(quat_n_a_des);

  // Angle Axis Representation for the given quaternion
  Eigen::AngleAxis<double> angleaxis_a_a_des =
      Eigen::AngleAxis<double>(quat_a_a_des);
  MatrixXd axis = angleaxis_a_a_des.axis();
  MatrixXd angularVelocity = k_omega_ * axis * angleaxis_a_a_des.angle();

  // Transforming angular velocity from joint frame to world frame
  VectorXd angularVelocityWF = plant_.CalcRelativeTransform(
	  *plant_context, ee_joint_frame_, plant_world_frame_).rotation() * angularVelocity;

  // TODO: parse these from the json file
  // Limit maximum commanded linear velocity
  double linear_speed_limit = 3.0;
  double currVel = diff.norm();

  if (currVel > linear_speed_limit || currVel < -linear_speed_limit) {
      diff = diff * (linear_speed_limit/currVel);
      std::cout << "Warning: desired end effector velocity: " << currVel;
      std::cout << " exceeded limit of " << linear_speed_limit << std::endl;
      currVel = diff.norm();
      std::cout << "Set end effector velocity to " << currVel << std::endl;
  }

  // Limit maximum commanded angular velocity
  double angular_speed_limit = 3.0;
  double currAngVel = angularVelocityWF.norm();
  if (currAngVel > angular_speed_limit || currAngVel < -angular_speed_limit) {
      angularVelocityWF = angularVelocityWF * (angular_speed_limit/currAngVel);
      std::cout << "Warning: desired end effector velocity: " << currAngVel;
      std::cout << " exceeded limit of " << angular_speed_limit << std::endl;
      currAngVel = angularVelocityWF.norm();
      std::cout << "Set end effector angular velocity to " << currAngVel;
      std::cout << std::endl;
  }

  MatrixXd twist(6, 1);
  twist << angularVelocityWF, diff;
  output->set_value(twist);
}

} // namespace systems
} // namespace dairlib
