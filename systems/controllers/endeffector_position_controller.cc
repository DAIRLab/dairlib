#include "systems/controllers/endeffector_position_controller.h"

namespace dairlib{
namespace systems{

EndEffectorPositionController::EndEffectorPositionController(
	const MultibodyPlant<double>& plant, std::string ee_frame_name,
	Eigen::Vector3d ee_contact_frame, int num_joints, double k_p, double k_omega)
	: plant_(plant), plant_world_frame(plant_.world_frame()),
	ee_contact_frame_(ee_contact_frame),
	ee_joint_frame(plant_.GetFrameByName(ee_frame_name)){

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
  this->k_p = k_p;
  this->k_omega = k_omega;
}

void EndEffectorPositionController::CalcOutputTwist(
	const Context<double> &context, BasicVector<double>* output) const {

  VectorX<double> q_actual = this->EvalVectorInput(context,
	  joint_position_measured_port)->CopyToVector();

  VectorX<double> x_desired = this->EvalVectorInput(context,
	  endpoint_position_commanded_port)->CopyToVector();

  VectorX<double> orientation_desired = this->EvalVectorInput(context,
	  endpoint_orientation_commanded_port)->CopyToVector();


  Eigen::Vector3d x_actual;
  const std::unique_ptr<Context<double>> plant_context = plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q_actual);
  plant_.CalcPointsPositions(*plant_context, ee_joint_frame, ee_contact_frame_,
	                         plant_world_frame, &x_actual);

  MatrixXd diff = k_p * (x_desired - x_actual);

  // Quaternion for rotation from base to end effector
  Eigen::Quaternion<double> quat_n_a = plant_.CalcRelativeTransform(
	  *plant_context, plant_world_frame, ee_joint_frame).rotation().ToQuaternion();

  // Quaternion for rotation from world frame to desired end effector attitude.
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
  MatrixXd angularVelocityWF = plant_.CalcRelativeTransform(
	  *plant_context, ee_joint_frame, plant_world_frame).rotation() * angularVelocity;

  std::cout << x_desired << std::endl;
  printf("@@@@@@@@@@@@@@@@@\n");
  std::cout << x_actual << std::endl;

  // Limit maximum commanded velocities
  double linear_speed_limit = 0.5;

  double currVel = sqrt(diff(0, 0)*diff(0, 0) + diff(1, 0)*diff(1, 0) + diff(2, 0)*diff(2, 0));

  if (currVel > linear_speed_limit) {
      for (int i = 0; i < 3; i++) {
          diff(i, 0) = diff(i, 0) * (linear_speed_limit/currVel); // Setting max speed to linear_speed_limit
      }
      std::cout << "Warning: desired end effector velocity: " << currVel;
      std::cout << " exceeded limit of " << linear_speed_limit << std::endl;
      currVel = sqrt(diff(0, 0)*diff(0, 0) + diff(1, 0)*diff(1, 0) + diff(2, 0)*diff(2, 0));
      std::cout << "Set end effector velocity to " << currVel << std::endl;
  }

  // for(int i = 0; i < 3; i++) {
	//   double currSpeed = diff(i, 0);
	//   if (diff(i, 0) > linear_speed_limit) {
	// 	  diff(i, 0) = linear_speed_limit;
	// 	  std::cout << "Warning: velocity of component " << i;
	// 	  std::cout << " limited from " << currSpeed << " to ";
	// 	  std::cout << linear_speed_limit << std::endl;
	//   }
  // }

  MatrixXd twist(6, 1);
  twist << angularVelocityWF, diff;
  output->set_value(twist);
}

} // namespace systems
} // namespace dairlib
