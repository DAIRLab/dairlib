#include "systems/controllers/endeffector_velocity_controller.h"

namespace dairlib{
namespace systems{

EndEffectorVelocityController::EndEffectorVelocityController(
    const MultibodyPlant<double>& plant, std::string ee_frame_name,
    Eigen::Vector3d ee_contact_frame, int num_joints, double k_d, double k_r)
    : plant_(plant), num_joints_(num_joints),
    ee_joint_frame(plant_.GetFrameByName(ee_frame_name)){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints_)).get_index();
  joint_velocity_measured_port = this->DeclareVectorInputPort(
      "joint_velocity_measured", BasicVector<double>(num_joints_)).get_index();
  endpoint_twist_commanded_port = this->DeclareVectorInputPort(
      "endpoint_twist_commanded", BasicVector<double>(6)).get_index();

  // Note that this function contains a pointer to the callback function below.
  endpoint_torque_output_port = this->DeclareVectorOutputPort(
      BasicVector<double>(num_joints_),
      &EndEffectorVelocityController::CalcOutputTorques).get_index();

  this->ee_contact_frame = ee_contact_frame;
  this->k_d = k_d;
  this->k_r = k_r;
}

// Callback for DeclareVectorInputPort. No return value.
// The parameter 'output' is the output.
// This function is called many times a second.
void EndEffectorVelocityController::CalcOutputTorques(
  const Context<double>& context, BasicVector<double>* output) const {
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp
  // from the vector input ports
  VectorX<double> q = this->EvalVectorInput(context,
      joint_position_measured_port)->CopyToVector().head(num_joints_);

  VectorX<double> q_dot = this->EvalVectorInput(context,
      joint_velocity_measured_port)->CopyToVector().head(num_joints_);

  VectorX<double> twist_desired = this->EvalVectorInput(context,
      endpoint_twist_commanded_port)->CopyToVector();

  const std::unique_ptr<Context<double>> plant_context = plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q);
  plant_.SetVelocities(plant_context.get(), q_dot);

  // Calculating the jacobian of the kuka arm
  Eigen::MatrixXd frameSpatialVelocityJacobian(6, num_joints_);
  plant_.CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context, ee_joint_frame, ee_contact_frame,
      &frameSpatialVelocityJacobian);

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd twist_actual = frameSpatialVelocityJacobian * q_dot;

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << k_r, k_r, k_r, k_d, k_d, k_d;

  // Calculating the error
  MatrixXd generalizedForces = gains * (twist_desired - twist_actual);

  // Multiplying J^t x force to get torque outputs
  VectorXd outTorques(num_joints_);
  outTorques = frameSpatialVelocityJacobian.transpose() * generalizedForces;

  // Limit maximum commanded velocities
  double max_torque_limit = (double) num_joints_;
  double currVel = 0;
  for (int i = 0; i < num_joints_; i++) {
      currVel += outTorques(i, 0)*outTorques(i, 0);
  }
  currVel = sqrt(currVel);

  if (currVel > max_torque_limit) {
      for (int i = 0; i < num_joints_; i++) {
          outTorques(i, 0) = outTorques(i, 0) * (max_torque_limit/currVel);
      }
      std::cout << "Warning: l2 norm of desired joint joint torques: " << currVel;
      std::cout << " exceeded limit of " << max_torque_limit << std::endl;
  }

  // Storing them in the output vector
  output->set_value(outTorques); // (7 x 6) * (6 x 1) = 7 x 1

}

} // namespace systems
} // namespace dairlib
