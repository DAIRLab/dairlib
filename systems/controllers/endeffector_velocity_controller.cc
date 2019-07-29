#include "systems/controllers/endeffector_velocity_controller.h"

namespace dairlib{
namespace systems{

EndEffectorVelocityController::EndEffectorVelocityController(
    const MultibodyPlant<double>& plant, std::string ee_frame_name,
    Eigen::Vector3d ee_contact_frame, double k_d, double k_r,
    double joint_torque_limit) : plant_(plant),
    num_joints_(plant_.num_positions()),
    ee_joint_frame_(plant_.GetFrameByName(ee_frame_name)){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port_ = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints_)).get_index();
  joint_velocity_measured_port_ = this->DeclareVectorInputPort(
      "joint_velocity_measured", BasicVector<double>(num_joints_)).get_index();
  endpoint_twist_commanded_port_ = this->DeclareVectorInputPort(
      "endpoint_twist_commanded", BasicVector<double>(6)).get_index();

  // Note that this function contains a pointer to the callback function below.
  endpoint_torque_output_port_ = this->DeclareVectorOutputPort(
      BasicVector<double>(num_joints_),
      &EndEffectorVelocityController::CalcOutputTorques).get_index();

  ee_contact_frame_ = ee_contact_frame;
  k_d_ = k_d;
  k_r_ = k_r;
  joint_torque_limit_ = joint_torque_limit;
}

// Callback for DeclareVectorInputPort. No return value.
// The parameter 'output' is the output.
// This function is called many times a second.
void EndEffectorVelocityController::CalcOutputTorques(
  const Context<double>& context, BasicVector<double>* output) const {
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp
  // from the vector input ports
  const BasicVector<double>* q_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, joint_position_measured_port_);
  auto q = q_timestamped->get_value();

  const BasicVector<double>* q_dot_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, joint_velocity_measured_port_);
  auto q_dot = q_dot_timestamped->get_value();

  const BasicVector<double>* twist_desired_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, endpoint_twist_commanded_port_);
  auto twist_desired = twist_desired_timestamped->get_value();

  const std::unique_ptr<Context<double>> plant_context =
      plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q);
  plant_.SetVelocities(plant_context.get(), q_dot);

  // Calculating the jacobian of the kuka arm
  Eigen::MatrixXd J(6, num_joints_);
  plant_.CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context, ee_joint_frame_, ee_contact_frame_,
      &J);

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd twist_actual = J * q_dot;

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << k_r_, k_r_, k_r_, k_d_, k_d_, k_d_;

  // Calculating Mass Matrix
  Eigen::MatrixXd H(plant_.num_positions(), plant_.num_positions());
  plant_.CalcMassMatrixViaInverseDynamics(*plant_context.get(), &H);
  MatrixXd Hee = J * H.inverse() * J.transpose();

  // Calculating the error
  MatrixXd error = gains * (twist_desired - twist_actual);

  MatrixXd generalizedForces = Hee.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(error);

  // Multiplying J^t x force to get torque outputs
  VectorXd commandedTorques(num_joints_);
  commandedTorques = J.transpose() * generalizedForces;

  std::cout << generalizedForces << std::endl;

  // Limit maximum commanded velocities
  for (int i = 0; i < num_joints_; i++) {
      if (commandedTorques(i, 0) > joint_torque_limit_) {
          commandedTorques(i, 0) = joint_torque_limit_;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << joint_torque_limit_ << std::endl;
      } else if (commandedTorques(i, 0) < -joint_torque_limit_) {
          commandedTorques(i, 0) = -joint_torque_limit_;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << -joint_torque_limit_ << std::endl;
      }
  }


  // Storing them in the output vector
  output->set_value(commandedTorques); // (7 x 6) * (6 x 1) = 7 x 1

}

} // namespace systems
} // namespace dairlib
