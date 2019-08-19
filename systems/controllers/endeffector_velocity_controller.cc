#include "systems/controllers/endeffector_velocity_controller.h"
#include <math.h>

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
  auto vel_desired = twist_desired.tail(3);

  const std::unique_ptr<Context<double>> plant_context =
      plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q);
  plant_.SetVelocities(plant_context.get(), q_dot);

  Eigen::Vector3d xactual_joint4;
  Eigen::Vector3d xactual_ee;
  Eigen::Vector3d zeros3d;
  Eigen::Vector3d joint4offset;
  joint4offset << 0, 0, 0.57;
  zeros3d.setZero();
  plant_.CalcPointsPositions(*plant_context, plant_.GetFrameByName("iiwa_link_4"), zeros3d,
	                         plant_.world_frame(), &xactual_joint4);
  plant_.CalcPointsPositions(*plant_context, plant_.GetFrameByName("iiwa_link_7"), ee_contact_frame_,
	                         plant_.world_frame(), &xactual_ee);

  Eigen::Vector3d joint4_contact_frame = xactual_ee - xactual_joint4;

  // Calculating the jacobian of the kuka arm
  Eigen::MatrixXd Jee(6, num_joints_);
  plant_.CalcFrameGeometricJacobianExpressedInWorld(
      *plant_context, plant_.GetFrameByName("iiwa_link_7"), ee_contact_frame_,
      &Jee);

  Eigen::MatrixXd Jeet = Jee.transpose();

  Eigen::MatrixXd Jee_1 = Jee.block(0, 0, 6, 4);
  Eigen::MatrixXd Jee_2 = Jee.block(0, 4, 6, 3);

  double a = q[0];
  double b = q[1];
  double c = q[2];
  double d = q[3];

  MatrixXd Jf(3, 4);
  Jf << 0,
       (sin(c)*sin(d))/(pow(cos(b),2)*pow(sin(d),2) + pow(sin(b),2)*pow(sin(c),2) + pow(cos(c),2)*pow(cos(d),2)*pow(sin(b),2) - 2*cos(b)*cos(c)*cos(d)*sin(b)*sin(d)),
       (pow(cos(b),2)*cos(d) - cos(d) + cos(b)*cos(c)*sin(b)*sin(d))/(pow(cos(b),2)*pow(sin(d),2) + pow(sin(b),2)*pow(sin(c),2) + pow(cos(c),2)*pow(cos(d),2)*pow(sin(b),2) - 2*cos(b)*cos(c)*cos(d)*sin(b)*sin(d)),
       -(sin(b)*sin(c)*(cos(b)*cos(d) + cos(c)*sin(b)*sin(d)))/(pow((cos(b)*sin(d) - cos(c)*cos(d)*sin(b)),2) + pow(sin(b),2)*pow(sin(c),2)),
       0,
       -(cos(d)*sin(b) - cos(b)*cos(c)*sin(d))/(pow((1 - pow((cos(b)*cos(d) + cos(c)*sin(b)*sin(d)),2)),(0.5))),
       -(sin(b)*sin(c)*sin(d))/(pow((1 - pow((cos(b)*cos(d) + cos(c)*sin(b)*sin(d)),2)),(0.5))),
       -(cos(b)*sin(d) - cos(c)*cos(d)*sin(b))/(pow((1 - pow((cos(b)*cos(d) + cos(c)*sin(b)*sin(d)),2)),(0.5))),
       1,
       (sin(c)*sin(d)*(cos(b)*cos(d) + cos(c)*sin(b)*sin(d)))/(pow(cos(d),2)*pow(sin(b),2) + pow(sin(c),2)*pow(sin(d),2) + pow(cos(b),2)*pow(cos(c),2)*pow(sin(d),2) - 2*cos(b)*cos(c)*cos(d)*sin(b)*sin(d)),
       -(cos(b)*pow(cos(d),2) - cos(b) + cos(c)*cos(d)*sin(b)*sin(d))/(pow(cos(d),2)*pow(sin(b),2) + pow(sin(c),2)*pow(sin(d),2) + pow(cos(b),2)*pow(cos(c),2)*pow(sin(d),2) - 2*cos(b)*cos(c)*cos(d)*sin(b)*sin(d)),
       -(sin(b)*sin(c))/(pow(cos(d),2)*pow(sin(b),2) + pow(sin(c),2)*pow(sin(d),2) + pow(cos(b),2)*pow(cos(c),2)*pow(sin(d),2) - 2*cos(b)*cos(c)*cos(d)*sin(b)*sin(d));

  Eigen::MatrixXd J = (Jee_1 + Jee_2 * Jf).block(3, 0, 3, 4);
  Eigen::MatrixXd Jt = J.transpose();

  // std::cout << "J" << std::endl;
  // std::cout << J << std::endl;


  // Calculating Mass Matrix
  Eigen::MatrixXd H(plant_.num_positions(), plant_.num_positions());
  plant_.CalcMassMatrixViaInverseDynamics(*plant_context.get(), &H);
  Eigen::MatrixXd Hi = H.inverse();
  Eigen::MatrixXd H1 = Hi.block(0, 0, 4, 4);

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd vel_actual = J * q_dot.head(4);

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 3> gains(3);
  gains.diagonal() << k_d_, k_d_, k_d_;

  // Calculating the error
  // MatrixXd error = gains * (twist_desired - twist_actual);
  VectorXd error = gains*(vel_desired - vel_actual);

  // Multiplying J^t x force to get torque outputs
  VectorXd torques(num_joints_);
  VectorXd commandedTorques(num_joints_);

  commandedTorques << Jt * (J * H1 * Jt).inverse() * error, 0, 0, 0;
  std::cout << "commandedTorques" << std::endl;
  std::cout << commandedTorques << std::endl;

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
  //output->set_value(commandedTorques); // (7 x 6) * (6 x 1) = 7 x 1
  //std::cout << commandedTorques << std::endl;
  // output->set_value(torques);
  output->set_value(commandedTorques);


}

} // namespace systems
} // namespace dairlib
