#define NUM_JOINTS 7
#define K_P 50
#define K_OMEGA 50
#define WORLDFRAME_ID 0
#define ENDEFFECTOR_FRAME_ID 10

#include "systems/controllers/iiwa_position_controller.h"

namespace dairlib{
namespace systems{

KukaIiwaPositionController::KukaIiwaPositionController()
{

	// Set up this block's input and output ports
  	// Input port values will be accessed via EvalVectorInput() later
	joint_position_measured_port = this->DeclareVectorInputPort("joint_position_measured", BasicVector<double>(NUM_JOINTS)).get_index();
	endpoint_position_commanded_port = this->DeclareVectorInputPort("endpoint_position_commanded", BasicVector<double>(3)).get_index();

	this->DeclareVectorOutputPort(BasicVector<double>(3), &KukaIiwaPositionController::CalcOutputVelocity);
	this->DeclareVectorOutputPort(BasicVector<double>(3), &KukaIiwaPositionController::CalcOutputAngularVelocity);

    // Adding Kuka model from URDF-- from Drake kuka simulation files
    const char* kModelPath = "../drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
    const std::string urdf = FindResourceOrThrow(kModelPath);

    // initialize a rigidbodytree, and add a model of the Kuka Iiwa arm to it.
    tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(urdf, drake::multibody::joints::kFixed, tree.get());

    // The coordinates for the end effector with respect to the last joint.
    // Eventually passed into transformPointsJacobian()
    eeContactFrame << 0.0, 0, 0.09;


}

void KukaIiwaPositionController::CalcOutputVelocity(const Context<double> &context, BasicVector<double>* output) const
{

	VectorX<double> q_actual = this->EvalVectorInput(context, joint_position_measured_port)->CopyToVector();
	VectorX<double> x_desired = this->EvalVectorInput(context, endpoint_position_commanded_port)->CopyToVector();

	KinematicsCache<double> cache = tree->doKinematics(q_actual);

	MatrixXd x_actual = tree->transformPoints(cache, eeContactFrame, ENDEFFECTOR_FRAME_ID, WORLDFRAME_ID);
	MatrixXd diff = x_desired - x_actual;

	// If you want to collect data about the desired position vs actual position,
	// uncomment the below line, recompile, and pipe the program into a .csv file
	//std::cout << x_desired(0) << "," << x_desired(1) << "," << x_desired(2) << "," << x_actual(0) << "," << x_actual(1) << "," << x_actual(2) << std::endl;
	output->set_value(K_P * diff);

}

void KukaIiwaPositionController::CalcOutputAngularVelocity(
    const Context<double> &context,
    BasicVector<double>* output
) const {

   	VectorX<double> q_actual = this->
                               EvalVectorInput(context, joint_position_measured_port)->
                               CopyToVector();

   	KinematicsCache<double> cache = tree->doKinematics(q_actual);

   	// Quaternion for rotation from base to end effector
   	MatrixXd quat_tmp= tree->relativeQuaternion(cache, ENDEFFECTOR_FRAME_ID, WORLDFRAME_ID);
   	Eigen::Quaternion<double> quat_n_a = Eigen::Quaternion<double>(quat_tmp(0), quat_tmp(1), quat_tmp(2), quat_tmp(3));

   	// Quaternion for rotation from world frame to desired end effector attitude. This is constant,
   	// since we don't want the orientation of the end effector to move.
   	Eigen::Quaternion<double> quat_n_a_des = Eigen::Quaternion<double>(0, 0, 1, 0);

   	// Quaternion for rotation from end effector attitude to desired end effector attitude.
   	Eigen::Quaternion<double> quat_a_a_des = quat_n_a.conjugate().operator*(quat_n_a_des);

   	// Angle Axis Representation for the given quaternion
   	Eigen::AngleAxis<double> angleaxis_a_a_des = Eigen::AngleAxis<double>(quat_a_a_des);
   	MatrixXd axis = angleaxis_a_a_des.axis();
   	MatrixXd angularVelocity = K_OMEGA * axis * angleaxis_a_a_des.angle();

   	// We transform the angularVelocity from the end effector frame
   	// to world frame, then output it.
   	output->set_value(tree->relativeTransform(cache, WORLDFRAME_ID, ENDEFFECTOR_FRAME_ID).linear() * angularVelocity);

}


} // namespace systems
} // namespace dairlib
