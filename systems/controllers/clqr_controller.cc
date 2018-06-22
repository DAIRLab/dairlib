#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(const RigidBodyTree<double>& tree, VectorXd x0, int num_positions, int num_velocities, int num_actuators): LinearController(num_positions, num_velocities, num_actuators), tree_(tree), x0_(x0)
{
    num_positions_ = num_positions;
    num_velocities_ = num_velocities;
    num_states_ = num_positions_ + num_velocities_;
    num_actuators_ = num_actuators;
    F_ = computeF();
    //std::cout << F_ << std::endl;

    //input_state_port_index_ = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();
    //input_desired_port_index_ = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();

    //output_actuator_port_index_ = this->DeclareVectorOutputPort(BasicVector<double>(num_actuators_), &ClqrController::calcControl).get_index(); 

}

Matrix<double, Dynamic, Dynamic> ClqrController::computeF()
{
    VectorXd q = x0_.head(num_positions_);
    VectorXd v = x0_.tail(num_velocities_);
    
    KinematicsCache<double> kcache = tree_.doKinematics(q, v);
    Matrix<double, Dynamic, Dynamic> c_jac = tree_.positionConstraintsJacobian(kcache);

    const int r = c_jac.rows();
    const int c = c_jac.cols();

    // j_dot is zero for the time invariant case

    Matrix<double, Dynamic, Dynamic> F =  MatrixXd::Zero(2*r, 2*c);

    std::cout << c_jac << std::endl;

    F.block(0, 0, r, c) = c_jac;
    F.block(r, c, r, c) = c_jac;
    std::cout << F << std::endl;
    std::cout << "TEST" << std::endl;

    return F;

    
}

//void ClqrController::calcControl(const Context<double>& context, BasicVector<double>* output_bv) const
//{
//    VectorXd out = VectorXd::Ones(num_actuators_)*10.0;
//    output_bv->SetFromVector(out);
//
//    auto input_state_port_vec = dynamic_cast<const BasicVector<double>*>(EvalVectorInput(context, input_state_port_index_))->get_value();
//    VectorXd q = input_state_port_vec.head(num_positions_);
//    VectorXd v = input_state_port_vec.tail(num_velocities_);
//
//    AutoDiffVecXd input_state_port_vec_autodiff = drake::math::initializeAutoDiff(input_state_port_vec);
//    AutoDiffVecXd q_autodiff = input_state_port_vec_autodiff.head(num_positions_);
//    AutoDiffVecXd v_autodiff = input_state_port_vec_autodiff.tail(num_velocities_);
//    
//    auto input_desired_port_vec = dynamic_cast<const BasicVector<double>*>(EvalVectorInput(context, input_state_port_index_))->get_value();
//    KinematicsCache<AutoDiffXd> kcache = tree_.doKinematics(q_autodiff, v_autodiff);
//
//    AutoDiffVecXd c_jac_autodiff = tree_.positionConstraintsJacobian(kcache);
//
//    Matrix<double, Eigen::Dynamic, Eigen::Dynamic> c_jac_dot = drake::math::autoDiffToGradientMatrix(c_jac_autodiff);
//    std::cout << c_jac_dot << std::endl;
//    std::cout << "----------------------" << std::endl;
//    std::cout << std::setprecision(9) << c_jac_dot(0, 10) << " " << c_jac_dot(0, 11) << std::endl;
//    std::cout << "----------------------" << std::endl;
//    
//    
//
//
//}

//const InputPortDescriptor<double>& ClqrController::getInputStatePort()
//{
//    return this->get_input_port(input_state_port_index_);
//}
//
//const InputPortDescriptor<double>& ClqrController::getInputDesiredPort()
//{
//    return this->get_input_port(input_state_port_index_);
//}
//
//const OutputPort<double>& ClqrController::getOutputActuatorPort()
//{
//    return this->get_output_port(output_actuator_port_index_);
//}

int ClqrController::getNumPositions()
{
    return num_positions_;
}

int ClqrController::getNumVelocities()
{
    return num_velocities_;
}

int ClqrController::getNumStates()
{
    return num_states_;
}

int ClqrController::getNumActuators()
{
    return num_actuators_;
}



}//namespace systems
}//namespace dairlib

    







