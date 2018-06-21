#include "systems/controllers/clqr_controller.h"

namespace dairlib{
namespace systems{

ClqrController::ClqrController(const RigidBodyTree<double>& tree, int num_positions, int num_velocities, int num_actuators): tree_(tree)
{
    num_positions_ = num_positions;
    num_velocities_ = num_velocities;
    num_states_ = num_positions_ + num_velocities_;
    num_actuators_ = num_actuators;

    input_state_port_index_ = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();
    input_desired_port_index_ = this->DeclareVectorInputPort(BasicVector<double>(num_states_)).get_index();

    output_actuator_port_index_ = this->DeclareVectorOutputPort(BasicVector<double>(num_actuators_), &ClqrController::calcControl).get_index(); 

}

void ClqrController::calcControl(const Context<double>& context, BasicVector<double>* output_bv) const
{
    VectorXd out = VectorXd::Ones(num_actuators_)*10.0;
    output_bv->SetFromVector(out);

    auto input_state_port_vec = dynamic_cast<const BasicVector<double>*>(EvalVectorInput(context, input_state_port_index_))->get_value();
    auto input_desired_port_vec = dynamic_cast<const BasicVector<double>*>(EvalVectorInput(context, input_desired_port_index_))->get_value();
    auto q = input_state_port_vec.head(num_positions_);
    auto v = input_state_port_vec.tail(num_velocities_);
    //KinematicsCache<double> kcache = tree_.doKinematics(VectorXd::Zero(num_actuators_));


}


const InputPortDescriptor<double>& ClqrController::getInputStatePort()
{
    return this->get_input_port(input_state_port_index_);
}

const InputPortDescriptor<double>& ClqrController::getInputDesiredPort()
{
    return this->get_input_port(input_state_port_index_);
}

const OutputPort<double>& ClqrController::getOutputActuatorPort()
{
    return this->get_output_port(output_actuator_port_index_);
}

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

    







