#ifndef CLQR_CONTROLLER_H
#define CLQR_CONTROLLER_H

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/rigid_body_tree.h"

using Eigen::VectorXd;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::LeafSystem;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;

namespace dairlib{
namespace systems{

class ClqrController : public LeafSystem<double>
{
    public: 

        ClqrController(const RigidBodyTree<double>& tree, int num_positions, int num_velocities, int num_inputs);
        const InputPortDescriptor<double>& getInputStatePort();
        const InputPortDescriptor<double>& getInputDesiredPort();
        const OutputPort<double>& getOutputActuatorPort();
        int getNumPositions();
        int getNumVelocities();
        int getNumStates();
        int getNumActuators();


    private:

        void calcControl(const Context<double>& context, BasicVector<double>*output) const;
        const RigidBodyTree<double>& tree_;
        int input_state_port_index_;
        int input_desired_port_index_;
        int output_actuator_port_index_;
        int num_positions_;
        int num_velocities_;
        int num_states_;
        int num_actuators_;
};

}// namespace systems
}//namespace dairlib




#endif
