#ifndef CLQR_CONTROLLER_H
#define CLQR_CONTROLLER_H

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/basic_vector.h"

namespace dairlib{
namespace systems{

using drake::systems::BasicVector;

class ClqrController : public LeafSystem<double>
{
    public: 

        ClqrController(int num_positions, int num_velocities, int num_inputs);

    private:

        int input_state_port_index_;
        int input_desired_port_index_;
        int output_actuator_port_index_;
        int num_positions_;
        int num_velocities_;
        int num_states_;
        int num_inputs_;




#endif
