//
// Created by brian on 3/8/21.
//

#include "koopman_mpc.h"

using drake::multibody::MultibodyPlant;
using drake::systems::Context;

namespace dairlib::systems::controllers{

KoopmanMPC::KoopmanMPC(const MultibodyPlant<double>& plant,
                       const Context<double> *plant_context,
                       bool planar, bool used_with_finite_state_machine) :
                       plant_(plant),
                       plant_context_(plant_context){

  nx_ = planar ? kNxPlanar : kNx3d;
  nu_ = planar ? kNuPlanar : kNu3d;

  // Create Ports
  state_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

   x_des_port_ = this->DeclareVectorInputPort(
      BasicVector<double>(nx_)).get_index();

  if ( use_fsm_ ) {
    fsm_port_ = this->DeclareVectorInputPort(
        BasicVector<double>(2)).get_index();
  }
}


}