#include "cartpole_output_interface.h"

using drake::systems::Context;
using drake::multibody::MultibodyPlant;

using dairlib::systems::OutputVector;

namespace dairlib{

CartpoleOutputInterface::CartpoleOutputInterface(
    const MultibodyPlant<double> &plant) : plant_(plant) {
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(
          plant_.num_positions(),
          plant_.num_velocities(),
          plant.num_actuators()),
      &CartpoleOutputInterface::Output);
}

void CartpoleOutputInterface::SetupOutputInterface() {
  ConfigureEpos();
  ConfigureLabjack();
}

void CartpoleOutputInterface::ConfigureEpos() {
  unsigned int error_code = 0;
  KeyHandle_ = epos::OpenDevice(&error_code);
  DRAKE_ASSERT(error_code == 0);
}

void CartpoleOutputInterface::ConfigureLabjack() {
  
}



}