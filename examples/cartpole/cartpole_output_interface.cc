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
  epos::HomeDevice(KeyHandle_);
  epos::EnableDevice(KeyHandle_);
}

void CartpoleOutputInterface::ConfigureLabjack() {

}

void CartpoleOutputInterface::Output(
    const Context<double> &context, OutputVector<double> *output) const {

  // Maxon API calls
  double cart_pos = epos::GetCartPosition(KeyHandle_);
  double effort = epos::GetForceFromCurrent(KeyHandle_);

  // Labjack API calls
}



}