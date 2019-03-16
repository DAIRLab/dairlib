#include <algorithm>
#include "examples/Cassie/networking/cassie_output_receiver.h"


namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::LeafSystem;

void copy_elmo(const elmo_out_t& input, lcmt_elmo_out* output);
void copy_leg(const cassie_leg_out_t& input, lcmt_cassie_leg_out* output);
template <typename T> void copy_vector(const T* input, T* output, int size);

CassieOutputReceiver::CassieOutputReceiver() {
  this->DeclareAbstractInputPort("lcmt_cassie_out",
    drake::Value<dairlib::lcmt_cassie_out>{});
  this->DeclareAbstractOutputPort(
      "cassie_out_t", &CassieOutputReceiver::CopyOutput);
}


// Utility methods for copying data
void copy_elmo(const lcmt_elmo_out& input, elmo_out_t* output) {
  output->statusWord = input.statusWord;
  output->position = input.position;
  output->velocity = input.velocity;
  output->torque = input.torque;
  output->driveTemperature = input.driveTemperature;
  output->dcLinkVoltage = input.dcLinkVoltage;
  output->torqueLimit = input.torqueLimit;
  output->gearRatio = input.gearRatio;
}


void copy_leg(const lcmt_cassie_leg_out& input, cassie_leg_out_t* output) {
  copy_elmo(input.hipRollDrive, &output->hipRollDrive);
  copy_elmo(input.hipYawDrive, &output->hipYawDrive);
  copy_elmo(input.hipPitchDrive, &output->hipPitchDrive);
  copy_elmo(input.kneeDrive, &output->kneeDrive);
  copy_elmo(input.footDrive, &output->footDrive);
  output->shinJoint.position = input.shinJoint.position;
  output->shinJoint.velocity = input.shinJoint.velocity;
  output->tarsusJoint.position = input.tarsusJoint.position;
  output->tarsusJoint.velocity = input.tarsusJoint.velocity;
  output->footJoint.position = input.footJoint.position;
  output->footJoint.velocity = input.footJoint.velocity;
  output->medullaCounter = input.medullaCounter;
  output->medullaCpuLoad = input.medullaCpuLoad;
  output->reedSwitchState = input.reedSwitchState;
}

template <typename T>
void copy_vector(const T* input, T* output, int size) {
  for (int i = 0; i < size; i++) {
    output[i] = input[i];
  }
}

void CassieOutputReceiver::CopyOutput(const Context<double>& context,
                                     cassie_out_t* cassie_out) const {
  const auto& message = this->EvalAbstractInput(
      context, 0)->get_value<dairlib::lcmt_cassie_out>();

  // copy pelvis
  copy_vector(message.pelvis.targetPc.etherCatStatus,
              cassie_out->pelvis.targetPc.etherCatStatus, 6);
  copy_vector(message.pelvis.targetPc.etherCatNotifications,
              cassie_out->pelvis.targetPc.etherCatNotifications, 21);

  cassie_out->pelvis.targetPc.taskExecutionTime =
      message.pelvis.targetPc.taskExecutionTime;
  cassie_out->pelvis.targetPc.overloadCounter =
      message.pelvis.targetPc.overloadCounter;

  cassie_out->pelvis.targetPc.cpuTemperature =
      message.pelvis.targetPc.cpuTemperature;

  cassie_out->pelvis.battery.dataGood =
      message.pelvis.battery.dataGood;
  cassie_out->pelvis.battery.stateOfCharge =
      message.pelvis.battery.stateOfCharge;
  cassie_out->pelvis.battery.current =
      message.pelvis.battery.current;
  copy_vector(message.pelvis.battery.voltage,
              cassie_out->pelvis.battery.voltage, 12);
  copy_vector(message.pelvis.battery.temperature,
              cassie_out->pelvis.battery.temperature, 4);

  cassie_out->pelvis.radio.radioReceiverSignalGood =
      message.pelvis.radio.radioReceiverSignalGood;
  cassie_out->pelvis.radio.receiverMedullaSignalGood =
      message.pelvis.radio.receiverMedullaSignalGood;
  copy_vector(message.pelvis.radio.channel,
              cassie_out->pelvis.radio.channel, 16);

  cassie_out->pelvis.vectorNav.dataGood =
      message.pelvis.vectorNav.dataGood;
  cassie_out->pelvis.vectorNav.vpeStatus =
      message.pelvis.vectorNav.vpeStatus;
  cassie_out->pelvis.vectorNav.pressure =
      message.pelvis.vectorNav.pressure;
  cassie_out->pelvis.vectorNav.temperature =
      message.pelvis.vectorNav.temperature;
  copy_vector(message.pelvis.vectorNav.magneticField,
              cassie_out->pelvis.vectorNav.magneticField, 3);
  copy_vector(message.pelvis.vectorNav.angularVelocity,
              cassie_out->pelvis.vectorNav.angularVelocity, 3);
  copy_vector(message.pelvis.vectorNav.linearAcceleration,
              cassie_out->pelvis.vectorNav.linearAcceleration, 3);
  copy_vector(message.pelvis.vectorNav.orientation,
              cassie_out->pelvis.vectorNav.orientation, 4);
  cassie_out->pelvis.medullaCounter = message.pelvis.medullaCounter;
  cassie_out->pelvis.medullaCpuLoad = message.pelvis.medullaCpuLoad;
  cassie_out->pelvis.bleederState = message.pelvis.bleederState;
  cassie_out->pelvis.leftReedSwitchState = message.pelvis.leftReedSwitchState;
  cassie_out->pelvis.rightReedSwitchState = message.pelvis.rightReedSwitchState;
  cassie_out->pelvis.vtmTemperature = message.pelvis.vtmTemperature;

  copy_leg(message.leftLeg, &cassie_out->leftLeg);
  copy_leg(message.rightLeg, &cassie_out->rightLeg);

  copy_vector(message.messages,
              cassie_out->messages, 4);
  cassie_out->isCalibrated = message.isCalibrated;
}

}  // namespace systems
}  // namespace dairlib
