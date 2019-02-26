#include <algorithm>
#include "examples/Cassie/networking/cassie_output_sender.h"


namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::LeafSystem;

void copy_elmo(const elmo_out_t& input, lcmt_elmo_out* output);
void copy_leg(const cassie_leg_out_t& input, lcmt_cassie_leg_out* output);
template <typename T> void copy_vector(const T* input, T* output, int size);

CassieOutputSender::CassieOutputSender() {
  this->DeclareAbstractInputPort("cassie_out_t",
      drake::Value<cassie_out_t>{});
  this->DeclareAbstractOutputPort("lcmt_cassie_out",
      &CassieOutputSender::Output);
}


// Utility methods for copying data
void copy_elmo(const elmo_out_t& input, lcmt_elmo_out* output) {
  output->statusWord = input.statusWord;
  output->position = input.position;
  output->velocity = input.velocity;
  output->torque = input.torque;
  output->driveTemperature = input.driveTemperature;
  output->dcLinkVoltage = input.dcLinkVoltage;
  output->torqueLimit = input.torqueLimit;
  output->gearRatio = input.gearRatio;
}


void copy_leg(const cassie_leg_out_t& input, lcmt_cassie_leg_out* output) {
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

void CassieOutputSender::Output(const Context<double>& context,
                                     lcmt_cassie_out* output) const {
  // std::cout << "CassieOutputSender::Output t:" <<  context.get_time() << std::endl;
  const cassie_out_t& cassie_out =
    EvalAbstractInput(context, 0)->GetValue<cassie_out_t>();
  // using the time from the context
  output->utime = context.get_time() * 1e6;

  // copy pelvis
  copy_vector(cassie_out.pelvis.targetPc.etherCatStatus,
              output->pelvis.targetPc.etherCatStatus, 6);
  copy_vector(cassie_out.pelvis.targetPc.etherCatNotifications,
              output->pelvis.targetPc.etherCatNotifications, 21);
  output->pelvis.targetPc.taskExecutionTime =
      cassie_out.pelvis.targetPc.taskExecutionTime;
  output->pelvis.targetPc.overloadCounter =
      cassie_out.pelvis.targetPc.overloadCounter;
  output->pelvis.targetPc.cpuTemperature =
      cassie_out.pelvis.targetPc.cpuTemperature;

  output->pelvis.battery.dataGood =
      cassie_out.pelvis.battery.dataGood;
  output->pelvis.battery.stateOfCharge =
      cassie_out.pelvis.battery.stateOfCharge;
  output->pelvis.battery.current =
      cassie_out.pelvis.battery.current;
  copy_vector(cassie_out.pelvis.battery.voltage,
              output->pelvis.battery.voltage, 12);
  copy_vector(cassie_out.pelvis.battery.temperature,
              output->pelvis.battery.temperature, 4);

  output->pelvis.radio.radioReceiverSignalGood =
      cassie_out.pelvis.radio.radioReceiverSignalGood;
  output->pelvis.radio.receiverMedullaSignalGood =
      cassie_out.pelvis.radio.receiverMedullaSignalGood;
  copy_vector(cassie_out.pelvis.radio.channel,
              output->pelvis.radio.channel, 16);

  output->pelvis.vectorNav.dataGood =
      cassie_out.pelvis.vectorNav.dataGood;
  output->pelvis.vectorNav.vpeStatus =
      cassie_out.pelvis.vectorNav.vpeStatus;
  output->pelvis.vectorNav.pressure =
      cassie_out.pelvis.vectorNav.pressure;
  output->pelvis.vectorNav.temperature =
      cassie_out.pelvis.vectorNav.temperature;
  copy_vector(cassie_out.pelvis.vectorNav.magneticField,
              output->pelvis.vectorNav.magneticField, 3);
  copy_vector(cassie_out.pelvis.vectorNav.angularVelocity,
              output->pelvis.vectorNav.angularVelocity, 3);
  copy_vector(cassie_out.pelvis.vectorNav.linearAcceleration,
              output->pelvis.vectorNav.linearAcceleration, 3);
  copy_vector(cassie_out.pelvis.vectorNav.orientation,
              output->pelvis.vectorNav.orientation, 4);
  output->pelvis.medullaCounter = cassie_out.pelvis.medullaCounter;
  output->pelvis.medullaCpuLoad = cassie_out.pelvis.medullaCpuLoad;
  output->pelvis.bleederState = cassie_out.pelvis.bleederState;
  output->pelvis.leftReedSwitchState = cassie_out.pelvis.leftReedSwitchState;
  output->pelvis.rightReedSwitchState = cassie_out.pelvis.rightReedSwitchState;
  output->pelvis.vtmTemperature = cassie_out.pelvis.vtmTemperature;

  copy_leg(cassie_out.leftLeg, &output->leftLeg);
  copy_leg(cassie_out.rightLeg, &output->rightLeg);

  copy_vector(cassie_out.messages,
              output->messages, 4);
  output->isCalibrated = cassie_out.isCalibrated;
}

}  // namespace systems
}  // namespace dairlib
