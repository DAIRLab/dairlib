#include "examples/Cassie/networking/udp_lcm_translator.h"

namespace dairlib {

void copy_elmo(const elmo_out_t& input, lcmt_elmo_out* output);
void copy_leg(const cassie_leg_out_t& input, lcmt_cassie_leg_out* output);
template <typename T> void copy_vector(const T* input, T* output, int size);

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

void cassieOutFromLcm(const lcmt_cassie_out& message,
    cassie_out_t* cassie_out) {
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

  // Testing
  // TODO(yminchen): delete this after you resolve the issue of delayed update
  // cassie_out->pelvis.targetPc.taskExecutionTime = message.utime*1e-6;
}

void cassieOutToLcm(const cassie_out_t& cassie_out, double time_seconds,
    lcmt_cassie_out* message) {
  message->utime = time_seconds * 1e6;

  // copy pelvis
  copy_vector(cassie_out.pelvis.targetPc.etherCatStatus,
              message->pelvis.targetPc.etherCatStatus, 6);
  copy_vector(cassie_out.pelvis.targetPc.etherCatNotifications,
              message->pelvis.targetPc.etherCatNotifications, 21);

  message->pelvis.targetPc.taskExecutionTime =
      cassie_out.pelvis.targetPc.taskExecutionTime;
  message->pelvis.targetPc.overloadCounter =
      cassie_out.pelvis.targetPc.overloadCounter;

  message->pelvis.targetPc.cpuTemperature =
      cassie_out.pelvis.targetPc.cpuTemperature;

  message->pelvis.battery.dataGood =
      cassie_out.pelvis.battery.dataGood;
  message->pelvis.battery.stateOfCharge =
      cassie_out.pelvis.battery.stateOfCharge;
  message->pelvis.battery.current =
      cassie_out.pelvis.battery.current;
  copy_vector(cassie_out.pelvis.battery.voltage,
              message->pelvis.battery.voltage, 12);
  copy_vector(cassie_out.pelvis.battery.temperature,
              message->pelvis.battery.temperature, 4);

  message->pelvis.radio.radioReceiverSignalGood =
      cassie_out.pelvis.radio.radioReceiverSignalGood;
  message->pelvis.radio.receiverMedullaSignalGood =
      cassie_out.pelvis.radio.receiverMedullaSignalGood;
  copy_vector(cassie_out.pelvis.radio.channel,
              message->pelvis.radio.channel, 16);

  message->pelvis.vectorNav.dataGood =
      cassie_out.pelvis.vectorNav.dataGood;
  message->pelvis.vectorNav.vpeStatus =
      cassie_out.pelvis.vectorNav.vpeStatus;
  message->pelvis.vectorNav.pressure =
      cassie_out.pelvis.vectorNav.pressure;
  message->pelvis.vectorNav.temperature =
      cassie_out.pelvis.vectorNav.temperature;
  copy_vector(cassie_out.pelvis.vectorNav.magneticField,
              message->pelvis.vectorNav.magneticField, 3);
  copy_vector(cassie_out.pelvis.vectorNav.angularVelocity,
              message->pelvis.vectorNav.angularVelocity, 3);
  copy_vector(cassie_out.pelvis.vectorNav.linearAcceleration,
              message->pelvis.vectorNav.linearAcceleration, 3);
  copy_vector(cassie_out.pelvis.vectorNav.orientation,
              message->pelvis.vectorNav.orientation, 4);
  message->pelvis.medullaCounter = cassie_out.pelvis.medullaCounter;
  message->pelvis.medullaCpuLoad = cassie_out.pelvis.medullaCpuLoad;
  message->pelvis.bleederState = cassie_out.pelvis.bleederState;
  message->pelvis.leftReedSwitchState = cassie_out.pelvis.leftReedSwitchState;
  message->pelvis.rightReedSwitchState = cassie_out.pelvis.rightReedSwitchState;
  message->pelvis.vtmTemperature = cassie_out.pelvis.vtmTemperature;

  copy_leg(cassie_out.leftLeg, &message->leftLeg);
  copy_leg(cassie_out.rightLeg, &message->rightLeg);

  copy_vector(cassie_out.messages,
              message->messages, 4);

  message->isCalibrated = cassie_out.isCalibrated;
}


void cassieInToLcm(const cassie_user_in_t& cassie_in, double time_seconds,
    lcmt_cassie_in* message) {
  message->utime = time_seconds * 1e6;

  copy_vector(cassie_in.torque, message->torque, 10);
  copy_vector(cassie_in.telemetry, message->telemetry, 9);
}

void cassieInFromLcm(const lcmt_cassie_in& message,
    cassie_user_in_t* cassie_in) {
  copy_vector(message.torque, cassie_in->torque, 10);
  copy_vector(message.telemetry, cassie_in->telemetry, 9);
}


}  // namespace dairlib
