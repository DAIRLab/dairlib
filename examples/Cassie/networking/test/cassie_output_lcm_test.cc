#include <memory>
#include <numeric>
#include <utility>

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/Cassie/networking/cassie_output_sender.h"
#include "examples/Cassie/networking/cassie_output_receiver.h"

namespace dairlib {
namespace systems {
namespace {

using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::Context;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::make_unique;
using std::unique_ptr;
using drake::systems::System;
using drake::systems::SystemOutput;
using drake::systems::BasicVector;

class CassieOutputLcmTest : public ::testing::Test {
 public:
  void compareLeg(const cassie_leg_out_t& in, const cassie_leg_out_t& out) {
    compareElmo(in.hipRollDrive, out.hipRollDrive);
    compareElmo(in.hipYawDrive, out.hipYawDrive);
    compareElmo(in.hipPitchDrive, out.hipPitchDrive);
    compareElmo(in.kneeDrive, out.kneeDrive);
    compareElmo(in.footDrive, out.footDrive);

    ASSERT_EQ(in.shinJoint.position, out.shinJoint.position);
    ASSERT_EQ(in.shinJoint.velocity, out.shinJoint.velocity);
    ASSERT_EQ(in.tarsusJoint.position, out.tarsusJoint.position);
    ASSERT_EQ(in.tarsusJoint.velocity, out.tarsusJoint.velocity);
    ASSERT_EQ(in.footJoint.position, out.footJoint.position);
    ASSERT_EQ(in.footJoint.velocity, out.footJoint.velocity);

    ASSERT_EQ(in.medullaCounter, out.medullaCounter);
    ASSERT_EQ(in.medullaCpuLoad, out.medullaCpuLoad);
    ASSERT_EQ(in.reedSwitchState, out.reedSwitchState);
  }

  void compareElmo(const elmo_out_t& in, const elmo_out_t& out) {
    ASSERT_EQ(in.statusWord, out.statusWord);
    ASSERT_EQ(in.position, out.position);
    ASSERT_EQ(in.velocity, out.velocity);
    ASSERT_EQ(in.torque, out.torque);
    ASSERT_EQ(in.driveTemperature, out.driveTemperature);
    ASSERT_EQ(in.dcLinkVoltage, out.dcLinkVoltage);
    ASSERT_EQ(in.torqueLimit, out.torqueLimit);
    ASSERT_EQ(in.gearRatio, out.gearRatio);
  }
};

TEST_F(CassieOutputLcmTest, SendReceiveTest) {
  DiagramBuilder<double> builder;

  // Create publisher.
  auto sender = builder.AddSystem<systems::CassieOutputSender>();
  auto receiver = builder.AddSystem<systems::CassieOutputReceiver>();
  // connect state publisher
  builder.Connect(sender->get_output_port(0), receiver->get_input_port(0));

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  cassie_out_t input_struct{};

  // fill input_struct with consecutive numbers
  memset(&input_struct, 2, sizeof(input_struct));
  unsigned char* p = (unsigned char *) &input_struct;
  for (uint i = 0; i < sizeof(input_struct); i++) {
    p[i] = i % 63;
  }

  auto& sender_context = diagram->GetMutableSubsystemContext(
      *sender, diagram_context.get());
  auto& receiver_context = diagram->GetMutableSubsystemContext(
      *receiver, diagram_context.get());

  sender_context.FixInputPort(sender->get_input_port(0).get_index(),
      std::make_unique<drake::Value<cassie_out_t>>(input_struct));

  auto output = receiver->AllocateOutput();
  receiver->CalcOutput(receiver_context, output.get());

  const cassie_out_t& output_struct =
      output->get_data(0)->get_value<cassie_out_t>();

  // cannot memcmp the structs because of buffers between used data
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.targetPc.etherCatStatus,
               &output_struct.pelvis.targetPc.etherCatStatus,
               sizeof(input_struct.pelvis.targetPc.etherCatStatus)));
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.targetPc.etherCatNotifications,
               &output_struct.pelvis.targetPc.etherCatNotifications,
               sizeof(input_struct.pelvis.targetPc.etherCatNotifications)));
  ASSERT_EQ(input_struct.pelvis.targetPc.taskExecutionTime,
            output_struct.pelvis.targetPc.taskExecutionTime);
  ASSERT_EQ(input_struct.pelvis.targetPc.overloadCounter,
            output_struct.pelvis.targetPc.overloadCounter);
  ASSERT_EQ(input_struct.pelvis.targetPc.cpuTemperature,
            output_struct.pelvis.targetPc.cpuTemperature);

  ASSERT_EQ(input_struct.pelvis.battery.dataGood,
            output_struct.pelvis.battery.dataGood);
  ASSERT_EQ(input_struct.pelvis.battery.stateOfCharge,
            output_struct.pelvis.battery.stateOfCharge);
  ASSERT_EQ(input_struct.pelvis.battery.current,
            output_struct.pelvis.battery.current);
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.battery.voltage,
               &output_struct.pelvis.battery.voltage,
               sizeof(input_struct.pelvis.battery.voltage)));
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.battery.temperature,
               &output_struct.pelvis.battery.temperature,
               sizeof(input_struct.pelvis.battery.temperature)));

  ASSERT_EQ(input_struct.pelvis.radio.radioReceiverSignalGood,
            output_struct.pelvis.radio.radioReceiverSignalGood);
  ASSERT_EQ(input_struct.pelvis.radio.receiverMedullaSignalGood,
            output_struct.pelvis.radio.receiverMedullaSignalGood);
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.radio.channel,
               &output_struct.pelvis.radio.channel,
               sizeof(input_struct.pelvis.radio.channel)));

  ASSERT_EQ(input_struct.pelvis.vectorNav.dataGood,
            output_struct.pelvis.vectorNav.dataGood);
  ASSERT_EQ(input_struct.pelvis.vectorNav.vpeStatus,
            output_struct.pelvis.vectorNav.vpeStatus);
  ASSERT_EQ(input_struct.pelvis.vectorNav.pressure,
            output_struct.pelvis.vectorNav.pressure);
  ASSERT_EQ(input_struct.pelvis.vectorNav.temperature,
            output_struct.pelvis.vectorNav.temperature);
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.vectorNav.magneticField,
               &output_struct.pelvis.vectorNav.magneticField,
               sizeof(input_struct.pelvis.vectorNav.magneticField)));
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.vectorNav.angularVelocity,
               &output_struct.pelvis.vectorNav.angularVelocity,
               sizeof(input_struct.pelvis.vectorNav.angularVelocity)));
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.vectorNav.linearAcceleration,
               &output_struct.pelvis.vectorNav.linearAcceleration,
               sizeof(input_struct.pelvis.vectorNav.linearAcceleration)));
  ASSERT_EQ(0, memcmp(&input_struct.pelvis.vectorNav.orientation,
               &output_struct.pelvis.vectorNav.orientation,
               sizeof(input_struct.pelvis.vectorNav.orientation)));

  ASSERT_EQ(input_struct.pelvis.medullaCounter,
            output_struct.pelvis.medullaCounter);
  ASSERT_EQ(input_struct.pelvis.medullaCpuLoad,
            output_struct.pelvis.medullaCpuLoad);
  ASSERT_EQ(input_struct.pelvis.bleederState,
            output_struct.pelvis.bleederState);
  ASSERT_EQ(input_struct.pelvis.leftReedSwitchState,
            output_struct.pelvis.leftReedSwitchState);
  ASSERT_EQ(input_struct.pelvis.rightReedSwitchState,
            output_struct.pelvis.rightReedSwitchState);
  ASSERT_EQ(input_struct.pelvis.vtmTemperature,
            output_struct.pelvis.vtmTemperature);

  ASSERT_EQ(input_struct.isCalibrated, output_struct.isCalibrated);
  ASSERT_EQ(0, memcmp(&input_struct.messages,
               &output_struct.messages,
               sizeof(input_struct.messages)));

  compareLeg(input_struct.leftLeg, output_struct.leftLeg);
  compareLeg(input_struct.rightLeg, output_struct.rightLeg);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
