#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"

#include "systems/lcm/lcm_subscriber_multiplexer.h"

namespace dairlib {
namespace systems {
namespace {

using drake::systems::Context;
using drake::systems::SystemOutput;
using drake::lcmt_drake_signal;

struct SampleData {
  explicit SampleData(double x) : value({1, {x}, {"x"}, 12345}) {}

  lcmt_drake_signal value;

  void MockPublish(
      drake::lcm::DrakeMockLcm* lcm, const std::string& channel_name) const {
    const int num_bytes = value.getEncodedSize();
    std::vector<uint8_t> buffer(num_bytes);
    value.encode(buffer.data(), 0, num_bytes);
    lcm->InduceSubscriberCallback(channel_name, buffer.data(), num_bytes);
  }
};

class LcmSubscriberMultiplexerTest : public ::testing::Test {};

// Tests LcmSubscriberMultiplexer by encoding within a simulation
GTEST_TEST(LcmSubscriberMultiplexerTest, SerializerTest) {
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_1 = "channel_1";
  const std::string channel_2 = "channel_2";
  std::vector<std::string> channels = {channel_1, channel_2};
  drake::systems::DiagramBuilder<double> builder;
  auto mux =
      lcm::LcmSubscriberMultiplexer::MakeMuxAndSubscribers<lcmt_drake_signal>(
          &builder, channels, &lcm);

  double value_1 = 2.0;
  double value_2 = -3.5;

  // MockLcm produces sample messages on both channels
  SampleData sample_data_1(value_1);
  sample_data_1.MockPublish(&lcm, channel_1);

  SampleData sample_data_2(value_2);
  sample_data_2.MockPublish(&lcm, channel_2);

  auto diagram = builder.Build();
  drake::systems::Simulator<double> sim(*diagram);
  drake::systems::Context<double>& context = sim.get_mutable_context();

  drake::systems::Context<double>& mux_context =
      diagram->GetMutableSubsystemContext(*mux , &context);
  // Set initial value for channel
  mux_context.FixInputPort(mux->get_channel_input_port().get_index(),
      std::make_unique<drake::Value<std::string>>(channel_1));

  sim.Initialize();

  sim.StepTo(1);
  auto& context_1 = sim.get_mutable_context();
  std::unique_ptr<SystemOutput<double>> output_1 = mux->AllocateOutput();

  auto& mux_context_1 = diagram->GetMutableSubsystemContext(*mux , &context_1);
  mux->CalcOutput(mux_context_1, output_1.get());

  auto message_1 = output_1->get_data(0)->get_value<lcmt_drake_signal>();

  // Confirm that the output corresponds to channel_1
  EXPECT_EQ(value_1, message_1.val[0]);

  sample_data_1.MockPublish(&lcm, channel_1);
  sample_data_2.MockPublish(&lcm, channel_2);

  // Change the channel port to channel_2
  mux_context_1.FixInputPort(mux->get_channel_input_port().get_index(),
      std::make_unique<drake::Value<std::string>>(channel_2));

  sim.StepTo(2);
  auto& context_2 = sim.get_mutable_context();
  std::unique_ptr<SystemOutput<double>> output_2 = mux->AllocateOutput();

  auto& mux_context_2 = diagram->GetMutableSubsystemContext(*mux , &context_2);
  mux->CalcOutput(mux_context_2, output_2.get());

  auto message_2 = output_2->get_data(0)->get_value<lcmt_drake_signal>();

  // Confirm that the output corresponds to channel_2
  EXPECT_EQ(value_2, message_2.val[0]);
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
