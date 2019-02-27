#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"
#include "drake/systems/framework/diagram_builder.h"

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

// Tests SwitchingLcmSubscriberTest using a Serializer, checking the number
// of received messages matches properly.
GTEST_TEST(LcmSubscriberMultiplexerTest, SerializerTest) {
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";
  const std::string new_channel_name = "new_channel";
  std::vector<std::string> channels = {channel_name, new_channel_name};
  drake::systems::DiagramBuilder<double> builder;
  auto mux =
      lcm::LcmSubscriberMultiplexer::MakeMuxAndSubscribers<lcmt_drake_signal>(
          &builder, channels, &lcm);

  // Establishes the context and output for the mux
  std::unique_ptr<Context<double>> context = mux->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = mux->AllocateOutput();

  // Set initial value for channel
  context->FixInputPort(mux->get_channel_input_port().get_index(),
      std::make_unique<drake::Value<std::string>>(channel_name));

  // MockLcm produces a sample message.
  SampleData sample_data(1.0);
  sample_data.MockPublish(&lcm, channel_name);
  mux->CalcOutput(*context, output.get());
  // EXPECT_EQ(1, dut->GetInternalMessageCount());

  // SwitchChannel(&lcm, switching_channel_name, new_channel_name);
  // sample_data.MockPublish(&lcm, channel_name);
  // EXPECT_EQ(1, dut->GetInternalMessageCount());
  // sample_data.MockPublish(&lcm, new_channel_name);
  // EXPECT_EQ(2, dut->GetInternalMessageCount());

  // SwitchChannel(&lcm, switching_channel_name, new_channel_name);
  // sample_data.MockPublish(&lcm, channel_name);
  // EXPECT_EQ(2, dut->GetInternalMessageCount());
  // sample_data.MockPublish(&lcm, new_channel_name);
  // EXPECT_EQ(3, dut->GetInternalMessageCount());

  // SwitchChannel(&lcm, switching_channel_name, channel_name);
  // sample_data.MockPublish(&lcm, channel_name);
  // EXPECT_EQ(4, dut->GetInternalMessageCount());
  // sample_data.MockPublish(&lcm, new_channel_name);
  // EXPECT_EQ(4, dut->GetInternalMessageCount());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
