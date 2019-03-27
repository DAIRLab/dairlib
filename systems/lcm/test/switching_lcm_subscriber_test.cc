#include <gtest/gtest.h>

#include <memory>
#include <utility>

#include "drake/lcmt_drake_signal.hpp"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/systems/lcm/lcmt_drake_signal_translator.h"

#include "systems/lcm/switching_lcm_subscriber_system.h"
#include "dairlib/lcmt_subscriber_switch.hpp"

namespace dairlib {
namespace systems {
namespace {

using drake::systems::Context;
using drake::systems::SystemOutput;
using drake::lcmt_drake_signal;

struct SampleData {
  lcmt_drake_signal value{2, {1.0, 2.0}, {"x", "y"}, 12345};

  void MockPublish(
      drake::lcm::DrakeMockLcm* lcm, const std::string& channel_name) const {
    const int num_bytes = value.getEncodedSize();
    std::vector<uint8_t> buffer(num_bytes);
    value.encode(buffer.data(), 0, num_bytes);
    lcm->InduceSubscriberCallback(channel_name, buffer.data(), num_bytes);
  }
};

class SwitchingLcmSubscriberTest : public ::testing::Test {};

void SwitchChannel(drake::lcm::DrakeMockLcm* lcm,
    const std::string switching_channel, const std::string channel) {
  lcmt_subscriber_switch value{channel};
  const int num_bytes = value.getEncodedSize();
  std::vector<uint8_t> buffer(num_bytes);
  value.encode(buffer.data(), 0, num_bytes);
  lcm->InduceSubscriberCallback(switching_channel, buffer.data(), num_bytes);
}

// Tests SwitchingLcmSubscriberTest using a Serializer, checking the number
// of received messages matches properly.
GTEST_TEST(SwitchingLcmSubscriberTest, SerializerTest) {
  drake::lcm::DrakeMockLcm lcm;
  const std::string channel_name = "channel_name";
  const std::string new_channel_name = "new_channel";
  const std::string switching_channel_name = "switching_channel";
  // The "device under test".
  auto dut = lcm::SwitchingLcmSubscriberSystem::Make<lcmt_drake_signal>(
      channel_name, switching_channel_name, &lcm);

  // Establishes the context and output for the dut.
  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // MockLcm produces a sample message.
  SampleData sample_data;
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_EQ(1, dut->GetInternalMessageCount());

  SwitchChannel(&lcm, switching_channel_name, new_channel_name);
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_EQ(1, dut->GetInternalMessageCount());
  sample_data.MockPublish(&lcm, new_channel_name);
  EXPECT_EQ(2, dut->GetInternalMessageCount());

  SwitchChannel(&lcm, switching_channel_name, new_channel_name);
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_EQ(2, dut->GetInternalMessageCount());
  sample_data.MockPublish(&lcm, new_channel_name);
  EXPECT_EQ(3, dut->GetInternalMessageCount());

  SwitchChannel(&lcm, switching_channel_name, channel_name);
  sample_data.MockPublish(&lcm, channel_name);
  EXPECT_EQ(4, dut->GetInternalMessageCount());
  sample_data.MockPublish(&lcm, new_channel_name);
  EXPECT_EQ(4, dut->GetInternalMessageCount());
}

}  // namespace
}  // namespace systems
}  // namespace dairlib

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
