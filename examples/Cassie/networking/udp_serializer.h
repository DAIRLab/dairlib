#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "drake/systems/lcm/serializer.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/framework/value.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"

namespace dairlib {
namespace systems {

/**
 * %CassieUDPSerializer  translates between
 * UDP message bytes and cassie_out_t structs
 *
 */
class CassieUDPSerializer : public drake::systems::lcm::SerializerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieUDPSerializer)

  CassieUDPSerializer() {}
  ~CassieUDPSerializer() override {}

  std::unique_ptr<drake::systems::AbstractValue> CreateDefaultValue()
      const override {
    // NOTE: We create the message using value-initialization ("{}") to ensure
    // the POD fields are zeroed (instead of using default construction ("()"),
    // which would leave the POD data uninitialized.)
    return std::make_unique<drake::systems::Value<cassie_out_t>>(
        cassie_out_t{});
  }

  void Deserialize(
      const void* message_bytes, int message_length,
      drake::systems::AbstractValue* abstract_value) const override {
    DRAKE_DEMAND(abstract_value != nullptr);

    // Unpack received data into cassie output struct
    unpack_cassie_out_t(reinterpret_cast<const unsigned char *>(message_bytes),
        &abstract_value->GetMutableValue<cassie_out_t>());
  }

  void Serialize(const drake::systems::AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    DRAKE_DEMAND(message_bytes != nullptr);
    const cassie_user_in_t& message =
        abstract_value.GetValue<cassie_user_in_t>();
    message_bytes->resize(CASSIE_USER_IN_T_LEN + 2);
    pack_cassie_user_in_t(&message, &message_bytes->data()[2]);
  }
};

}  // namespace systems
}  // namespace dairlib
