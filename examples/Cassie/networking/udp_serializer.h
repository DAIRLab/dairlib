#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "drake/systems/lcm/serializer.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/value.h"
#include "examples/Cassie/datatypes/cassie_out_t.h"
#include "examples/Cassie/datatypes/cassie_user_in_t.h"

namespace dairlib {
namespace systems {

/**
 * %CassieUDPInSerializer  translates between
 * UDP message bytes and cassie_user_in_t structs
 *
 */
class CassieUDPInSerializer : public drake::systems::lcm::SerializerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CassieUDPInSerializer)

  CassieUDPInSerializer() {}
  ~CassieUDPInSerializer() override {}

  std::unique_ptr<drake::AbstractValue> CreateDefaultValue()
      const override {
    // NOTE: We create the message using value-initialization ("{}") to ensure
    // the POD fields are zeroed (instead of using default construction ("()"),
    // which would leave the POD data uninitialized.)
    return std::make_unique<drake::Value<cassie_user_in_t>>(
        cassie_user_in_t{});
  }

  void Deserialize(
      const void* message_bytes, int message_length,
      drake::AbstractValue* abstract_value) const override {
    throw std::logic_error(
        "CassieUDPInSerializer::Deserialize not yet implemented.");
  }

  void Serialize(const drake::AbstractValue& abstract_value,
                 std::vector<uint8_t>* message_bytes) const override {
    DRAKE_DEMAND(message_bytes != nullptr);
    const cassie_user_in_t& message =
        abstract_value.get_value<cassie_user_in_t>();
    message_bytes->resize(CASSIE_USER_IN_T_LEN + 2);

    pack_cassie_user_in_t(&message, &message_bytes->data()[2]);
  }

  std::unique_ptr<drake::systems::lcm::SerializerInterface> Clone() const  {
    return std::make_unique<CassieUDPInSerializer>();
  }
};


}  // namespace systems
}  // namespace dairlib
