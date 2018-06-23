#pragma once

#include <mutex>
#include <memory>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"

#include "drake/systems/lcm/serializer.h"

#include "cassie_udp_spoofer.h"
#include "datatypes/cassie_dispatch_types.h"
#include "datatypes/cassie_names.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"

namespace dairlib {
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::PublishEvent;
class CassieUdpOutputPublisher : public LeafSystem<double> {
 public:
  CassieUdpOutputPublisher(std::shared_ptr<CassieUdpSpoofer> spoofer);
  void set_publish_period(double period);
 private:
  void DoPublish(const Context<double>& context,
               const std::vector<const PublishEvent<double>*>&) const;
  std::shared_ptr<CassieUdpSpoofer> _spoofer;
};

class CassieUdpInputSubscriber : public LeafSystem<double> {
 public:
  CassieUdpInputSubscriber(std::shared_ptr<CassieUdpSpoofer> spoofer);
  //has no inputs
  void get_input_port(int) = delete;

 private:
  void Output(const Context<double>& context,
                    lcmt_robot_input* output) const;

  mutable std::mutex mux;
   std::unique_ptr<lcmt_robot_input> last_received;
  //std::unique_ptr<SerializerInterface> _serializer;
  void ProcessInput(cassie_dispatch_robot_in_t new_input);
};

}
