#include "cassie_director_dispatch_interface.h"

#include <vector>
#include <chrono>


#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/lcm/serializer.h"


void CassieDirectorDispatchInterface::SetupChannel()
{
  
}

void CassieDirectorDispatchInterface::StartPolling(std::function<void(cassie_dispatch_director_out_t)> handler)
{

}

void CassieDirectorDispatchInterface::StopPolling()
{
   
}

void CassieDirectorDispatchInterface::Send(cassie_dispatch_director_in_t message)
{
  drake::systems::Value<cassie_dispatch_director_in_t> lcm_message_val(message);
  drake::systems::lcm::Serializer<cassie_dispatch_director_in_t> serializer;
  std::vector<uint8_t> lcm_message_bytes;
  serializer.Serialize(lcm_message_val, &lcm_message_bytes);
  lcm->Publish(telemetry_channel, lcm_message_bytes.data(), lcm_message_bytes.size(), static_cast<int>(time(NULL)));
}