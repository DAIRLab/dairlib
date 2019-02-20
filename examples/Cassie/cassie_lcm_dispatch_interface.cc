#include "cassie_lcm_dispatch_interface.h"

#include <vector>
#include <chrono>


//#include "templates/hdrs/umessage.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/common/value.h"
#include "drake/systems/lcm/serializer.h"

void CassieLcmDispatchInterface::LcmSubscribeHandler(const void* lcm_message_bytes, int lcm_message_length, std::function<void(cassie_dispatch_lcm_out_t)> handler)
{
  //std::cout <<  "subscription caught" << std::endl;std::cout.flush();
  drake::systems::lcm::Serializer<cassie_dispatch_lcm_out_t> serializer;

  auto lcm_message_abstract = serializer.CreateDefaultValue();

  serializer.Deserialize(lcm_message_bytes, lcm_message_length, lcm_message_abstract.get());
  //std::cout <<  "lcm  messaged processed" << std::endl;std::cout.flush();
  handler(lcm_message_abstract->GetValue<cassie_dispatch_lcm_out_t>());
  
}


void CassieLcmDispatchInterface::SetupChannel()
{
}

void CassieLcmDispatchInterface::StartPolling(std::function<void(cassie_dispatch_lcm_out_t)> handler)
{
  lcm->Subscribe(input_channel, [this, handler](const void* lcm_message_bytes, int lcm_message_length){
  this->LcmSubscribeHandler(lcm_message_bytes, lcm_message_length, handler);
  });
  lcm->StartReceiveThread();
}

void CassieLcmDispatchInterface::StopPolling()
{
   lcm->StopReceiveThread();
}

void CassieLcmDispatchInterface::Send(cassie_dispatch_lcm_in_t message)
{
  drake::Value<cassie_dispatch_lcm_in_t> lcm_message_val(message);
  drake::systems::lcm::Serializer<cassie_dispatch_lcm_in_t> serializer;
  std::vector<uint8_t> lcm_message_bytes;
  serializer.Serialize(lcm_message_val, &lcm_message_bytes);
  lcm->Publish(state_channel, lcm_message_bytes.data(), lcm_message_bytes.size(), static_cast<int>(time(NULL)));
}
