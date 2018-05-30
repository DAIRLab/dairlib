#ifndef CASSIE_LCM_DISPATCH_INTERFACE_H
#define CASSIE_LCM_DISPATCH_INTERFACE_H
#include <functional>
#include <string>

#include "drake/lcm/drake_lcm.h"
#include "polling_interface.h"
#include "datatypes/cassie_dispatch_types.h"


class CassieLcmDispatchInterface : public PollingInterface<cassie_dispatch_lcm_in_t,
        cassie_dispatch_lcm_out_t>
{
  public:
    CassieLcmDispatchInterface(std::string schannel, std::string ichannel, drake::lcm::DrakeLcm * lcmp) :
      state_channel(schannel) , input_channel(ichannel), lcm(lcmp) {}
    virtual void SetupChannel();
    virtual void StartPolling(std::function<void(cassie_dispatch_lcm_out_t)> handler);
    virtual void StopPolling();
    virtual void Send(cassie_dispatch_lcm_in_t message);
  protected:
    virtual cassie_dispatch_lcm_out_t Receive() {return cassie_dispatch_lcm_out_t();}
    void LcmSubscribeHandler(const void* lcm_message_bytes, int lcm_message_length, std::function<void(cassie_dispatch_lcm_out_t)> handler);
    std::string state_channel;
    std::string input_channel;
    drake::lcm::DrakeLcm * lcm;
};
#endif
