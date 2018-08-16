#ifndef CASSIE_DIRECTOR_DISPATCH_INTERFACE_H
#define CASSIE_DIRECTOR_DISPATCH_INTERFACE_H
#include <functional>
#include <string>
#include <chrono>

#include "drake/lcm/drake_lcm.h"
#include "polling_interface.h"
#include "datatypes/cassie_dispatch_types.h"


class CassieDirectorDispatchInterface : public PollingInterface<cassie_dispatch_director_in_t,
        cassie_dispatch_director_out_t>
{
  public:
    CassieDirectorDispatchInterface(std::string tchannel, drake::lcm::DrakeLcm * lcmp) :
      telemetry_channel(tchannel), lcm(lcmp) {}
    virtual void SetupChannel();
    virtual void StartPolling(std::function<void(cassie_dispatch_director_out_t)> handler);
    virtual void StopPolling();
    virtual void Send(cassie_dispatch_director_in_t message);
  protected:
    virtual cassie_dispatch_director_out_t Receive() {return cassie_dispatch_director_out_t();}
    std::string telemetry_channel;
    drake::lcm::DrakeLcm * lcm;
};
#endif
