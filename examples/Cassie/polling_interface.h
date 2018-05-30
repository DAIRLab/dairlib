#ifndef POLLING_INTERFACE_H
#define POLLING_INTERFACE_H
#include <functional>

template<typename T_in, typename T_out>
class PollingInterface {
  public:
    PollingInterface() {};
    virtual void SetupChannel();
    virtual void StartPolling(std::function<void(T_out)> handler) = 0;
    virtual void StopPolling() = 0;
    virtual void Send(T_in message) = 0;

    // set as inline to avoid linking errors...
    inline virtual ~PollingInterface() {};
  protected:
    virtual T_out Receive() = 0;
};

#endif
