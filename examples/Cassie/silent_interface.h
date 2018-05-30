#ifndef SILENT_INTERFACE_H
#define SILENT_INTERFACE_H
#include <functional>

template<typename T_in, typename T_out>
class SilentInterface : public PollingInterface<T_in, T_out> {
  public:
    SilentInterface() {}
    virtual void SetupChannel() {}
    virtual void StartPolling(std::function<void(T_out)> handler) {}
    virtual void StopPolling() {}
    virtual void Send(T_in message) {}
  protected:
    virtual T_out Receive() {return T_out();}
};
#endif
