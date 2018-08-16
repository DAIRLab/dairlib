#ifndef CASSIE_DISPATCHER_H
#define CASSIE_DISPATCHER_H
#include <memory>
#include <chrono>
#include "dispatcher.h"
#include "datatypes/cassie_dispatch_types.h"

class CassieDispatcher : public Dispatcher<cassie_dispatch_robot_in_t,
        cassie_dispatch_robot_out_t,
        cassie_dispatch_lcm_in_t,
        cassie_dispatch_lcm_out_t,
        cassie_dispatch_director_in_t,
        cassie_dispatch_director_out_t,
        cassie_dispatch_robot_state_t,
        cassie_dispatch_director_state_t
        > {
  public:
    CassieDispatcher(std::shared_ptr<PollingInterface<cassie_dispatch_robot_in_t, cassie_dispatch_robot_out_t>> ri,
      std::shared_ptr<PollingInterface<cassie_dispatch_lcm_in_t, cassie_dispatch_lcm_out_t>> li,
      std::shared_ptr<PollingInterface<cassie_dispatch_director_in_t, cassie_dispatch_director_out_t>> di, double period) :
        Dispatcher<cassie_dispatch_robot_in_t,
        cassie_dispatch_robot_out_t,
        cassie_dispatch_lcm_in_t,
        cassie_dispatch_lcm_out_t,
        cassie_dispatch_director_in_t,
        cassie_dispatch_director_out_t,
        cassie_dispatch_robot_state_t,
        cassie_dispatch_director_state_t
        >(ri, li, di), 
        telemetry_send_period(std::chrono::duration<double>(period)) {}
    virtual void Setup();
    virtual void Run() ;//override;
  protected:
    virtual void robot_interface_handler(cassie_dispatch_robot_out_t robot_out) ;//override;
    virtual void lcm_interface_handler(cassie_dispatch_lcm_out_t lcm_out) ;//override;
    virtual void director_interface_handler(cassie_dispatch_director_out_t director_out) ;//override;
    
  private:
    void TelemetryUpdate();
    cassie_dispatch_director_state_t director_state;
    cassie_dispatch_robot_state_t robot_state;
    std::chrono::duration<double> telemetry_send_period;
    std::chrono::duration<double> next_telemetry_time;
};
#endif
