#ifndef DISPATCHER_H
#define DISPATCHER_H

#include <memory>

#include "polling_interface.h"
#include "datatypes/cassie_dispatch_types.h"

template<typename Robot_in, typename Robot_out,
         typename LCM_in, typename LCM_out,
         typename Director_in, typename Director_out,
         typename Robot_state, typename Director_state>
class Dispatcher {
  public:
    Dispatcher(std::shared_ptr<PollingInterface<Robot_in, Robot_out>> ri,
      std::shared_ptr<PollingInterface<LCM_in, LCM_out>> li,
      std::shared_ptr<PollingInterface<Director_in, Director_out>> di) :
      robot_interface(ri), lcm_interface(li), director_interface(di) {}

    virtual void Setup()
    {
      robot_interface->SetupChannel();
      lcm_interface->SetupChannel();
      director_interface->SetupChannel();
    }

    /*inline virtual void Run()
    {
      robot_interface->StartPolling([this](Robot_out robot_out){this->robot_interface_handler(robot_out);});
      lcm_interface->StartPolling([this](LCM_out lcm_out){this->lcm_interface_handler(lcm_out);});
      director_interface->StartPolling([this](Director_out director_out){this->director_interface_handler(director_out);});
      while (1) {}
    }*/
    virtual ~Dispatcher(){}
  protected:
    std::shared_ptr<PollingInterface<Robot_in, Robot_out>> robot_interface;
    std::shared_ptr<PollingInterface<LCM_in, LCM_out>> lcm_interface;
    std::shared_ptr<PollingInterface<Director_in, Director_out>> director_interface;
    /*virtual void robot_interface_handler(Robot_out robot_out);
    virtual void lcm_interface_handler(LCM_out lcm_out);
    virtual void director_interface_handler(Director_out director_out);*/
    Robot_state robot_state;
    Director_state director_state;
};

#endif
