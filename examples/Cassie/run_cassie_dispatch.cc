#include "cassie_lcm_dispatch_interface.h"
#include "cassie_robot_dispatch_interface.h"
#include "cassie_dispatcher.h"
#include "silent_interface.h"
#include "datatypes/cassie_names.h"
#include "datatypes/cassie_networking.h"
#include "datatypes/cassie_dispatch_types.h"
#include "drake/lcm/drake_lcm.h"
#include <chrono>
#include <iostream>
//template class PollingInterface<int, int>;

int main()
{
  drake::lcm::DrakeLcm lcm;
  std::shared_ptr<CassieLcmDispatchInterface>
    lcmInterface{std::make_shared<CassieLcmDispatchInterface>(CASSIE_STATE_LCM_CHANNEL, CASSIE_INPUT_LCM_CHANNEL, &lcm)};
  std::shared_ptr<PollingInterface<cassie_dispatch_lcm_in_t, cassie_dispatch_lcm_out_t>> t1 = lcmInterface;
  std::shared_ptr<PollingInterface<cassie_dispatch_robot_in_t, cassie_dispatch_robot_out_t>>
    robotInterface{new CassieRobotDispatchInterface(CASSIE_UDP_LOCAL_ADDR, CASSIE_UDP_REMOTE_ADDR, CASSIE_UDP_LOCAL_PORT, CASSIE_UDP_REMOTE_PORT)};
  std::shared_ptr<PollingInterface<cassie_dispatch_director_in_t, cassie_dispatch_director_out_t>>
    directorInterface{new SilentInterface<cassie_dispatch_director_in_t, cassie_dispatch_director_out_t>()};
  CassieDispatcher cassieDispatcher(robotInterface, t1, directorInterface);
  cassieDispatcher.Setup();
  cassieDispatcher.Run();
  return 0;
}

