#include "cassie_lcm_dispatch_interface.h"
#include "cassie_robot_dispatch_interface.h"
#include "cassie_director_dispatch_interface.h"
#include "cassie_dispatcher.h"
#include "datatypes/cassie_names.h"
#include "datatypes/cassie_networking.h"
#include "datatypes/cassie_dispatch_types.h"
#include "drake/lcm/drake_lcm.h"
#include <chrono>
#include <iostream>
//template class PollingInterface<int, int>;

using namespace drake;
int main()
{
  drake::lcm::DrakeLcm lcm_local;
  drake::lcm::DrakeLcm lcm_broadcast ("udpm://239.255.76.68:7667?ttl=1");
  std::shared_ptr<CassieLcmDispatchInterface>
    lcmInterface{std::make_shared<CassieLcmDispatchInterface>(CASSIE_STATE_LCM_CHANNEL, CASSIE_INPUT_LCM_CHANNEL, &lcm_local)};
  std::shared_ptr<PollingInterface<cassie_dispatch_lcm_in_t, cassie_dispatch_lcm_out_t>> t1 = lcmInterface;
  std::shared_ptr<PollingInterface<cassie_dispatch_robot_in_t, cassie_dispatch_robot_out_t>>
    robotInterface{new CassieRobotDispatchInterface(CASSIE_UDP_LOCAL_ADDR, CASSIE_UDP_REMOTE_ADDR, CASSIE_UDP_LOCAL_PORT, CASSIE_UDP_REMOTE_PORT)};
  std::shared_ptr<PollingInterface<cassie_dispatch_director_in_t, cassie_dispatch_director_out_t>>
    directorInterface{new CassieDirectorDispatchInterface(CASSIE_TELEMETRY_LCM_CHANNEL, &lcm_broadcast)};
  CassieDispatcher cassieDispatcher(robotInterface, t1, directorInterface, 0.1);
  cassieDispatcher.Setup();
  cassieDispatcher.Run();
  return 0;
}

