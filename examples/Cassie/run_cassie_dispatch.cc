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
#include <gflags/gflags.h>
//template class PollingInterface<int, int>;


//using namespace drake;
namespace dairlib{

DEFINE_double(telemetry_period, 0.04, "The period that the dispatcher will telemeter robot state.");

int do_cassie_dispatch(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // construct local LCM object using default url
  drake::lcm::DrakeLcm lcm_local;

  // construct LCM object to communicate with basestation by using configurable URL
  drake::lcm::DrakeLcm lcm_broadcast(CASSIE_LCM_BROADCAST_URL);

  // construct LCM interface for Dispatch
  std::shared_ptr<CassieLcmDispatchInterface>
    lcmInterface{std::make_shared<CassieLcmDispatchInterface>(CASSIE_STATE_LCM_CHANNEL, CASSIE_INPUT_LCM_CHANNEL, &lcm_local)};
  std::shared_ptr<PollingInterface<cassie_dispatch_lcm_in_t, cassie_dispatch_lcm_out_t>> t1 = lcmInterface;
  
  // Cassie UDP interface
  std::shared_ptr<PollingInterface<cassie_dispatch_robot_in_t, cassie_dispatch_robot_out_t>>
    robotInterface{new CassieRobotDispatchInterface(CASSIE_UDP_LOCAL_ADDR, CASSIE_UDP_REMOTE_ADDR, CASSIE_UDP_LOCAL_PORT, CASSIE_UDP_REMOTE_PORT)};
  
  // basestation interface
  std::shared_ptr<PollingInterface<cassie_dispatch_director_in_t, cassie_dispatch_director_out_t>>
    directorInterface{new CassieDirectorDispatchInterface(CASSIE_TELEMETRY_LCM_CHANNEL, &lcm_broadcast)};
  

  // build dispatcher
  CassieDispatcher cassieDispatcher(robotInterface, t1, directorInterface, FLAGS_telemetry_period);
  cassieDispatcher.Setup();

  // run dispatcher indefinitely
  cassieDispatcher.Run();
  return 0;
}
}
int main(int argc, char* argv[]) {
  return dairlib::do_cassie_dispatch(argc, argv);
}