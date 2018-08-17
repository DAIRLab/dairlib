#include "cassie_dispatcher.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_state.hpp"
#include "cassie_director_dispatch_interface.h"
#include <iostream>
#include <chrono>



// callback for received Cassie UDP message
void CassieDispatcher::robot_interface_handler(cassie_dispatch_robot_out_t robot_out)
{

  // construct LCM object 
  cassie_dispatch_lcm_in_t lcm_in = CassieRobotOutToLcmIn(robot_out);

  // update last received robot information
  this->robot_state.last_robot_out = lcm_in;

  // signal updated robot state to telemetry management
  this->TelemetryUpdate();

  // send lcm message
  this->lcm_interface->Send(lcm_in);
}

void CassieDispatcher::lcm_interface_handler(cassie_dispatch_lcm_out_t lcm_out)
{
  // construct Agility UDP struct for torque command
  cassie_dispatch_robot_in_t robot_in = CassieLcmOutToRobotIn(lcm_out);
  

  // send torque command to motors
  this->robot_interface->Send(robot_in);
  
  // update last sent robot command
  this->robot_state.last_robot_in = lcm_out;

  // signal updated robot state to telemetry management
  this->TelemetryUpdate();
}


// callback for messages received from basestation; currently unimplemented as we have no direct-to-cassie overrides.
void CassieDispatcher::director_interface_handler(cassie_dispatch_director_out_t director_out)
{

}


// setup function for communication channels
void CassieDispatcher::Setup()
{
  Dispatcher<cassie_dispatch_robot_in_t,
        cassie_dispatch_robot_out_t,
        cassie_dispatch_lcm_in_t,
        cassie_dispatch_lcm_out_t,
        cassie_dispatch_director_in_t,
        cassie_dispatch_director_out_t,
        cassie_dispatch_robot_state_t,
        cassie_dispatch_director_state_t
        >::Setup();
  robot_state.last_robot_out = cassie_dispatch_lcm_in_t();
  robot_state.last_robot_in = cassie_dispatch_lcm_out_t();
  this->next_telemetry_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch());
}


// starts polling on interfaces
void CassieDispatcher::Run()
{
  robot_interface->StartPolling([this](cassie_dispatch_robot_out_t robot_out){this->robot_interface_handler(robot_out);});
  lcm_interface->StartPolling([this](cassie_dispatch_lcm_out_t lcm_out){this->lcm_interface_handler(lcm_out);});
  director_interface->StartPolling([this](cassie_dispatch_director_out_t director_out){this->director_interface_handler(director_out);});
  while (1) {}
}


// callback when robot information is updated; decides whether or not basestation gets update
void CassieDispatcher::TelemetryUpdate()
{
  auto cur_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch());

  // if telemetered state is stale
  if (this->next_telemetry_time < cur_time)
  {

    // construct new telemetry message
    cassie_dispatch_director_in_t director_in = CassieRobotStateToDirectorIn(&this->robot_state);
    
    // send to basestation
    this->director_interface->Send(director_in);

    // wait until the next requested time
    this->next_telemetry_time += this->telemetry_send_period;
  }
}