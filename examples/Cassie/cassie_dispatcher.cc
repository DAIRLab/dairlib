#include "cassie_dispatcher.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_state.hpp"
#include "cassie_director_dispatch_interface.h"
#include <iostream>
#include <chrono>

void CassieDispatcher::robot_interface_handler(cassie_dispatch_robot_out_t robot_out)
{
  //std::cout << "Robot State Received!" << std::endl;std::cout.flush();
  cassie_dispatch_lcm_in_t lcm_in = CassieRobotOutToLcmIn(robot_out);

  this->robot_state.last_robot_out = lcm_in;

  this->TelemetryUpdate();

  this->lcm_interface->Send(lcm_in);
  //std::cout << "LCM State Sent!" << std::endl;std::cout.flush();
}

void CassieDispatcher::lcm_interface_handler(cassie_dispatch_lcm_out_t lcm_out)
{
  //std::cout << "LCM Input Received!" << std::endl;std::cout.flush();
  //std::cout <<  "converting LCM/ROBOT type" << std::endl;std::cout.flush();
  cassie_dispatch_robot_in_t robot_in = CassieLcmOutToRobotIn(lcm_out);
  //std::cout <<  "finished conversion" << std::endl;std::cout.flush();
  this->robot_interface->Send(robot_in);
  //std::cout << "Robot Input Sent!" << std::endl;std::cout.flush();
  this->robot_state.last_robot_in = lcm_out;

  this->TelemetryUpdate();
}

void CassieDispatcher::director_interface_handler(cassie_dispatch_director_out_t director_out)
{

}

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


void CassieDispatcher::Run()
{
  robot_interface->StartPolling([this](cassie_dispatch_robot_out_t robot_out){this->robot_interface_handler(robot_out);});
  lcm_interface->StartPolling([this](cassie_dispatch_lcm_out_t lcm_out){this->lcm_interface_handler(lcm_out);});
  director_interface->StartPolling([this](cassie_dispatch_director_out_t director_out){this->director_interface_handler(director_out);});
  while (1) {}
}


void CassieDispatcher::TelemetryUpdate()
{
  auto cur_time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch());
  if (this->next_telemetry_time < cur_time)
  {
    cassie_dispatch_director_in_t director_in = CassieRobotStateToDirectorIn(&this->robot_state);
    this->director_interface->Send(director_in);
    this->next_telemetry_time += this->telemetry_send_period;
  }
}