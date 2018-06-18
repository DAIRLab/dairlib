#include "cassie_dispatcher.h"
#include <iostream>

void CassieDispatcher::robot_interface_handler(cassie_dispatch_robot_out_t robot_out)
{
  std::cout << "Robot State Received!" << std::endl;std::cout.flush();
  cassie_dispatch_lcm_in_t lcm_in = CassieRobotOutToLcmIn(robot_out);
  this->lcm_interface->Send(lcm_in);
  std::cout << "LCM State Sent!" << std::endl;std::cout.flush();
}

void CassieDispatcher::lcm_interface_handler(cassie_dispatch_lcm_out_t lcm_out)
{
  std::cout << "LCM Input Received!" << std::endl;std::cout.flush();
  //std::cout <<  "converting LCM/ROBOT type" << std::endl;std::cout.flush();
  cassie_dispatch_robot_in_t robot_in = CassieLcmOutToRobotIn(lcm_out);
  //std::cout <<  "finished conversion" << std::endl;std::cout.flush();
  this->robot_interface->Send(robot_in);
  std::cout << "Robot Input Sent!" << std::endl;std::cout.flush();
}

void CassieDispatcher::director_interface_handler(cassie_dispatch_director_out_t director_out)
{

}

void CassieDispatcher::Run()
{
  robot_interface->StartPolling([this](cassie_dispatch_robot_out_t robot_out){this->robot_interface_handler(robot_out);});
  lcm_interface->StartPolling([this](cassie_dispatch_lcm_out_t lcm_out){this->lcm_interface_handler(lcm_out);});
  director_interface->StartPolling([this](cassie_dispatch_director_out_t director_out){this->director_interface_handler(director_out);});
  while (1) {}
}
