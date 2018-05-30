#ifndef CASSIE_DISPATCH_TYPES_H
#define CASSIE_DISPATCH_TYPES_H
extern "C" {
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
}
#include "drake/lcmt_cassie_state.hpp"
#include "drake/lcmt_cassie_input.hpp"
typedef cassie_user_in_t cassie_dispatch_robot_in_t;
typedef cassie_out_t cassie_dispatch_robot_out_t;
//placeholders
typedef drake::lcmt_cassie_state cassie_dispatch_lcm_in_t;
typedef drake::lcmt_cassie_input cassie_dispatch_lcm_out_t;
typedef int cassie_dispatch_director_in_t;
typedef int cassie_dispatch_director_out_t;
typedef int cassie_dispatch_robot_state_t;
typedef int cassie_dispatch_director_state_t;

cassie_dispatch_lcm_in_t CassieRobotOutToLcmIn(cassie_dispatch_robot_out_t robot_out);
cassie_dispatch_robot_in_t CassieLcmOutToRobotIn(cassie_dispatch_lcm_out_t lcm_out);
#endif
