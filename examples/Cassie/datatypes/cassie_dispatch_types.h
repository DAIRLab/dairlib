#ifndef CASSIE_DISPATCH_TYPES_H
#define CASSIE_DISPATCH_TYPES_H
extern "C" {
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
}
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_state.hpp"
typedef cassie_user_in_t cassie_dispatch_robot_in_t;
typedef cassie_out_t cassie_dispatch_robot_out_t;
//placeholders
typedef dairlib::lcmt_robot_output cassie_dispatch_lcm_in_t;
typedef dairlib::lcmt_robot_input cassie_dispatch_lcm_out_t;
typedef dairlib::lcmt_robot_state cassie_dispatch_director_in_t;
typedef int cassie_dispatch_director_out_t;
typedef struct {
	cassie_dispatch_lcm_in_t last_robot_out;
	cassie_dispatch_lcm_out_t last_robot_in;

} cassie_dispatch_robot_state_t;
typedef int cassie_dispatch_director_state_t;

cassie_dispatch_lcm_in_t CassieRobotOutToLcmIn(cassie_dispatch_robot_out_t robot_out);
cassie_dispatch_robot_in_t CassieLcmOutToRobotIn(cassie_dispatch_lcm_out_t lcm_out);
cassie_dispatch_lcm_out_t CassieRobotInToLcmOut(cassie_dispatch_robot_in_t robot_in);
cassie_dispatch_robot_out_t CassieLcmInToRobotOut(cassie_dispatch_lcm_in_t lcm_in);
cassie_dispatch_director_in_t CassieRobotStateToDirectorIn(cassie_dispatch_robot_state_t * robot_state);
#endif
