#pragma once

#include <string>
#include <vector>


// in cassie_user_in_t ordering
// must be hardcoded as cassie_user_in_t uses a vector without names
// if the URDF changes, these must also change
static std::vector<std::string> cassieEffortNames = {
"hip_roll_left_motor",
"hip_yaw_left_motor",
"hip_pitch_left_motor",
"knee_left_motor",
"toe_left_motor",
"hip_roll_right_motor",
"hip_yaw_right_motor",
"hip_pitch_right_motor",
"knee_right_motor",
"toe_right_motor"
};
