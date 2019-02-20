#pragma once

#include <string>
#include <vector>


// in cassie_user_in_t ordering


static std::vector<std::string> cassiePositionNames = {
"hip_roll_left",
"hip_yaw_left",
"hip_pitch_left",
"knee_left",
//"toe_crank_left",
"toe_left",
"knee_joint_left",
"ankle_joint_left",
"ankle_spring_joint_left",

"hip_roll_right",
"hip_yaw_right",
"hip_pitch_right",
"knee_right",
//"toe_crank_right",
"toe_right",
"knee_joint_right",
"ankle_joint_right",
"ankle_spring_joint_right",

};

static std::vector<std::string> cassieVelocityNames = {
"hip_roll_leftdot",
"hip_yaw_leftdot",
"hip_pitch_leftdot",
"knee_leftdot",
//"toe_crank_leftdot",
"toe_leftdot",
"knee_joint_leftdot",
"ankle_joint_leftdot",
"ankle_spring_joint_leftdot",


"hip_roll_rightdot",
"hip_yaw_rightdot",
"hip_pitch_rightdot",
"knee_rightdot",
//"toe_crank_rightdot",
"toe_rightdot",
"knee_joint_rightdot",
"ankle_joint_rightdot",
"ankle_spring_joint_rightdot",

};

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
