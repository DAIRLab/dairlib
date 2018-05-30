#include "cassie_dispatch_types.h"
#include "cassie_names.h"

cassie_dispatch_lcm_in_t CassieRobotOutToLcmIn(cassie_dispatch_robot_out_t robot_out)
{
  const int CASSIE_NUM_JOINTS = cassieJointNames.size();

  cassie_dispatch_lcm_in_t lcm_in = cassie_dispatch_lcm_in_t();

  lcm_in.joint_names.resize(CASSIE_NUM_JOINTS);
  lcm_in.position.resize(CASSIE_NUM_JOINTS);
  lcm_in.velocity.resize(CASSIE_NUM_JOINTS);

  for (int i = 0; i < CASSIE_NUM_JOINTS; i++) {
    lcm_in.joint_names[i] = cassieJointNames[i];
  }

  lcm_in.num_joints = CASSIE_NUM_JOINTS;
  lcm_in.position[0] = robot_out.leftLeg.hipRollDrive.position;
  lcm_in.position[1] = robot_out.leftLeg.hipYawDrive.position;
  lcm_in.position[2] = robot_out.leftLeg.hipPitchDrive.position;
  lcm_in.position[3] = robot_out.leftLeg.kneeDrive.position;
  lcm_in.position[4] = robot_out.leftLeg.footDrive.position;
  lcm_in.position[5] = robot_out.rightLeg.hipRollDrive.position;
  lcm_in.position[6] = robot_out.rightLeg.hipYawDrive.position;
  lcm_in.position[7] = robot_out.rightLeg.hipPitchDrive.position;
  lcm_in.position[8] = robot_out.rightLeg.kneeDrive.position;
  lcm_in.position[9] = robot_out.rightLeg.footDrive.position;

  lcm_in.velocity[0] = robot_out.leftLeg.hipRollDrive.velocity;
  lcm_in.velocity[1] = robot_out.leftLeg.hipYawDrive.velocity;
  lcm_in.velocity[2] = robot_out.leftLeg.hipPitchDrive.velocity;
  lcm_in.velocity[3] = robot_out.leftLeg.kneeDrive.velocity;
  lcm_in.velocity[4] = robot_out.leftLeg.footDrive.velocity;
  lcm_in.velocity[5] = robot_out.rightLeg.hipRollDrive.velocity;
  lcm_in.velocity[6] = robot_out.rightLeg.hipYawDrive.velocity;
  lcm_in.velocity[7] = robot_out.rightLeg.hipPitchDrive.velocity;
  lcm_in.velocity[8] = robot_out.rightLeg.kneeDrive.velocity;
  lcm_in.velocity[9] = robot_out.rightLeg.footDrive.velocity;

  return lcm_in;
}

cassie_dispatch_robot_in_t CassieLcmOutToRobotIn(cassie_dispatch_lcm_out_t lcm_out)
{
  cassie_dispatch_robot_in_t robot_in = cassie_dispatch_robot_in_t();

  const int CASSIE_NUM_JOINTS = cassieJointNames.size();

  for (int i = 0; i < CASSIE_NUM_JOINTS; i++) {
    robot_in.torque[i] = lcm_out.inputs[i];
  }

  return robot_in;
}
