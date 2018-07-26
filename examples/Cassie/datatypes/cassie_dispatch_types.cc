#include "cassie_dispatch_types.h"
#include "cassie_names.h"
#include <vector>
#include <chrono>

#define CASSIE_NUM_POS 16
#define CASSIE_NUM_VEL 16
#define CASSIE_NUM_EFF 10

cassie_dispatch_lcm_in_t CassieRobotOutToLcmIn(cassie_dispatch_robot_out_t robot_out)
{
    
    cassie_dispatch_lcm_in_t lcm_in = cassie_dispatch_lcm_in_t();

    lcm_in.position.resize(CASSIE_NUM_POS);
    lcm_in.velocity.resize(CASSIE_NUM_VEL);
    lcm_in.effort.resize(CASSIE_NUM_EFF);
    
    lcm_in.position_names.resize(CASSIE_NUM_POS);
    lcm_in.velocity_names.resize(CASSIE_NUM_VEL);
    lcm_in.effort_names.resize(CASSIE_NUM_EFF);
    
    lcm_in.num_positions = CASSIE_NUM_POS;
    lcm_in.num_velocities = CASSIE_NUM_VEL;
    lcm_in.num_efforts = CASSIE_NUM_EFF;
    lcm_in.timestamp = 0;
    
    for (int i = 0; i < CASSIE_NUM_POS; i++) {
        lcm_in.position_names[i] = cassiePositionNames[i];
    }
    
    for (int i = 0; i < CASSIE_NUM_VEL; i++) {
        lcm_in.velocity_names[i] = cassieVelocityNames[i];
    }
    
    for (int i = 0; i < CASSIE_NUM_EFF; i++) {
        lcm_in.effort_names[i] = cassieEffortNames[i];
    }
  
    lcm_in.position[0] = robot_out.leftLeg.hipRollDrive.position;
    lcm_in.position[1] = robot_out.leftLeg.hipYawDrive.position;
    lcm_in.position[2] = robot_out.leftLeg.hipPitchDrive.position;
    lcm_in.position[3] = robot_out.leftLeg.kneeDrive.position;
    lcm_in.position[4] = robot_out.leftLeg.footDrive.position;
    lcm_in.position[5] = robot_out.leftLeg.shinJoint.position;
    lcm_in.position[6] = robot_out.leftLeg.tarsusJoint.position;
    lcm_in.position[7] = 0.0;//robot_out.leftLeg.footJoint.position;
    lcm_in.position[8] = robot_out.rightLeg.hipRollDrive.position;
    lcm_in.position[9] = robot_out.rightLeg.hipYawDrive.position;
    lcm_in.position[10] = robot_out.rightLeg.hipPitchDrive.position;
    lcm_in.position[11] = robot_out.rightLeg.kneeDrive.position;
    lcm_in.position[12] = robot_out.rightLeg.footDrive.position;
    lcm_in.position[13] = robot_out.rightLeg.shinJoint.position;
    lcm_in.position[14] = robot_out.rightLeg.tarsusJoint.position;
    lcm_in.position[15] = 0.0;//robot_out.rightLeg.footJoint.position;

    lcm_in.velocity[0] = robot_out.leftLeg.hipRollDrive.velocity;
    lcm_in.velocity[1] = robot_out.leftLeg.hipYawDrive.velocity;
    lcm_in.velocity[2] = robot_out.leftLeg.hipPitchDrive.velocity;
    lcm_in.velocity[3] = robot_out.leftLeg.kneeDrive.velocity;
    lcm_in.velocity[4] = robot_out.leftLeg.footDrive.velocity;
    lcm_in.velocity[5] = robot_out.leftLeg.shinJoint.velocity;
    lcm_in.velocity[6] = robot_out.leftLeg.tarsusJoint.velocity;
    lcm_in.velocity[7] = 0.0;//robot_out.leftLeg.footJoint.velocity;
    lcm_in.velocity[8] = robot_out.rightLeg.hipRollDrive.velocity;
    lcm_in.velocity[9] = robot_out.rightLeg.hipYawDrive.velocity;
    lcm_in.velocity[10] = robot_out.rightLeg.hipPitchDrive.velocity;
    lcm_in.velocity[11] = robot_out.rightLeg.kneeDrive.velocity;
    lcm_in.velocity[12] = robot_out.rightLeg.footDrive.velocity;
    lcm_in.velocity[13] = robot_out.rightLeg.shinJoint.velocity;
    lcm_in.velocity[14] = robot_out.rightLeg.tarsusJoint.velocity;
    lcm_in.velocity[15] = 0.0;//robot_out.rightLeg.footJoint.velocity;

    lcm_in.effort[0] = robot_out.leftLeg.hipRollDrive.torque;
    lcm_in.effort[1] = robot_out.leftLeg.hipYawDrive.torque;
    lcm_in.effort[2] = robot_out.leftLeg.hipPitchDrive.torque;
    lcm_in.effort[3] = robot_out.leftLeg.kneeDrive.torque;
    lcm_in.effort[4] = robot_out.leftLeg.footDrive.torque;
    lcm_in.effort[5] = robot_out.rightLeg.hipRollDrive.torque;
    lcm_in.effort[6] = robot_out.rightLeg.hipYawDrive.torque;
    lcm_in.effort[7] = robot_out.rightLeg.hipPitchDrive.torque;
    lcm_in.effort[8] = robot_out.rightLeg.kneeDrive.torque;
    lcm_in.effort[9] = robot_out.rightLeg.footDrive.torque;

    auto current_time = std::chrono::system_clock::now();
    auto duration_in_seconds = std::chrono::duration<double>(current_time.time_since_epoch());
    lcm_in.timestamp  = duration_in_seconds.count()*1e6;
    
    return lcm_in;
}

cassie_dispatch_robot_in_t CassieLcmOutToRobotIn(cassie_dispatch_lcm_out_t lcm_out)
{
    cassie_dispatch_robot_in_t robot_in;
    for (int i = 0; i < cassieEffortNames.size(); i++)
        for (int j = 0; j < lcm_out.num_efforts; j++)
            if (lcm_out.effort_names[j] == cassieEffortNames[i])
                robot_in.torque[i] = lcm_out.efforts[j];
    return robot_in;
}

cassie_dispatch_lcm_out_t CassieRobotInToLcmOut(cassie_dispatch_robot_in_t robot_in)
{

    cassie_dispatch_lcm_out_t lcm_out = cassie_dispatch_lcm_out_t();
    lcm_out.efforts.resize(CASSIE_NUM_EFF);
    lcm_out.effort_names.resize(CASSIE_NUM_EFF);
    
    for (int i = 0; i < cassieEffortNames.size(); i++)
    {
         lcm_out.efforts[i] = robot_in.torque[i];
         lcm_out.effort_names[i] = cassieEffortNames[i];
    }
    return lcm_out;
}
cassie_dispatch_robot_out_t CassieLcmInToRobotOut(cassie_dispatch_lcm_in_t lcm_in)
{
    cassie_dispatch_robot_out_t robot_out;
    double position[CASSIE_NUM_POS];
    double velocity[CASSIE_NUM_VEL];
    double effort[CASSIE_NUM_EFF];
    for (int i = 0; i < cassiePositionNames.size(); i++)
    {
        position[i] = 0.0;
        for (int j = 0; j < lcm_in.num_positions; j++)
            if (lcm_in.position_names[j] == cassiePositionNames[i])
                position[i] = lcm_in.position[j];
    }
    for (int i = 0; i < cassieVelocityNames.size(); i++)
    {
        velocity[i] = 0.0;
        for (int j = 0; j < lcm_in.num_velocities; j++)
            if (lcm_in.velocity_names[j] == cassieVelocityNames[i])
                velocity[i] = lcm_in.velocity[j];
    }
    for (int i = 0; i < cassieEffortNames.size(); i++)
    {
        effort[i] = 0.0;
        for (int j = 0; j < lcm_in.num_efforts; j++)
            if (lcm_in.effort_names[j] == cassieEffortNames[i])
                effort[i] = lcm_in.effort[j];
    }
    
    robot_out.leftLeg.hipRollDrive.position = position[0];
    robot_out.leftLeg.hipYawDrive.position = position[1];
    robot_out.leftLeg.hipPitchDrive.position = position[2];
    robot_out.leftLeg.kneeDrive.position = position[3];
    robot_out.leftLeg.footDrive.position = position[4];
    robot_out.leftLeg.shinJoint.position = position[5];
    robot_out.leftLeg.tarsusJoint.position = position[6];
    robot_out.leftLeg.footJoint.position = position[7];
    robot_out.rightLeg.hipRollDrive.position = position[8];
    robot_out.rightLeg.hipYawDrive.position = position[9];
    robot_out.rightLeg.hipPitchDrive.position = position[10];
    robot_out.rightLeg.kneeDrive.position = position[11];
    robot_out.rightLeg.footDrive.position = position[12];
    robot_out.rightLeg.shinJoint.position = position[13];
    robot_out.rightLeg.tarsusJoint.position = position[14];
    robot_out.rightLeg.footJoint.position = position[15];
    

    robot_out.leftLeg.hipRollDrive.velocity = velocity[0];
    robot_out.leftLeg.hipYawDrive.velocity = velocity[1];
    robot_out.leftLeg.hipPitchDrive.velocity = velocity[2];
    robot_out.leftLeg.kneeDrive.velocity = velocity[3];
    robot_out.leftLeg.footDrive.velocity = velocity[4];
    robot_out.leftLeg.shinJoint.velocity = velocity[5];
    robot_out.leftLeg.tarsusJoint.velocity = velocity[6];
    robot_out.leftLeg.footJoint.velocity = velocity[7];
    robot_out.rightLeg.hipRollDrive.velocity = velocity[8];
    robot_out.rightLeg.hipYawDrive.velocity = velocity[9];
    robot_out.rightLeg.hipPitchDrive.velocity = velocity[10];
    robot_out.rightLeg.kneeDrive.velocity = velocity[11];
    robot_out.rightLeg.footDrive.velocity = velocity[12];
    robot_out.rightLeg.shinJoint.velocity = velocity[13];
    robot_out.rightLeg.tarsusJoint.velocity = velocity[14];
    robot_out.rightLeg.footJoint.velocity = velocity[15];
    
    
    robot_out.leftLeg.hipRollDrive.torque = effort[0];
    robot_out.leftLeg.hipYawDrive.torque = effort[1];
    robot_out.leftLeg.hipPitchDrive.torque = effort[2];
    robot_out.leftLeg.kneeDrive.torque = effort[3];
    robot_out.leftLeg.footDrive.torque = effort[4];
    robot_out.rightLeg.hipRollDrive.torque = effort[5];
    robot_out.rightLeg.hipYawDrive.torque = effort[6];
    robot_out.rightLeg.hipPitchDrive.torque = effort[7];
    robot_out.rightLeg.kneeDrive.torque = effort[8];
    robot_out.rightLeg.footDrive.torque = effort[9];
    
    return robot_out;
        
}





