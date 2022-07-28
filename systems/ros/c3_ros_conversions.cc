#include "c3_ros_conversions.h"
#include "franka_msgs/FrankaState.h"
#include <sstream>
namespace dairlib {
namespace systems {

using Eigen::VectorXd;

/***************************************************************************************/
// TimestampedVectorToROS implementation

TimestampedVectorToROS::TimestampedVectorToROS(int num_elements)
  : num_elements_(num_elements) {
  
  this->DeclareVectorInputPort("u, t",
    TimestampedVector<double>(num_elements_));
  this->DeclareAbstractOutputPort("ROS Float64MultiArray",
                                  &TimestampedVectorToROS::ConvertToROS);
};

void TimestampedVectorToROS::ConvertToROS(const drake::systems::Context<double>& context,
                std_msgs::Float64MultiArray* output) const {
  
  const TimestampedVector<double>* input =
    (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
  
  // note: this function currently appends the timestamp to the end of the output
  output->data.clear();
  for (int i = 0; i < num_elements_; i++){
    if (std::isnan(input->GetAtIndex(i))){
      output->data.push_back(0);
    }
    else{
      output->data.push_back(input->GetAtIndex(i));
    }
  }
  output->data.push_back(input->get_timestamp()); // TODO: figure out if 1e6 needed
}

/***************************************************************************************/
// ROSToRobotOuput implementation

ROSToRobotOutputLCM::ROSToRobotOutputLCM(int num_positions, int num_velocities, int num_efforts)
  : num_positions_(num_positions), num_velocities_(num_velocities), num_efforts_(num_efforts) {

  this->DeclareAbstractInputPort("Franka JointState topic", 
    drake::Value<sensor_msgs::JointState>());
  this->DeclareAbstractOutputPort("lcmt_robot_output",
    &ROSToRobotOutputLCM::ConvertToLCM);
}

void ROSToRobotOutputLCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_robot_output* output) const {

  const drake::AbstractValue* const input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& msg = input->get_value<sensor_msgs::JointState>();

  output->num_positions = num_positions_;
  output->num_velocities = num_velocities_;
  output->num_efforts = num_efforts_;
  output->position_names.resize(num_positions_);
  output->velocity_names.resize(num_velocities_);
  output->effort_names.resize(num_efforts_);
  output->position_names = position_names_;
  output->velocity_names = velocity_names_;
  output->effort_names = effort_names_;
  output->position.resize(num_positions_);
  output->velocity.resize(num_velocities_);
  output->effort.resize(num_efforts_);
  
  // yet to receive message from rostopic
  if (msg.position.empty()){
    for (int i = 0; i < num_positions_; i++){
      output->position[i] = nan("");
    }
    for (int i = 0; i < num_velocities_; i++){
      output->velocity[i] = nan("");
    }
    for (int i = 0; i < num_efforts_; i++){
      output->effort[i] = nan("");
    }
    for (int i = 0; i < 3; i++){
      output->imu_accel[i] = nan("");
    }
  }
  // input should be order as "positions, velocities, effort, timestamp"
  else{
    for (int i = 0; i < num_franka_joints_; i++){
      output->position[i] = msg.position[i];
    }
    // hard coded to add 7 zeros to the end of position
    for (int i = num_franka_joints_; i < num_positions_; i++){
      output->position[i] = default_ball_position_[i-num_franka_joints_];
    }

    for (int i = 0; i < num_velocities_; i++){
      output->velocity[i] = msg.velocity[i];
    }
    // hard coded to add 6 zeros to the end of velocity
    for (int i = num_franka_joints_; i < num_velocities_; i++){
      output->velocity[i] = 0;
    }
    
    for (int i = 0; i < num_efforts_; i++){
      output->effort[i] = msg.effort[i];
    }
    for (int i = 0; i < 3; i++){
      output->imu_accel[i] = 0;
    }
  }
  // TODO: figure out which time to use
  output->utime = context.get_time() * 1e6;
  //output->utime = msg.head.stamp.secs
  //output->utime = msg.data[num_positions_+num_velocities_+num_efforts_];
}

/***************************************************************************************/
// ROSToC3LCM implementation

ROSToC3LCM::ROSToC3LCM(int num_positions, int num_velocities,
    int lambda_size, int misc_size)
  : num_positions_(num_positions), num_velocities_(num_velocities),
    lambda_size_(lambda_size), misc_size_(misc_size) {

  this->DeclareAbstractInputPort("ROS Float64MultiArray", 
    drake::Value<std_msgs::Float64MultiArray>());
  this->DeclareAbstractOutputPort("lcmt_c3",
    &ROSToC3LCM::ConvertToLCM);
  
  data_size_ = num_positions_+num_velocities_+lambda_size_+misc_size_;
}

void ROSToC3LCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_c3* c3_msg) const {

  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<std_msgs::Float64MultiArray>();

  c3_msg->data_size = data_size_;
  c3_msg->data.resize(data_size_);
  if (msg.data.empty()){
    for (int i = 0; i < data_size_; i++){
      c3_msg->data[i] = nan("");
    }
  }
  else{
    for (int i = 0; i < data_size_; i++){
      c3_msg->data[i] = msg.data[i];
    }
  }
  // TODO: figure out which time to use
  c3_msg->utime = context.get_time() * 1e6;
  // c3_msg->utime = msg.data[data_size_];
}

/***************************************************************************************/
// ROSToFrankaStateLCM implementation

ROSToFrankaStateLCM::ROSToFrankaStateLCM() {

  this->DeclareAbstractInputPort("ROS Float64MultiArray", 
    drake::Value<franka_msgs::FrankaState>());
  this->DeclareAbstractOutputPort("lcmt_franka_state",
    &ROSToFrankaStateLCM::ConvertToLCM);
}

void ROSToFrankaStateLCM::ConvertToLCM(const drake::systems::Context<double>& context,
                  dairlib::lcmt_franka_state* franka_state) const {
  
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<franka_msgs::FrankaState>();

  for (uint8_t i = 0; i < msg.O_T_EE.size(); i++){
    franka_state->O_T_EE[i] = msg.O_T_EE[i];
  }
  for (uint8_t i = 0; i < msg.O_T_EE_d.size(); i++){
    franka_state->O_T_EE_d[i] = msg.O_T_EE_d[i];
  }
  for (uint8_t i = 0; i < msg.F_T_EE.size(); i++){
    franka_state->F_T_EE[i] = msg.F_T_EE[i];
  }
  for (uint8_t i = 0; i < msg.F_T_NE.size(); i++){
    franka_state->F_T_NE[i] = msg.F_T_NE[i];
  }
  for (uint8_t i = 0; i < msg.NE_T_EE.size(); i++){
    franka_state->NE_T_EE[i] = msg.NE_T_EE[i];
  }
  for (uint8_t i = 0; i < msg.EE_T_K.size(); i++){
    franka_state->EE_T_K[i] = msg.EE_T_K[i];
  }
  franka_state->m_ee = msg.m_ee;
  for (uint8_t i = 0; i < msg.I_ee.size(); i++){
    franka_state->I_ee[i] = msg.I_ee[i];
  }
  for (uint8_t i = 0; i < msg.F_x_Cee.size(); i++){
    franka_state->F_x_Cee[i] = msg.F_x_Cee[i];
  }
  franka_state->m_load = msg.m_load;
  for (uint8_t i = 0; i < msg.I_load.size(); i++){
    franka_state->I_load[i] = msg.I_load[i];
  }
  for (uint8_t i = 0; i < msg.F_x_Cload.size(); i++){
    franka_state->F_x_Cload[i] = msg.F_x_Cload[i];
  }
  franka_state->m_total = msg.m_total;
  for (uint8_t i = 0; i < msg.I_total.size(); i++){
    franka_state->I_total[i] = msg.I_total[i];
  }
  for (uint8_t i = 0; i < msg.F_x_Ctotal.size(); i++){
    franka_state->F_x_Ctotal[i] = msg.F_x_Ctotal[i];
  }
  for (uint8_t i = 0; i < msg.elbow.size(); i++){
    franka_state->elbow[i] = msg.elbow[i];
  }
  for (uint8_t i = 0; i < msg.elbow_d.size(); i++){
    franka_state->elbow_d[i] = msg.elbow_d[i];
  }
  for (uint8_t i = 0; i < msg.elbow_c.size(); i++){
    franka_state->elbow_c[i] = msg.elbow_c[i];
  }
  for (uint8_t i = 0; i < msg.delbow_c.size(); i++){
    franka_state->delbow_c[i] = msg.delbow_c[i];
  }
  for (uint8_t i = 0; i < msg.ddelbow_c.size(); i++){
    franka_state->ddelbow_c[i] = msg.ddelbow_c[i];
  }
  for (uint8_t i = 0; i < msg.tau_J.size(); i++){
    franka_state->tau_J[i] = msg.tau_J[i];
  }
  for (uint8_t i = 0; i < msg.tau_J_d.size(); i++){
    franka_state->tau_J_d[i] = msg.tau_J_d[i];
  }
  for (uint8_t i = 0; i < msg.dtau_J.size(); i++){
    franka_state->dtau_J[i] = msg.dtau_J[i];
  }
  for (uint8_t i = 0; i < msg.q.size(); i++){
    franka_state->q[i] = msg.q[i];
  }
  for (uint8_t i = 0; i < msg.q_d.size(); i++){
    franka_state->q_d[i] = msg.q_d[i];
  }
  for (uint8_t i = 0; i < msg.dq.size(); i++){
    franka_state->dq[i] = msg.dq[i];
  }
  for (uint8_t i = 0; i < msg.dq_d.size(); i++){
    franka_state->dq_d[i] = msg.dq_d[i];
  }
  for (uint8_t i = 0; i < msg.ddq_d.size(); i++){
    franka_state->ddq_d[i] = msg.ddq_d[i];
  }
  for (uint8_t i = 0; i < msg.joint_contact.size(); i++){
    franka_state->joint_contact[i] = msg.joint_contact[i];
  }
  for (uint8_t i = 0; i < msg.cartesian_contact.size(); i++){
    franka_state->cartesian_contact[i] = msg.cartesian_contact[i];
  }
  for (uint8_t i = 0; i < msg.joint_collision.size(); i++){
    franka_state->joint_collision[i] = msg.joint_collision[i];
  }
  for (uint8_t i = 0; i < msg.cartesian_collision.size(); i++){
    franka_state->cartesian_collision[i] = msg.cartesian_collision[i];
  }
  for (uint8_t i = 0; i < msg.tau_ext_hat_filtered.size(); i++){
    franka_state->tau_ext_hat_filtered[i] = msg.tau_ext_hat_filtered[i];
  }
  for (uint8_t i = 0; i < msg.O_F_ext_hat_K.size(); i++){
    franka_state->O_F_ext_hat_K[i] = msg.O_F_ext_hat_K[i];
  }
  for (uint8_t i = 0; i < msg.K_F_ext_hat_K.size(); i++){
    franka_state->K_F_ext_hat_K[i] = msg.K_F_ext_hat_K[i];
  }
  for (uint8_t i = 0; i < msg.O_dP_EE_d.size(); i++){
    franka_state->O_dP_EE_d[i] = msg.O_dP_EE_d[i];
  }
  for (uint8_t i = 0; i < msg.O_T_EE_c.size(); i++){
    franka_state->O_T_EE_c[i] = msg.O_T_EE_c[i];
  }
  for (uint8_t i = 0; i < msg.O_dP_EE_c.size(); i++){
    franka_state->O_dP_EE_c[i] = msg.O_dP_EE_c[i];
  }
  for (uint8_t i = 0; i < msg.O_ddP_EE_c.size(); i++){
    franka_state->O_ddP_EE_c[i] = msg.O_ddP_EE_c[i];
  }
  for (uint8_t i = 0; i < msg.theta.size(); i++){
    franka_state->theta[i] = msg.theta[i];
  }
  for (uint8_t i = 0; i < msg.dtheta.size(); i++){
    franka_state->dtheta[i] = msg.dtheta[i];
  }

  std::stringstream ss_current_errors;
  ss_current_errors << msg.current_errors;
  franka_state->current_errors = ss_current_errors.str();
  
  std::stringstream ss_last_motion_errors;
  ss_last_motion_errors << msg.last_motion_errors;
  franka_state->last_motion_errors = ss_last_motion_errors.str();

  franka_state->control_command_success_rate = msg.control_command_success_rate;
  franka_state->robot_mode = msg.robot_mode;
  franka_state->time = msg.time;
}

}  // namespace systems
}  // namespace dairlib
