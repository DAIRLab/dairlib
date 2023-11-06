#include "systems/ros/franka_ros_lcm_conversions.h"
//#include "franka_msgs/FrankaState.h"
#include <sstream>

#include "multibody/multibody_utils.h"
namespace dairlib {
namespace systems {

using Eigen::VectorXd;

// LcmToRosTimestampedVector implementation
LcmToRosTimestampedVector::LcmToRosTimestampedVector(int vector_size)
    : vector_size_(vector_size) {
  this->DeclareVectorInputPort("u, t", TimestampedVector<double>(vector_size_));
  this->DeclareAbstractOutputPort("ROS Float64MultiArray",
                                  &LcmToRosTimestampedVector::ConvertToROS);
};

void LcmToRosTimestampedVector::ConvertToROS(
    const drake::systems::Context<double>& context,
    std_msgs::Float64MultiArray* output) const {
  const TimestampedVector<double>* input =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  // note: this function currently appends the timestamp to the end of the
  // output
  output->data.clear();
  //  std::cout << "time: " << input->get_timestamp() << std::endl;
  for (int i = 0; i < vector_size_; i++) {
    //    std::cout << "value: " << i << " " << input->GetAtIndex(i) <<
    //    std::endl;
    if (std::isnan(input->GetAtIndex(i))) {
      output->data.push_back(0);
    } else {
      output->data.push_back(input->GetAtIndex(i));
    }
  }
  output->data.push_back(input->get_timestamp());
}

/***************************************************************************************/
// ROSToRobotOuput implementation

RosToLcmRobotState::RosToLcmRobotState(int num_positions, int num_velocities,
                                       int num_efforts)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      num_efforts_(num_efforts) {
  this->DeclareAbstractInputPort("Franka JointState topic",
                                 drake::Value<sensor_msgs::JointState>());
  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RosToLcmRobotState::ConvertToLCM);
}

void RosToLcmRobotState::ConvertToLCM(
    const drake::systems::Context<double>& context,
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
  if (msg.position.empty()) {
    for (int i = 0; i < num_positions_; i++) {
      output->position[i] = nan("");
    }
    for (int i = 0; i < num_velocities_; i++) {
      output->velocity[i] = nan("");
    }
    for (int i = 0; i < num_efforts_; i++) {
      output->effort[i] = nan("");
    }
    for (int i = 0; i < 3; i++) {
      output->imu_accel[i] = nan("");
    }
  }
  // input should be order as "positions, velocities, effort, timestamp"
  else {
    for (int i = 0; i < num_franka_joints_; i++) {
      output->position[i] = msg.position[i];
    }
    // hard coded to add 7 zeros to the end of position
    for (int i = num_franka_joints_; i < num_positions_; i++) {
      output->position[i] = default_ball_position_[i - num_franka_joints_];
    }

    for (int i = 0; i < num_velocities_; i++) {
      output->velocity[i] = msg.velocity[i];
    }
    // hard coded to add 6 zeros to the end of velocity
    for (int i = num_franka_joints_; i < num_velocities_; i++) {
      output->velocity[i] = 0;
    }

    for (int i = 0; i < num_efforts_; i++) {
      output->effort[i] = msg.effort[i];
    }
    for (int i = 0; i < 3; i++) {
      output->imu_accel[i] = 0;
    }
  }
  output->utime = context.get_time() * 1e6;
}

/***************************************************************************************/
// RosToLcmFrankaState implementation

RosToLcmFrankaState::RosToLcmFrankaState() {
  this->DeclareAbstractInputPort("ROS Float64MultiArray",
                                 drake::Value<franka_msgs::FrankaState>());
  this->DeclareAbstractOutputPort("lcmt_franka_state",
                                  &RosToLcmFrankaState::ConvertToLCM);
}

void RosToLcmFrankaState::ConvertToLCM(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_franka_state* franka_state) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<franka_msgs::FrankaState>();

  if (msg.O_T_EE.empty()) {
    franka_state->valid = false;
  } else {
    for (size_t i = 0; i < msg.O_T_EE.size(); i++) {
      franka_state->O_T_EE[i] = msg.O_T_EE[i];
    }
    for (size_t i = 0; i < msg.O_T_EE_d.size(); i++) {
      franka_state->O_T_EE_d[i] = msg.O_T_EE_d[i];
    }
    for (size_t i = 0; i < msg.F_T_EE.size(); i++) {
      franka_state->F_T_EE[i] = msg.F_T_EE[i];
    }
    for (size_t i = 0; i < msg.F_T_NE.size(); i++) {
      franka_state->F_T_NE[i] = msg.F_T_NE[i];
    }
    for (size_t i = 0; i < msg.NE_T_EE.size(); i++) {
      franka_state->NE_T_EE[i] = msg.NE_T_EE[i];
    }
    for (size_t i = 0; i < msg.EE_T_K.size(); i++) {
      franka_state->EE_T_K[i] = msg.EE_T_K[i];
    }
    franka_state->m_ee = msg.m_ee;
    for (size_t i = 0; i < msg.I_ee.size(); i++) {
      franka_state->I_ee[i] = msg.I_ee[i];
    }
    for (size_t i = 0; i < msg.F_x_Cee.size(); i++) {
      franka_state->F_x_Cee[i] = msg.F_x_Cee[i];
    }
    franka_state->m_load = msg.m_load;
    for (size_t i = 0; i < msg.I_load.size(); i++) {
      franka_state->I_load[i] = msg.I_load[i];
    }
    for (size_t i = 0; i < msg.F_x_Cload.size(); i++) {
      franka_state->F_x_Cload[i] = msg.F_x_Cload[i];
    }
    franka_state->m_total = msg.m_total;
    for (size_t i = 0; i < msg.I_total.size(); i++) {
      franka_state->I_total[i] = msg.I_total[i];
    }
    for (size_t i = 0; i < msg.F_x_Ctotal.size(); i++) {
      franka_state->F_x_Ctotal[i] = msg.F_x_Ctotal[i];
    }
    for (size_t i = 0; i < msg.elbow.size(); i++) {
      franka_state->elbow[i] = msg.elbow[i];
    }
    for (size_t i = 0; i < msg.elbow_d.size(); i++) {
      franka_state->elbow_d[i] = msg.elbow_d[i];
    }
    for (size_t i = 0; i < msg.elbow_c.size(); i++) {
      franka_state->elbow_c[i] = msg.elbow_c[i];
    }
    for (size_t i = 0; i < msg.delbow_c.size(); i++) {
      franka_state->delbow_c[i] = msg.delbow_c[i];
    }
    for (size_t i = 0; i < msg.ddelbow_c.size(); i++) {
      franka_state->ddelbow_c[i] = msg.ddelbow_c[i];
    }
    for (size_t i = 0; i < msg.tau_J.size(); i++) {
      franka_state->tau_J[i] = msg.tau_J[i];
    }
    for (size_t i = 0; i < msg.tau_J_d.size(); i++) {
      franka_state->tau_J_d[i] = msg.tau_J_d[i];
    }
    for (size_t i = 0; i < msg.dtau_J.size(); i++) {
      franka_state->dtau_J[i] = msg.dtau_J[i];
    }
    for (size_t i = 0; i < msg.q.size(); i++) {
      franka_state->q[i] = msg.q[i];
    }
    for (size_t i = 0; i < msg.q_d.size(); i++) {
      franka_state->q_d[i] = msg.q_d[i];
    }
    for (size_t i = 0; i < msg.dq.size(); i++) {
      franka_state->dq[i] = msg.dq[i];
    }
    for (size_t i = 0; i < msg.dq_d.size(); i++) {
      franka_state->dq_d[i] = msg.dq_d[i];
    }
    for (size_t i = 0; i < msg.ddq_d.size(); i++) {
      franka_state->ddq_d[i] = msg.ddq_d[i];
    }
    for (size_t i = 0; i < msg.joint_contact.size(); i++) {
      franka_state->joint_contact[i] = msg.joint_contact[i];
    }
    for (size_t i = 0; i < msg.cartesian_contact.size(); i++) {
      franka_state->cartesian_contact[i] = msg.cartesian_contact[i];
    }
    for (size_t i = 0; i < msg.joint_collision.size(); i++) {
      franka_state->joint_collision[i] = msg.joint_collision[i];
    }
    for (size_t i = 0; i < msg.cartesian_collision.size(); i++) {
      franka_state->cartesian_collision[i] = msg.cartesian_collision[i];
    }
    for (size_t i = 0; i < msg.tau_ext_hat_filtered.size(); i++) {
      franka_state->tau_ext_hat_filtered[i] = msg.tau_ext_hat_filtered[i];
    }
    for (size_t i = 0; i < msg.O_F_ext_hat_K.size(); i++) {
      franka_state->O_F_ext_hat_K[i] = msg.O_F_ext_hat_K[i];
    }
    for (size_t i = 0; i < msg.K_F_ext_hat_K.size(); i++) {
      franka_state->K_F_ext_hat_K[i] = msg.K_F_ext_hat_K[i];
    }
    for (size_t i = 0; i < msg.O_dP_EE_d.size(); i++) {
      franka_state->O_dP_EE_d[i] = msg.O_dP_EE_d[i];
    }
    for (size_t i = 0; i < msg.O_T_EE_c.size(); i++) {
      franka_state->O_T_EE_c[i] = msg.O_T_EE_c[i];
    }
    for (size_t i = 0; i < msg.O_dP_EE_c.size(); i++) {
      franka_state->O_dP_EE_c[i] = msg.O_dP_EE_c[i];
    }
    for (size_t i = 0; i < msg.O_ddP_EE_c.size(); i++) {
      franka_state->O_ddP_EE_c[i] = msg.O_ddP_EE_c[i];
    }
    for (size_t i = 0; i < msg.theta.size(); i++) {
      franka_state->theta[i] = msg.theta[i];
    }
    for (size_t i = 0; i < msg.dtheta.size(); i++) {
      franka_state->dtheta[i] = msg.dtheta[i];
    }

    std::stringstream ss_current_errors;
    ss_current_errors << msg.current_errors;
    franka_state->current_errors = ss_current_errors.str();

    std::stringstream ss_last_motion_errors;
    ss_last_motion_errors << msg.last_motion_errors;
    franka_state->last_motion_errors = ss_last_motion_errors.str();

    franka_state->control_command_success_rate =
        msg.control_command_success_rate;
    franka_state->robot_mode = msg.robot_mode;
    // time from franka robot
    franka_state->franka_time = msg.time;
    franka_state->valid = true;
  }
  // time for drake loop
  franka_state->utime = context.get_time() * 1e6;
}

/***************************************************************************************/
RosToLcmObjectState::RosToLcmObjectState(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance,
    const std::string& object_name) {
  auto positions_start_idx =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  auto velocities_start_idx =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();

  dairlib::lcmt_object_state object_state = dairlib::lcmt_object_state();
  object_state.num_positions = plant.num_positions(model_instance);
  object_state.num_velocities = plant.num_velocities(model_instance);
  object_state.object_name = object_name;
  object_state.position = std::vector<double>(object_state.num_positions);
  object_state.position_names = multibody::ExtractOrderedNamesFromMap(
      multibody::MakeNameToPositionsMap(plant, model_instance),
      positions_start_idx);
  object_state.velocity = std::vector<double>(object_state.num_velocities);
  object_state.velocity_names = multibody::ExtractOrderedNamesFromMap(
      multibody::MakeNameToVelocitiesMap(plant, model_instance),
      velocities_start_idx);
  object_state.position[0] = 1;
  this->DeclareAbstractOutputPort(object_name + "_state", object_state,
                                  &RosToLcmObjectState::ConvertToLCM);
  this->DeclareAbstractInputPort("ROS Float64MultiArray",
                                 drake::Value<std_msgs::Float64MultiArray>());
}

void RosToLcmObjectState::ConvertToLCM(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_object_state* object_state) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  const auto& msg = input->get_value<std_msgs::Float64MultiArray>();

  if (msg.data.empty()) {
//    for (size_t i = 0; i < object_state->num_positions; i++) {
//      object_state->position[i] = nan("");
//    }
  } else {
    for (size_t i = 0; i < object_state->num_positions; i++) {
      object_state->position[i] = msg.data[i];
    }
  }
  object_state->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
