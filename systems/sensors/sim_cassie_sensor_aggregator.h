#pragma once

#include <string>
#include <map>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "multibody/rbt_utils.h"
#include "multibody/mbt_utils.h"
#include "dairlib/lcmt_cassie_sensor.hpp"

namespace dairlib {
namespace systems {

class SimCassieSensorAggregator : public drake::systems::LeafSystem<double> {
 public:
  explicit SimCassieSensorAggregator(const RigidBodyTree<double>& tree);

  const drake::systems::InputPort<double>& get_input_port_input() const {
    return this->get_input_port(input_input_port_); }
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_); }
  const drake::systems::InputPort<double>& get_input_port_acce() const {
    return this->get_input_port(acce_input_port_); }
  const drake::systems::InputPort<double>& get_input_port_gyro() const {
    return this->get_input_port(gyro_input_port_); }

 private:
  void Aggregator(const drake::systems::Context<double>& context,
                   dairlib::lcmt_cassie_sensor* sensor_msg) const;

  int input_input_port_;
  int state_input_port_;
  int acce_input_port_;
  int gyro_input_port_;

  int num_positions_;
  int num_velocities_;
  std::map<std::string, int> positionIndexMap_;
  std::map<std::string, int> velocityIndexMap_;
  
  std::vector<std::string> motor_position_names_{
  	"hip_roll_left",
	"hip_roll_right",
	"hip_yaw_left",
	"hip_yaw_right",
	"hip_pitch_left",
	"hip_pitch_right",
	"knee_left",
	"knee_right",
	"toe_left",
	"toe_right"};
  std::vector<std::string> motor_velocity_names_{
	"hip_roll_left",
	"hip_roll_right",
	"hip_yaw_left",
	"hip_yaw_right",
	"hip_pitch_left",
	"hip_pitch_right",
	"knee_left",
	"knee_right",
	"toe_left",
	"toe_right"};
  std::vector<std::string> joint_position_names_{
	"knee_joint_left",
	"knee_joint_right",
	"ankle_joint_left",
	"ankle_joint_right",
	"toe_left",
	"toe_right"};
  std::vector<std::string> joint_velocity_names_{
	"knee_joint_leftdot",
	"knee_joint_rightdot",
	"ankle_joint_leftdot",
	"ankle_joint_rightdot",
	"toe_leftdot",
	"toe_rightdot"};
  std::vector<std::string> effort_names_{
	"hip_roll_left_motor",
	"hip_roll_right_motor",
	"hip_yaw_left_motor",
	"hip_yaw_right_motor",
	"hip_pitch_left_motor",
	"hip_pitch_right_motor",
	"knee_left_motor",
	"knee_right_motor",
	"toe_left_motor",
	"toe_right_motor"};
  std::vector<std::string> imu_linear_acc_names_{"accel_x","accel_y","accel_z"};
  std::vector<std::string> imu_ang_vel_names_{"w_x","w_y","w_z"};
  
};

}  // namespace systems
}  // namespace dairlib
