#pragma once
#include <limits>

#include "systems/framework/timestamped_vector.h"
#include "dairlib/lcmt_input_supervisor_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

static constexpr double kMaxControllerDelay = 0.1;
static constexpr double kEStopGain = 5.0;

namespace dairlib {

/// The InputSupervisor is a simple Drake System that acts as an intermediary
/// between commands from controllers and the actual robot. It's envisioned role
/// is to (1) sanity check any inputs, (2) shut down in case of errors or
/// instability, and (3) to mediate between multiple controllers. Of these three
/// potential purposes, (1) and (2) are currently implemented.
///  (1) is a simple threshold of the commands to [-input_limit, input_limit]
///  (2) is a check on the current velocity of the robot.
/// If the velocity passes a specified threshold,
/// in absolute value, then an error flag (discrete state) is set and the output
/// commands are set to zero. Note, in the presence of noise, this __might__ be
/// a brittle check. If this is the case, we will have to low-pass filter this
/// check. For instance, we could only set the flag if the threshold is reached
/// N times in a row.
///
/// Other future extensions could include more detailed error checking, such as
/// measured motor torque limits, dynamics errors, or joint limit errors. We
/// should also integrate information from the Agility error list.
class InputSupervisor : public drake::systems::LeafSystem<double> {
 public:
  // Constructor.
  // @param plant MultibodyPlant of the robot
  // @param max_joint_velocity
  // @param update_period
  // @param min_consecutive_failures (default = 1) before failure is triggered
  // @param input_limit (default = inf) to threshold all commands
  // If necessary, the max_joint_velocity and input_limit could be
  // replaced with a joint-specific vectors.
  explicit InputSupervisor(
      const drake::multibody::MultibodyPlant<double>& plant,
      double max_joint_velocity, double update_period,
      int min_consecutive_failures = 1,
      double input_limit = std::numeric_limits<double>::max());

  const drake::systems::InputPort<double>& get_input_port_command() const {
    return this->get_input_port(command_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_controller_switch() const {
    return this->get_input_port(controller_switch_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_cassie() const {
    return this->get_input_port(cassie_input_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_command() const {
    return this->get_output_port(command_output_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_status() const {
    return this->get_output_port(status_output_port_);
  }

  void SetMotorTorques(const drake::systems::Context<double>& context,
                       systems::TimestampedVector<double>* output) const;
  void UpdateErrorFlag(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  // Assign the lcmt_input_supervisor_status output
  // Sets the status bit to the current status
  // Starting from the rightmost bit, the bits represent:
  // 0th: velocity limits exceeded once
  // 1st: actuator limits
  // 2nd: velocity triggered shutdown
  void SetStatus(const drake::systems::Context<double>& context,
                 dairlib::lcmt_input_supervisor_status* output) const;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const int num_actuators_;
  const int num_positions_;
  const int num_velocities_;
  const int min_consecutive_failures_;
  double max_joint_velocity_;
  double input_limit_;
  mutable double blend_duration_ = 0.0;
  int status_vars_index_;
  int soft_estop_flag_index_;
  int n_fails_index_;
  int status_index_;
  int switch_time_index_;
  int prev_efforts_index_;
  int prev_efforts_time_index_;
  int state_input_port_;
  int command_input_port_;
  int controller_switch_input_port_;
  int cassie_input_port_;
  int command_output_port_;
  int status_output_port_;

  Eigen::MatrixXd K_;
};

}  // namespace dairlib
