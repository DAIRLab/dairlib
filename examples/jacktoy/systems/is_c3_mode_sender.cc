#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/is_c3_mode_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

IsC3ModeSender::IsC3ModeSender() {
  this->set_name("is_c3_mode_sender");

  is_c3_mode_input_port_ = this->DeclareVectorInputPort(
    "is_c3_mode_input", drake::systems::BasicVector<double>(1)).get_index();

  is_c3_mode_output_port_ = this->DeclareAbstractOutputPort(
    "is_c3_mode_output",
    dairlib::lcmt_timestamped_saved_traj(),
    &IsC3ModeSender::OutputIsC3Mode)
      .get_index();
}

void IsC3ModeSender::OutputIsC3Mode(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_is_c3_mode) const {
  
	// Evaluate input port to get the sample locations
  const auto is_c3_mode =
      this->EvalInputValue<drake::systems::BasicVector<double>>(
                                context, is_c3_mode_input_port_
                                )->value();
  
  // NOTE: This is a placeholder for the boolean value so we can output 
  // it as an existing lcm message type. It is unconventional to be 
  // using this message type for this purpose.
  Eigen::MatrixXd c3_mode_data = Eigen::MatrixXd::Zero(1, 1);
  Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(1);

  // Reading the boolean value into the matrix.
  c3_mode_data(0, 0) = is_c3_mode(0); 
  // This timestamp corresponds to the trajectory object.
  timestamp(0) = context.get_time();

  LcmTrajectory::Trajectory c3_mode;
  c3_mode.traj_name = "is_c3_mode";
  c3_mode.datatypes = std::vector<std::string>(1, "bool");
  c3_mode.datapoints = c3_mode_data;
  c3_mode.time_vector = timestamp.cast<double>();

  LcmTrajectory c3_mode_traj({c3_mode}, {"is_c3_mode"},
                          "is_c3_mode",
                          "is_c3_mode", false);

  // Output the mode as an lcm message
  output_is_c3_mode->saved_traj = c3_mode_traj.GenerateLcmObject();
  output_is_c3_mode->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib