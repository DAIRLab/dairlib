/** This system is used in the visualizer diagram. It subscribes to the
 * lcm message containing information about whether we are in c3 mode or not.
 * It also subscribes to the current lcs state and outputs a trajectory for
 * visualization. This trajectory is a single 3d point at the current location
 * of the end effector if we are in C3 mode. It is a single point at the origin
 * if we are not in C3 mode. The trajectory is output as a
 * lcmt_timestamped_saved_traj object. */

#include "examples/sampling_c3/c3_mode_visualizer.h"

#include <iostream>

#include "lcm/lcm_trajectory.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

using drake::systems::Context;
using systems::TimestampedVector;

namespace systems {

C3ModeVisualizer::C3ModeVisualizer() {
  this->set_name("C3ModeVisualizer");

  is_c3_mode_input_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: is_c3_mode_input",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  // 19 is the hardcoded size of the current lcs state vector. Alternatively,
  // pass in the plant and read the size from there.
  curr_lcs_state_ = this->DeclareVectorInputPort("curr_lcs_state",
                                                 TimestampedVector<double>(19))
                        .get_index();

  // Output c3_mode indicator for visualization.
  c3_mode_visualization_traj_port_ =
      this->DeclareAbstractOutputPort(
              "mode_visualization_traj", dairlib::lcmt_timestamped_saved_traj(),
              &C3ModeVisualizer::OutputC3ModeVisualization)
          .get_index();
}

void C3ModeVisualizer::OutputC3ModeVisualization(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* c3_mode_visualization_traj) const {
  // Initialize knots to be a 3x2 matrix of zeros.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, 2);

  // Only update trajectory if the C3 mode input port has received data.
  // This check handles the case when the visualizer starts before the
  // controller, resulting in an uninitialized c3_mode input vector.
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
              context, is_c3_mode_input_port_)
          ->utime > 1e-3) {
    // Evaluate input port to get the sample locations as an
    // lcmt_timestamped_saved_traj.
    const auto& is_c3_mode_lcmt =
        this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
            context, is_c3_mode_input_port_);

    const TimestampedVector<double>* curr_lcs_state =
        (TimestampedVector<double>*)this->EvalVectorInput(context,
                                                          curr_lcs_state_);

    auto is_c3_mode_lcm_obj = LcmTrajectory(is_c3_mode_lcmt->saved_traj);
    auto is_c3_mode_traj = is_c3_mode_lcm_obj.GetTrajectory("is_c3_mode");
    auto is_c3_mode = is_c3_mode_traj.datapoints;

    // This should be the size of the state vector.
    int n_x = curr_lcs_state->get_data().size();

    if (is_c3_mode(0)) {
      // If we are in C3 mode, we want to visualize the current state.
      knots.col(0) = curr_lcs_state->get_data().head(3);
      knots.col(1) = curr_lcs_state->get_data().head(3);
    }
  }

  Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(2);
  // This timestamp corresponds to the trajectory object.
  timestamp(0) = context.get_time();
  timestamp(1) = context.get_time() + 1e-3;

  LcmTrajectory::Trajectory c3_mode;
  c3_mode.traj_name = "c3_mode_visualization";
  c3_mode.datatypes = std::vector<std::string>(3, "double");
  c3_mode.datapoints = knots;
  c3_mode.time_vector = timestamp.cast<double>();

  LcmTrajectory c3_mode_traj({c3_mode}, {"c3_mode_visualization"},
                             "c3_mode_visualization", "c3_mode_visualization",
                             false);

  // Output as lcmt_timestamped_saved_traj
  c3_mode_visualization_traj->saved_traj = c3_mode_traj.GenerateLcmObject();
  c3_mode_visualization_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
