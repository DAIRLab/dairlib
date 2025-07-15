#include "examples/sampling_c3/c3_mode_visualizer.h"

#include "lcm/lcm_trajectory.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

using drake::systems::Context;
using systems::TimestampedVector;

namespace systems {

C3ModeVisualizer::C3ModeVisualizer(const drake::multibody::MultibodyPlant<double>& plant)
    : plant_(plant) {
  this->set_name("C3ModeVisualizer");

  is_c3_mode_input_port_ =
    this->DeclareAbstractInputPort(
      "lcmt_timestamped_saved_traj: is_c3_mode_input",
      drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
    .get_index();

  // 19 is the hardcoded size of the current lcs state vector. Alternatively,
  // pass in the plant and read the size from there.
  int lcs_state_size = plant_.num_positions() + plant_.num_velocities() + 6;
  curr_lcs_state_ = this->DeclareVectorInputPort(
    "curr_lcs_state", TimestampedVector<double>(lcs_state_size)).get_index();

  std::cout << "passed curr_lcs_state_ size: "
            << plant_.num_positions() + plant_.num_velocities() + 6
            << std::endl;
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
  // NOTE:  LcmTrajectory needs at least two knot points, so create a dummy
  // second knot also at the desired C3 mode visualization location.
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, 2);

  // Only update trajectory if the C3 mode input port has received data.
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
      context, is_c3_mode_input_port_)->utime > 1e-3) {
    // Evaluate input port to get the sample locations as an
    // lcmt_timestamped_saved_traj.
    const auto& is_c3_mode_lcmt =
      this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
        context, is_c3_mode_input_port_);

    const TimestampedVector<double>* curr_lcs_state =
      (TimestampedVector<double>*)this->EvalVectorInput(
        context, curr_lcs_state_);

    auto is_c3_mode_lcm_obj = LcmTrajectory(is_c3_mode_lcmt->saved_traj);
    auto is_c3_mode_traj = is_c3_mode_lcm_obj.GetTrajectory("is_c3_mode");
    auto is_c3_mode = is_c3_mode_traj.datapoints;

    // If we are in C3 mode, visualize the current EE location.
    if (is_c3_mode(0)) {
      knots.col(0) = curr_lcs_state->get_data().head(3);
      knots.col(1) = curr_lcs_state->get_data().head(3);
    }
  }
  Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(2);
  timestamp(0) = context.get_time();
  timestamp(1) = context.get_time() + 1e-3;

  LcmTrajectory::Trajectory c3_mode;
  c3_mode.traj_name = "c3_mode_visualization";
  c3_mode.datatypes = std::vector<std::string>(3, "double");
  c3_mode.datapoints = knots;
  c3_mode.time_vector = timestamp.cast<double>();

  LcmTrajectory c3_mode_traj(
    {c3_mode}, {"c3_mode_visualization"}, "c3_mode_visualization",
    "c3_mode_visualization", false);

  // Output as lcmt_timestamped_saved_traj
  c3_mode_visualization_traj->saved_traj = c3_mode_traj.GenerateLcmObject();
  c3_mode_visualization_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
