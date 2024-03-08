#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/c3_mode_visualizer.h"
#include <iostream>

namespace dairlib {

using systems::TimestampedVector;
using drake::systems::Context;

namespace systems {

C3ModeVisualizer::C3ModeVisualizer() {
  this->set_name("C3ModeVisualizer");

  is_c3_mode_input_port_ = 
      this->DeclareAbstractInputPort(
              "lcmt_timestamped_saved_traj: is_c3_mode_input", 
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
            .get_index();
  
  // TODO: Figure out how to read state vector size here. Might have to be an 
  // input like in C3StateSender.
  curr_lcs_state_ = this->DeclareVectorInputPort("curr_lcs_state",
                                              TimestampedVector<double>(19))
                    .get_index();

  // Execution trajectory output ports.
  c3_mode_visualization_traj_port_ = this->DeclareAbstractOutputPort(
    "mode_visualization_traj", dairlib::lcmt_timestamped_saved_traj(),
    &C3ModeVisualizer::OutputC3ModeVisualization).get_index();
}

void C3ModeVisualizer::OutputC3ModeVisualization(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* c3_mode_visualization_traj) const {
  // Check if it's too early to output.
  if (this->EvalInputValue<dairlib::lcmt_timestamped_saved_traj>(
          context, is_c3_mode_input_port_) ->utime < 1e-3) {
    // If it's too early, output a single point at 0,0,3.
    Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, 2);
    knots.col(0) << 0, 0, 3;
    knots.col(1) << 0, 0, 3;
    Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(2);
    timestamp(0) = context.get_time();
    timestamp(1) = context.get_time() + 1e-3;

    LcmTrajectory::Trajectory c3_mode;
    c3_mode.traj_name = "c3_mode_visualization";
    c3_mode.datatypes = std::vector<std::string>(3, "double");
    c3_mode.datapoints = knots;
    c3_mode.time_vector = timestamp.cast<double>();

    LcmTrajectory c3_mode_traj({c3_mode}, {"c3_mode_visualization"},
                            "c3_mode_visualization",
                            "c3_mode_visualization", false);

    c3_mode_visualization_traj->saved_traj = c3_mode_traj.GenerateLcmObject();
    c3_mode_visualization_traj->utime = context.get_time() * 1e6;
  }
  else{
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

    Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, 2);
    Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(2);

    // This timestamp corresponds to the trajectory object.
    timestamp(0) = context.get_time();
    timestamp(1) = context.get_time() + 1e-3;

    if (is_c3_mode(0)) {
      // If we are in C3 mode, we want to visualize the current state.
      knots.col(0) = curr_lcs_state->get_data().head(3);
      knots.col(1) = curr_lcs_state->get_data().head(3);
    }

    LcmTrajectory::Trajectory c3_mode;
    c3_mode.traj_name = "c3_mode_visualization";
    c3_mode.datatypes = std::vector<std::string>(3, "double");
    c3_mode.datapoints = knots;
    c3_mode.time_vector = timestamp.cast<double>();

    LcmTrajectory c3_mode_traj({c3_mode}, {"c3_mode_visualization"},
                            "c3_mode_visualization",
                            "c3_mode_visualization", false);

    // Output as lcmt_timestamped_saved_traj
    c3_mode_visualization_traj->saved_traj = c3_mode_traj.GenerateLcmObject();
    c3_mode_visualization_traj->utime = context.get_time() * 1e6;
  }
}

}  // namespace systems
}  // namespace dairlib