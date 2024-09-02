#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/dynamically_feasible_plan_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

DynamicallyFeasiblePlanSender::DynamicallyFeasiblePlanSender(std::string name) {
  this->set_name("dynamically_feasible_plan_sender" + name);

  std::vector<Eigen::VectorXd> dynamically_feasible_plan;

  dynamically_feasible_plan_input_port_ = this->DeclareAbstractInputPort(
          "dynamically_feasible_plan_input",
          drake::Value<std::vector<Eigen::VectorXd>>{dynamically_feasible_plan})
      .get_index();

  dynamically_feasible_plan_output_port_ = this->DeclareAbstractOutputPort(
          "dynamically_feasible_plan_output",
          dairlib::lcmt_timestamped_saved_traj(),
          &DynamicallyFeasiblePlanSender::OutputDynamicallyFeasiblePlan)
      .get_index();
}

void DynamicallyFeasiblePlanSender::OutputDynamicallyFeasiblePlan(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  
	// Evaluate input port to get the dynamically feasible plan
  const auto dynamically_feasible_plan =
      *this->EvalInputValue<std::vector<Eigen::VectorXd>>(
                                context, dynamically_feasible_plan_input_port_
                                );

  // Create a matrix containing the dynamically feasible plan including position and orientation
  // TODO: Change the 7 to read the number of states from the dynamically_feasible_plan
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(7, dynamically_feasible_plan.size());
	Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(dynamically_feasible_plan.size());
	for (int i = 0; i < dynamically_feasible_plan.size(); i++) {
		knots.col(i) = dynamically_feasible_plan[i];
		timestamps(i) = i;
	}

  LcmTrajectory::Trajectory object_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(position_samples.rows(), "double");
  object_traj.datapoints = position_samples;
  object_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);
	
  LcmTrajectory::Trajectory object_orientation_traj;
  // orientation as quaternion
  Eigen::MatrixXd orientation_samples = Eigen::MatrixXd::Zero(4, 6);
  orientation_samples = knots.topRows(4);
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector = timestamps.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib