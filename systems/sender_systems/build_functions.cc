#include <iostream>
#include "systems/sender_systems/build_functions.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include <Eigen/Dense>

namespace dairlib {
  using Eigen::MatrixXd;
namespace systems {

// Builds the dynamically feasible plan sender actor message.
void BuildDynamicallyFeasiblePlanSenderActor(const std::vector<Eigen::VectorXd>& dynamically_feasible_plan, 
                                             dairlib::lcmt_timestamped_saved_traj* output) {
  // Create a matrix containing the dynamically feasible plan including position and orientation
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(3, dynamically_feasible_plan.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(dynamically_feasible_plan.size());
  for (int i = 0; i < dynamically_feasible_plan.size(); i++) {
    knots.col(i) = dynamically_feasible_plan[i].head(3);
    timestamps(i) = i;
  }

  LcmTrajectory::Trajectory ee_traj;
  // position trajectory
  Eigen::MatrixXd position_samples = Eigen::MatrixXd::Zero(3, 6);
  position_samples = knots.bottomRows(3);
  ee_traj.traj_name = "ee_position_target";
  ee_traj.datatypes = std::vector<std::string>(position_samples.rows(), "double");
  ee_traj.datapoints = position_samples;
  ee_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory ee_traj_lcm({ee_traj}, {"ee_position_target"},
                            "ee_position_target",
                            "ee_position_target", false);

  // Output the trajectory as an lcm message
  output->saved_traj = ee_traj_lcm.GenerateLcmObject();
}

// Builds the dynamically feasible plan object sender message.
void BuildDynamicallyFeasiblePlanSenderObject(const std::vector<Eigen::VectorXd>& dynamically_feasible_plan, 
                                              dairlib::lcmt_timestamped_saved_traj* output_traj) {
  // Create a matrix containing the dynamically feasible plan including position and orientation
  // TODO: Change the 7 to read the number of states from the dynamically_feasible_plan
  Eigen::MatrixXd knots = Eigen::MatrixXd::Zero(7, dynamically_feasible_plan.size());
	Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(dynamically_feasible_plan.size());
	for (int i = 0; i < dynamically_feasible_plan.size(); i++) {
		knots.col(i) = dynamically_feasible_plan[i].segment(3, 7);
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
}


// Builds tracking trajectory actor sender message.
void BuildTrackingTrajectoryActorSender(const LcmTrajectory& tracking_trajectory, 
                                         dairlib::lcmt_timestamped_saved_traj* output) {
  // Create a matrix containing the tracking trajectory including position
  LcmTrajectory::Trajectory end_effector_traj = 
    tracking_trajectory.GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);
  // NOTE: End effector orientation functionality is not implemented.
  
  // TODO: Might need to add a force trajectory that is non-zero for the 
  // downstream osc to track.
  // TODO : ARE WE ACTUALLY TRACKING THE FORCE? 
  // TODO: The 5 here is the hardcoded planning horizon.
  MatrixXd force_samples = MatrixXd::Zero(3, 5);
  LcmTrajectory::Trajectory force_traj = 
    tracking_trajectory.GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
  }

// Builds tracking trajectory object sender message.
void BuildTrackingTrajectoryObjectSender(const LcmTrajectory& tracking_trajectory, 
                                          dairlib::lcmt_timestamped_saved_traj* output) {

  LcmTrajectory::Trajectory object_traj = 
    tracking_trajectory.GetTrajectory("object_position_target");
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                          "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj = 
    tracking_trajectory.GetTrajectory("object_orientation_target");
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                          object_orientation_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
}

// Builds c3_tracking trajectory actor sender message.
void BuildC3TrackingTrajectoryActorSender(const C3Output::C3Solution& c3_solution, 
                                           dairlib::lcmt_timestamped_saved_traj* output) {
  // Get N and n_v from the c3_solution
  int N = c3_solution.x_sol_.cols();
  int n_v = c3_solution.x_sol_.rows() - 10;

  MatrixXd knots = MatrixXd::Zero(6, N);
  knots.topRows(3) = c3_solution.x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution.x_sol_.bottomRows(n_v).topRows(3).cast<double>();
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution.time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                          "end_effector_position_target",
                          "end_effector_position_target", false);

  // NOTE: End effector orientation functionality is not implemented.

  // Generate force trajectory
  MatrixXd force_samples = c3_solution.u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(force_samples.rows(), "double");
  force_traj.datapoints = force_samples;
  force_traj.time_vector = c3_solution.time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
}

// Builds c3 tracking trajectory object sender message.
void BuildC3TrackingTrajectoryObjectSender(const C3Output::C3Solution& c3_solution, 
                                             dairlib::lcmt_timestamped_saved_traj* output) {
  // Get N and n_v from the c3_solution
  int N = c3_solution.x_sol_.cols();
  int n_v = c3_solution.x_sol_.rows() - 10;
  int n_q = 10;
  MatrixXd knots = MatrixXd::Zero(6, N);
  knots.topRows(3) = c3_solution.x_sol_.middleRows(n_q - 3, 3).cast<double>();
  knots.bottomRows(3) =
      c3_solution.x_sol_.middleRows(n_q + n_v - 3, 3).cast<double>();
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution.time_vector_.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                          "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N);
  orientation_samples =
      c3_solution.x_sol_.middleRows(n_q - 7, 4).cast<double>();
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector =
      c3_solution.time_vector_.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                          object_orientation_traj);

  output->saved_traj = lcm_traj.GenerateLcmObject();
}

} // namespace systems
}  // namespace dairlib