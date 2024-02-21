#include "examples/jacktoy/systems/tracking_trajectory_generator.h"

#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/jacktoy/systems/franka_kinematics_vector.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using systems::TimestampedVector;

namespace systems {

TrackingTrajectoryGenerator::TrackingTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, C3Options c3_options)
    : plant_(plant), c3_options_(std::move(c3_options)), N_(c3_options_.N) {
  this->set_name("c3_trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    n_lambda_ =
        2 * c3_options_.num_contacts +
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  } else if (c3_options_.contact_model == "anitescu") {
    n_lambda_ =
        2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  }
  n_u_ = plant_.num_actuators();

  tracking_trajectory_input_port_ = 
  this->DeclareAbstractInputPort(
    "tracking_trajectory_input",
    std::vector<TimestampedVector<double>>(N_ + 1, TimestampedVector<double>(n_x_)))
    .get_index();

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &TrackingTrajectoryGenerator::OutputActorTrajectory)
          .get_index();
  object_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "object_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &TrackingTrajectoryGenerator::OutputObjectTrajectory)
          .get_index();
}

void TrackingTrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  const std::vector<TimestampedVector<double>>& tracking_trajectory =
      this->EvalInputValue<std::vector<TimestampedVector<double>>(
        context, tracking_trajectory_input_port_);
  DRAKE_DEMAND((tracking_trajectory[0]->get_data()).size() == n_x_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  VectorXd timestamps = VectorXd::Zero(N_);
  for (int i = 0; i < N_; i++) {
    TimestampedVector<double> state = tracking_trajectory[i].get_data();
    knots.col(i).head(3) = state.head(3);
    knots.col(i).tail(3) = state.segment(n_q_, 3);
    timestamps(i) = tracking_trajectory[i].get_timestamp();
  }
  
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // TODO: Might need to add a force trajectory, or might not if we edit the
  // downstream OSC to not look for one.
  // MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  // LcmTrajectory::Trajectory force_traj;
  // force_traj.traj_name = "end_effector_force_target";
  // force_traj.datatypes =
  //     std::vector<std::string>(force_samples.rows(), "double");
  // force_traj.datapoints = force_samples;
  // force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  // lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  // if (publish_end_effector_orientation_) {
  //   LcmTrajectory::Trajectory end_effector_orientation_traj;
  //   // first 3 rows are rpy, last 3 rows are angular velocity
  //   MatrixXd orientation_samples = MatrixXd::Zero(6, N_);
  //   orientation_samples.topRows(3) =
  //       c3_solution->x_sol_.topRows(6).bottomRows(3).cast<double>();
  //   orientation_samples.bottomRows(3) = c3_solution->x_sol_.bottomRows(n_v_)
  //                                           .topRows(6)
  //                                           .bottomRows(3)
  //                                           .cast<double>();
  //   end_effector_orientation_traj.traj_name = "end_effector_orientation_target";
  //   end_effector_orientation_traj.datatypes =
  //       std::vector<std::string>(orientation_samples.rows(), "double");
  //   end_effector_orientation_traj.datapoints = orientation_samples;
  //   end_effector_orientation_traj.time_vector =
  //       c3_solution->time_vector_.cast<double>();
  //   lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
  //                          end_effector_orientation_traj);
  // }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

void TrackingTrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  const std::vector<TimestampedVector<double>>& tracking_trajectory =
      this->EvalInputValue<std::vector<TimestampedVector<double>>(
        context, tracking_trajectory_input_port_);
  DRAKE_DEMAND((tracking_trajectory[0]->get_data()).size() == n_x_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  VectorXd timestamps = VectorXd::Zero(N_);
  for (int i = 0; i < N_; i++) {
    TimestampedVector<double> state = tracking_trajectory[i].get_data();
    knots.col(i).head(3) = state.head(n_q_).tail(3);
    knots.col(i).tail(3) = state.tail(3);
    orientation_samples.col(i) = state.segment(n_q_ - 7, 4);
    timestamps(i) = tracking_trajectory[i].get_timestamp();
  }
  
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = timestamps.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // Just 4 rows for quaternion.
  orientation_samples = orientation_samples.cast<double>();
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
