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
    const drake::multibody::MultibodyPlant<double>& plant, C3Options c3_options,
    std::string name)
    : plant_(plant), c3_options_(std::move(c3_options)), N_(c3_options_.N) {
  this->set_name(name);

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
    drake::Value<LcmTrajectory>())
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

  const auto& tracking_trajectory =
      this->EvalInputValue<LcmTrajectory>(
        context, tracking_trajectory_input_port_);

  // DRAKE_DEMAND(tracking_trajectory.rows() == n_q_ + n_v_);

  LcmTrajectory::Trajectory end_effector_traj = 
    tracking_trajectory->GetTrajectory("end_effector_position_target");
  DRAKE_DEMAND(end_effector_traj.datapoints.rows() == 3);
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  // TODO: Might need to add a force trajectory that is non-zero for the 
  // downstream osc to track.
  MatrixXd force_samples = MatrixXd::Zero(3, N_);
  LcmTrajectory::Trajectory force_traj = 
    tracking_trajectory->GetTrajectory("end_effector_force_target");
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory end_effector_orientation_traj = 
      tracking_trajectory->GetTrajectory("end_effector_orientation_target");
    lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
                           end_effector_orientation_traj);
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

void TrackingTrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  const auto& tracking_trajectory =
      this->EvalInputValue<LcmTrajectory>(
        context, tracking_trajectory_input_port_);
  
  LcmTrajectory::Trajectory object_traj = 
    tracking_trajectory->GetTrajectory("object_position_target");
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj = 
    tracking_trajectory->GetTrajectory("object_orientation_target");
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
