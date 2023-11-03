#include "c3_trajectory_generator.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/franka/systems/franka_kinematics_vector.h"
#include "multibody/multibody_utils.h"

#include "solvers/c3_output.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::VectorXd;
using systems::TimestampedVector;

namespace systems {

C3TrajectoryGenerator::C3TrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, C3Options c3_options)
    : plant_(plant),
      context_(context),
      c3_options_(std::move(c3_options)),
      N_(c3_options_.N) {
  this->set_name("c3_trajectory_generator");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_lambda_ =
      2 * c3_options_.num_contacts +
      2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  n_u_ = plant_.num_actuators();

  c3_solution_port_ =
      this->DeclareAbstractInputPort("c3_solution", drake::Value<C3Output::C3Solution>())
          .get_index();

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "c3_actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &C3TrajectoryGenerator::OutputActorTrajectory)
          .get_index();
  object_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "c3_object_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &C3TrajectoryGenerator::OutputObjectTrajectory)
          .get_index();
}

void C3TrajectoryGenerator::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3);
  knots.bottomRows(3) = c3_solution->x_sol_.bottomRows(n_v_).topRows(3);
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_traj";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_;
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_traj"},
                         "end_effector_traj", "end_effector_traj", false);

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory end_effector_orientation_traj;
    // first 3 rows are rpy, last 3 rows are angular velocity
    MatrixXd orientation_samples = MatrixXd::Zero(6, N_);
    orientation_samples.topRows(3) = c3_solution->x_sol_.topRows(6).bottomRows(3);
    orientation_samples.bottomRows(3) =
        c3_solution->x_sol_.bottomRows(n_v_).topRows(6).bottomRows(3);
    end_effector_orientation_traj.traj_name = "end_effector_orientation_target";
    end_effector_orientation_traj.datatypes =
        std::vector<std::string>(orientation_samples.rows(), "double");
    end_effector_orientation_traj.datapoints = orientation_samples;
    end_effector_orientation_traj.time_vector = c3_solution->time_vector_;
    lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
                           std::move(end_effector_orientation_traj));
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

void C3TrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.middleRows(n_q_ - 3, 3);
  knots.bottomRows(3) = c3_solution->x_sol_.middleRows(n_q_ + n_v_ - 3, 3);
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_traj";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution->time_vector_;
  LcmTrajectory lcm_traj({object_traj}, {"object_traj"}, "object_traj",
                         "object_traj", false);

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory object_orientation_traj;
    // first 3 rows are rpy, last 3 rows are angular velocity
    MatrixXd orientation_samples = MatrixXd::Zero(7, N_);
    orientation_samples.topRows(4) = c3_solution->x_sol_.middleRows(3 + 3, 4);
    orientation_samples.bottomRows(3) = c3_solution->x_sol_.middleRows(n_q_ + 3 + 3, 3);
    object_orientation_traj.traj_name = "object_orientation_target";
    object_orientation_traj.datatypes =
        std::vector<std::string>(orientation_samples.rows(), "double");
    object_orientation_traj.datapoints = orientation_samples;
    object_orientation_traj.time_vector = c3_solution->time_vector_;
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           std::move(object_orientation_traj));
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib
