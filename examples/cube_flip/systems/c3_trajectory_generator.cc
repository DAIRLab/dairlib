#include "examples/cube_flip/systems/c3_trajectory_generator.h"

#include <utility>
#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "systems/franka_kinematics_vector.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3_output.h"
#include "solvers/lcs.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using solvers::LCS;
using systems::TimestampedVector;


C3TrajectoryGenerator::C3TrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant, C3Options c3_options)
    : plant_(plant), c3_options_(std::move(c3_options)), N_(c3_options_.N) {
  this->set_name("c3_trajectory_generator");

  std::cout << "HELLO INIT" << std::endl;

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

  c3_solution_port_ =
      this->DeclareAbstractInputPort("c3_solution",
                                     drake::Value<C3Output::C3Solution>())
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
  DRAKE_DEMAND(c3_solution->x_sol_.rows() == n_q_ + n_v_);

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.topRows(3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.bottomRows(n_v_).topRows(3).cast<double>();
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_position_target";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                         "end_effector_position_target",
                         "end_effector_position_target", false);

  std::cout << "Position: " << knots.col(0).transpose() << std::endl;
  std::cout << "Position: " << knots.col(1).transpose() << std::endl;
  std::cout << "Position: " << knots.col(2).transpose() << std::endl;
  std::cout << "Position: " << knots.col(3).transpose() << std::endl;
  std::cout << "Position: " << knots.col(4).transpose() << std::endl;

  MatrixXd force_samples = c3_solution->u_sol_.cast<double>();
  LcmTrajectory::Trajectory force_traj;
  force_traj.traj_name = "end_effector_force_target";
  force_traj.datatypes =
      std::vector<std::string>(3, "double");
  force_traj.datapoints = force_samples.topRows(3);
  force_traj.time_vector = c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(force_traj.traj_name, force_traj);

  std::cout << "Force: " << force_traj.datapoints.col(0).transpose() << std::endl;
  std::cout << "Force: " << force_traj.datapoints.col(1).transpose() << std::endl;
  std::cout << "Force: " << force_traj.datapoints.col(2).transpose() << std::endl;
  std::cout << "Force: " << force_traj.datapoints.col(3).transpose() << std::endl;
  std::cout << "Force: " << force_traj.datapoints.col(4).transpose() << std::endl;

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory torque_traj;
    torque_traj.traj_name = "end_effector_torque_target";
    torque_traj.datatypes =
        std::vector<std::string>(3, "double");
    MatrixXd torque_matrix(3, N_);
    torque_matrix.topRows(2) = force_samples.bottomRows(2);
    torque_traj.datapoints = torque_matrix;
    torque_traj.time_vector = c3_solution->time_vector_.cast<double>();
    lcm_traj.AddTrajectory(torque_traj.traj_name, torque_traj);

    LcmTrajectory::Trajectory end_effector_orientation_traj;
    // first 3 rows are rpy, last 3 rows are angular velocity
    MatrixXd orientation_samples = MatrixXd::Zero(6, N_);
    orientation_samples.topRows(2) =
        c3_solution->x_sol_.topRows(5).bottomRows(2).cast<double>();
    orientation_samples.bottomRows(2) = c3_solution->x_sol_.bottomRows(n_v_)
                                            .topRows(5)
                                            .bottomRows(2)
                                            .cast<double>();
    end_effector_orientation_traj.traj_name = "end_effector_orientation_target";
    end_effector_orientation_traj.datatypes =
        std::vector<std::string>(orientation_samples.rows(), "double");
    end_effector_orientation_traj.datapoints = orientation_samples;
    end_effector_orientation_traj.time_vector =
        c3_solution->time_vector_.cast<double>();
    lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
                           end_effector_orientation_traj);
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

void C3TrajectoryGenerator::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  const auto& c3_solution =
      this->EvalInputValue<C3Output::C3Solution>(context, c3_solution_port_);
  DRAKE_DEMAND(c3_solution->x_sol_.rows() == n_q_ + n_v_);
  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = c3_solution->x_sol_.middleRows(n_q_ - 3, 3).cast<double>();
  knots.bottomRows(3) =
      c3_solution->x_sol_.middleRows(n_q_ + n_v_ - 3, 3).cast<double>();
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_position_target";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = c3_solution->time_vector_.cast<double>();
  LcmTrajectory lcm_traj({object_traj}, {"object_position_target"},
                         "object_target", "object_target", false);

  LcmTrajectory::Trajectory object_orientation_traj;
  // first 3 rows are rpy, last 3 rows are angular velocity
  MatrixXd orientation_samples = MatrixXd::Zero(4, N_);
  orientation_samples =
      c3_solution->x_sol_.middleRows(n_q_ - 7, 4).cast<double>();
  object_orientation_traj.traj_name = "object_orientation_target";
  object_orientation_traj.datatypes =
      std::vector<std::string>(orientation_samples.rows(), "double");
  object_orientation_traj.datapoints = orientation_samples;
  object_orientation_traj.time_vector =
      c3_solution->time_vector_.cast<double>();
  lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                         object_orientation_traj);

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

}  // namespace dairlib
