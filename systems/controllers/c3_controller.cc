#include "c3_controller.h"

#include <iostream>
#include <utility>

#include <dairlib/lcmt_radio_out.hpp>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/franka/systems/franka_kinematics_vector.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"

#include "drake/solvers/moby_lcp_solver.h"

namespace dairlib {

using drake::multibody::ModelInstanceIndex;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using Eigen::VectorXd;
using solvers::C3MIQP;
using solvers::LCS;
using solvers::LCSFactory;
using systems::TimestampedVector;

namespace systems {

C3Controller::C3Controller(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::systems::Context<drake::AutoDiffXd>* context_ad,
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
        contact_geoms,
    C3Options c3_options)
    : plant_(plant),
      context_(context),
      plant_ad_(plant_ad),
      context_ad_(context_ad),
      contact_pairs_(contact_geoms),
      c3_options_(std::move(c3_options)),
      Q_(std::vector<MatrixXd>(c3_options_.N + 1, c3_options_.Q)),
      R_(std::vector<MatrixXd>(c3_options_.N, c3_options_.R)),
      G_(std::vector<MatrixXd>(c3_options_.N, c3_options_.G)),
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)),
      N_(c3_options_.N) {
  this->set_name("c3_controller");

  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_x_ = n_q_ + n_v_;
  n_lambda_ =
      2 * c3_options_.num_contacts +
      2 * c3_options_.num_friction_directions * c3_options_.num_contacts;
  n_u_ = plant_.num_actuators();
  Q_.back() = 100 * c3_options_.Q;

  lcs_state_input_port_ =
      this->DeclareVectorInputPort(
              "x_lcs", FrankaKinematicsVector<double>(
                           plant_.num_positions(ModelInstanceIndex(2)),
                           plant_.num_positions(ModelInstanceIndex(3)),
                           plant_.num_velocities(ModelInstanceIndex(2)),
                           plant_.num_velocities(ModelInstanceIndex(3))))
          .get_index();
  int x_des_size = plant_.num_positions(ModelInstanceIndex(2)) +
                   plant_.num_positions(ModelInstanceIndex(3)) +
                   plant_.num_velocities(ModelInstanceIndex(2)) +
                   plant_.num_velocities(ModelInstanceIndex(3));
  target_input_port_ =
      this->DeclareVectorInputPort("desired_position", x_des_size).get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("lcmt_radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();

  actor_trajectory_port_ =
      this->DeclareAbstractOutputPort("c3_actor_trajectory_output",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &C3Controller::OutputActorTrajectory)
          .get_index();
  object_trajectory_port_ =
      this->DeclareAbstractOutputPort("c3_object_trajectory_output",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &C3Controller::OutputObjectTrajectory)
          .get_index();
  plan_start_time_index_ = DeclareDiscreteState(1);
  DeclareForcedDiscreteUpdateEvent(&C3Controller::ComputePlan);
}

drake::systems::EventStatus C3Controller::ComputePlan(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);
  const BasicVector<double>& x_des =
      *this->template EvalVectorInput<BasicVector>(context, target_input_port_);
  const FrankaKinematicsVector<double>* lcs_x =
      (FrankaKinematicsVector<double>*)this->EvalVectorInput(
          context, lcs_state_input_port_);
  discrete_state->get_mutable_value(plan_start_time_index_)[0] =
      lcs_x->get_timestamp();
  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
                     plant_.num_actuators());
  q_v_u << lcs_x->GetState(), VectorXd::Zero(n_u_);
  drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);

  VectorXd x_des_adjusted = x_des.value();
  VectorXd current = x_des_adjusted.head(n_q_).tail(3);
  current(0) += radio_out->channel[0] * 0.2;
  current(1) += radio_out->channel[1] * 0.2;
  current(2) += radio_out->channel[2] * 0.2;
  x_des_adjusted.head(n_q_).tail(3) = current;
  if (radio_out->channel[13] > 0){
    x_des_adjusted.head(3) = current;
  }
  //  std::cout << x_des_adjusted.transpose() << std::endl;

  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(N_ + 1, x_des_adjusted);

  int n_x = plant_.num_positions() + plant_.num_velocities();
  int n_u = plant_.num_actuators();

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x));
  plant_ad_.SetPositionsAndVelocities(context_ad_, q_v_u_ad.head(n_x));
  multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u), context_);
  multibody::SetInputsIfNew<drake::AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u),
                                               context_ad_);
  auto [lcs, scale] = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N);
  DRAKE_DEMAND(Q_.front().rows() == lcs.n_);
  DRAKE_DEMAND(Q_.front().cols() == lcs.n_);
  DRAKE_DEMAND(R_.front().rows() == lcs.k_);
  DRAKE_DEMAND(R_.front().cols() == lcs.k_);
  DRAKE_DEMAND(G_.front().rows() == lcs.n_ + lcs.m_ + lcs.k_);
  DRAKE_DEMAND(G_.front().cols() == lcs.n_ + lcs.m_ + lcs.k_);
  c3_ = std::make_unique<C3MIQP>(lcs, Q_, R_, G_, U_, x_desired, c3_options_);
  c3_->SetOsqpSolverOptions(solver_options_);

  VectorXd delta_init = VectorXd::Zero(n_x_ + n_lambda_ + n_u_);
  delta_init.head(n_x_) = lcs_x->get_data();
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(n_x_ + n_lambda_ + n_u_));

  // Set actor bounds
  for (int i : vector<int>({0, 2})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, 0.3, 0.7, 1);
  }
  for (int i : vector<int>({1})) {
    Eigen::RowVectorXd A = VectorXd::Zero(n_x_);
    A(i) = 1.0;
    c3_->AddLinearConstraint(A, -0.4, 0.4, 1);
  }
  auto z_sol = c3_->Solve(lcs_x->get_data(), delta, w);
  return drake::systems::EventStatus::Succeeded();
}

void C3Controller::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  MatrixXd x_sol = MatrixXd::Zero(n_q_ + n_v_, N_);
  MatrixXd lambda_sol = MatrixXd::Zero(n_lambda_, N_);
  MatrixXd u_sol = MatrixXd::Zero(n_u_, N_);
  VectorXd breaks = VectorXd::Zero(N_);

  for (int i = 0; i < N_; i++) {
    breaks(i) = t + i * c3_options_.dt;
    x_sol.col(i) = z_sol[i].segment(0, n_x_);
    lambda_sol.col(i) = z_sol[i].segment(n_x_, n_lambda_);
    u_sol.col(i) = z_sol[i].segment(n_x_ + n_lambda_, n_u_);
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = x_sol.topRows(3);
  knots.bottomRows(3) = x_sol.bottomRows(n_v_).topRows(3);
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_traj";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = breaks;
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_traj"},
                         "end_effector_traj", "end_effector_traj", false);

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory end_effector_orientation_traj;
    // first 3 rows are rpy, last 3 rows are angular velocity
    MatrixXd orientation_samples = MatrixXd::Zero(6, N_);
    orientation_samples.topRows(3) = x_sol.topRows(6).bottomRows(3);
    orientation_samples.bottomRows(3) =
        x_sol.bottomRows(n_v_).topRows(6).bottomRows(3);
    end_effector_orientation_traj.traj_name = "end_effector_orientation_target";
    end_effector_orientation_traj.datatypes =
        std::vector<std::string>(orientation_samples.rows(), "double");
    end_effector_orientation_traj.datapoints = orientation_samples;
    end_effector_orientation_traj.time_vector = breaks;
    lcm_traj.AddTrajectory(end_effector_orientation_traj.traj_name,
                           std::move(end_effector_orientation_traj));
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = t * 1e6;
}

void C3Controller::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  // TODO:yangwill, check to make sure the time being used here makes sense
  double t = context.get_discrete_state(plan_start_time_index_)[0];

  auto z_sol = c3_->GetFullSolution();
  MatrixXd x_sol = MatrixXd::Zero(n_q_ + n_v_, N_);
  MatrixXd lambda_sol = MatrixXd::Zero(n_lambda_, N_);
  MatrixXd u_sol = MatrixXd::Zero(n_u_, N_);
  VectorXd breaks = VectorXd::Zero(N_);

  for (int i = 0; i < N_; i++) {
    breaks(i) = t + i * c3_options_.dt;
    x_sol.col(i) = z_sol[i].segment(0, n_x_);
    lambda_sol.col(i) = z_sol[i].segment(n_x_, n_lambda_);
    u_sol.col(i) = z_sol[i].segment(n_x_ + n_lambda_, n_u_);
  }

  MatrixXd knots = MatrixXd::Zero(6, N_);
  knots.topRows(3) = x_sol.middleRows(n_q_ - 3, 3);
  knots.bottomRows(3) = x_sol.middleRows(n_q_ + n_v_ - 3, 3);
  LcmTrajectory::Trajectory object_traj;
  object_traj.traj_name = "object_traj";
  object_traj.datatypes = std::vector<std::string>(knots.rows(), "double");
  object_traj.datapoints = knots;
  object_traj.time_vector = breaks;
  LcmTrajectory lcm_traj({object_traj}, {"object_traj"}, "object_traj",
                         "object_traj", false);

  if (publish_end_effector_orientation_) {
    LcmTrajectory::Trajectory object_orientation_traj;
    // first 3 rows are rpy, last 3 rows are angular velocity
    MatrixXd orientation_samples = MatrixXd::Zero(7, N_);
    orientation_samples.topRows(4) = x_sol.middleRows(3 + 3, 4);
    orientation_samples.bottomRows(3) = x_sol.middleRows(n_q_ + 3 + 3, 3);
    object_orientation_traj.traj_name = "object_orientation_target";
    object_orientation_traj.datatypes =
        std::vector<std::string>(orientation_samples.rows(), "double");
    object_orientation_traj.datapoints = orientation_samples;
    object_orientation_traj.time_vector = breaks;
    lcm_traj.AddTrajectory(object_orientation_traj.traj_name,
                           std::move(object_orientation_traj));
  }

  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = t * 1e6;
}

}  // namespace systems
}  // namespace dairlib
