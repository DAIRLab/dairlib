#include "c3_controller.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "solvers/lcs_factory.h"

#include "drake/solvers/moby_lcp_solver.h"

namespace dairlib {

using drake::systems::BasicVector;
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
  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();

  target_input_port_ = this->DeclareVectorInputPort("desired_position",
                                                    BasicVector<double>(2 + 16))
                           .get_index();
  lcs_state_input_port_ =
      this->DeclareVectorInputPort("x_lcs",
                                   TimestampedVector<double>(n_q_ + n_v_))
          .get_index();

  this->set_name("c3_controller");
  trajectory_output_port_ =
      this->DeclareAbstractOutputPort("c3_output",
                                      dairlib::lcmt_timestamped_saved_traj(),
                                      &C3Controller::OutputTrajectory)
          .get_index();
}

void C3Controller::OutputTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  //  const BasicVector<double>& x_des =
  //      *this->template EvalVectorInput<BasicVector>(context,
  //      target_input_port_);
  const TimestampedVector<double>& x =
      *this->template EvalVectorInput<TimestampedVector>(context,
                                                         lcs_state_input_port_);
  double t = x.get_timestamp();
  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
                     plant_.num_actuators());
  q_v_u << x.get_data(), VectorXd::Zero(n_u_);
  drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);

  VectorXd x_des = VectorXd::Zero(n_q_ + n_v_);
  x_des[0] = 0.7;
  x_des[1] = 0.02;
  x_des[2] = 0.35;
  x_des[3] = 1;
  x_des[4] = 0;
  x_des[5] = 0;
  x_des[6] = 0;
  x_des[7] = 0.4;
  x_des[8] = -0.2;
  x_des[9] = 0.45;
  std::vector<VectorXd> x_desired = std::vector<VectorXd>(N_ + 1, x_des);
//  std::cout << "x value: " << x.get_data() << std::endl;
//  std::cout << "x_desired: " << (x_des - x.get_data()).transpose() << std::endl;
  std::cout << "plate_error: " << (x_des.segment(7, 3) - x.get_data().segment(7, 3)).transpose() << std::endl;


  int n_x = plant_.num_positions() + plant_.num_velocities();
  int n_u = plant_.num_actuators();

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x));
  plant_ad_.SetPositionsAndVelocities(context_ad_, q_v_u_ad.head(n_x));
  multibody::SetInputsIfNew<double>(plant_, q_v_u.tail(n_u), context_);
  multibody::SetInputsIfNew<drake::AutoDiffXd>(plant_ad_, q_v_u_ad.tail(n_u), context_ad_);
  auto lcs_pair = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N);
  auto lcs = lcs_pair.first;
  DRAKE_DEMAND(Q_.front().rows() == lcs.n_);
  DRAKE_DEMAND(Q_.front().cols() == lcs.n_);
  DRAKE_DEMAND(R_.front().rows() == lcs.k_);
  DRAKE_DEMAND(R_.front().cols() == lcs.k_);
  DRAKE_DEMAND(G_.front().rows() == lcs.n_ + lcs.m_ + lcs.k_);
  DRAKE_DEMAND(G_.front().cols() == lcs.n_ + lcs.m_ + lcs.k_);
  c3_ = std::make_unique<C3MIQP>(lcs, Q_, R_, G_, U_, x_desired, c3_options_);
  c3_->SetOsqpSolverOptions(solver_options_);

  int n = ((lcs.A_)[0].cols());
  int m = ((lcs.D_)[0].cols());
  int k = ((lcs.B_)[0].cols());
  VectorXd delta_init = VectorXd::Zero(n + m + k);
  delta_init.head(n) = x.get_data();
  std::vector<VectorXd> delta(N_, delta_init);
  std::vector<VectorXd> w(N_, VectorXd::Zero(n + m + k));
  auto z_sol = c3_->Solve(x.get_data(), delta, w);

  MatrixXd x_sol = MatrixXd::Zero(lcs.n_, N_);
  MatrixXd lambda_sol = MatrixXd::Zero(lcs.m_, N_);
  MatrixXd u_sol = MatrixXd::Zero(lcs.k_, N_);
  VectorXd breaks = VectorXd::Zero(N_);

  for (int i = 0; i < N_; i++) {
    breaks(i) = t + i * lcs.dt_;
    x_sol.col(i) = z_sol[i].segment(0, lcs.n_);
    lambda_sol.col(i) = z_sol[i].segment(lcs.n_, lcs.m_);
    u_sol.col(i) = z_sol[i].segment(lcs.n_ + lcs.m_, lcs.k_);
  }

//  double solve_dt = c3;
  auto second_lcs_pair = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.solve_dt,
      c3_options_.N);
  auto second_lcs = second_lcs_pair.first;
  auto second_scale = second_lcs_pair.second;
  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;
  auto flag = LCPSolver.SolveLcpLemkeRegularized(
      second_lcs.F_[0],
      second_lcs.E_[0] * second_scale * x.get_data() +
          second_lcs.c_[0] * second_scale +
          second_lcs.H_[0] * second_scale * u_sol.col(0),
      &force);

  (void)flag;  // suppress compiler unused variable warning
  VectorXd state_next = second_lcs.A_[0] * x.get_data() + second_lcs.B_[0] * u_sol.col(0) +
                        second_lcs.D_[0] * force / second_scale +
                        second_lcs.d_[0];

  x_sol.col(0) = state_next;
  x_sol.col(1) = state_next;
  x_sol.col(2) = state_next;
  x_sol.col(3) = state_next;
  x_sol.col(4) = state_next;
//  x_sol.col(N_ - 1) = x.get_data();

  MatrixXd knots = x_sol.topRows(3);
  LcmTrajectory::Trajectory end_effector_traj;
  end_effector_traj.traj_name = "end_effector_traj";
  end_effector_traj.datatypes =
      std::vector<std::string>(knots.rows(), "double");
  end_effector_traj.datapoints = knots;
  end_effector_traj.time_vector = breaks;
  LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_traj"},
                         "end_effector_traj", "end_effector_traj", false);
  output_traj->saved_traj = lcm_traj.GenerateLcmObject();
  output_traj->utime = t * 1e6;
}

}  // namespace systems
}  // namespace dairlib
