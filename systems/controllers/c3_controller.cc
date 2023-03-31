#include "c3_controller.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "solvers/lcs_factory.h"

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
      U_(std::vector<MatrixXd>(c3_options_.N, c3_options_.U)) {
  //  DRAKE_DEMAND(Q_[0].rows() == lcs.A_[0].rows());
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

  VectorXd q_v_u =
      VectorXd::Zero(plant_.num_positions() + plant_.num_velocities() +
                     plant_.num_actuators());
  q_v_u << x.get_data(), VectorXd::Zero(n_u_);
  drake::AutoDiffVecXd q_v_u_ad = drake::math::InitializeAutoDiff(q_v_u);

  std::vector<VectorXd> x_desired =
      std::vector<VectorXd>(Q_.size() + 1, VectorXd::Zero(n_q_ + n_v_));
  //  q_v_u[0] = 1;
  //  q_v_u[4] = 1;
  //  q_v_u_ad[0] = 1;
  //  q_v_u_ad[4] = 1;

  int n_x = plant_.num_positions() + plant_.num_velocities();
  int n_u = plant_.num_actuators();

  plant_.SetPositionsAndVelocities(context_, q_v_u.head(n_x));
  plant_ad_.SetPositionsAndVelocities(context_ad_, q_v_u_ad.head(n_x));
  auto lcs = LCSFactory::LinearizePlantToLCS(
      plant_, *context_, plant_ad_, *context_ad_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, c3_options_.dt,
      c3_options_.N);
  c3_ = std::make_unique<C3MIQP>(lcs.first, Q_, R_, G_, U_, x_desired,
                                 c3_options_);

  std::vector<VectorXd> delta(N_, VectorXd::Zero(1));
  std::vector<VectorXd> w(N_, VectorXd::Zero(1));
  c3_->Solve(x.get_data(), delta, w);
  *output_traj = dairlib::lcmt_timestamped_saved_traj();
}

}  // namespace systems
}  // namespace dairlib
