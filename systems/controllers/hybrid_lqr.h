#pragma once

#include <stdlib.h>

#include "drake/systems/framework/event_status.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

/// This is an implementation of the naive hybrid lqr controller as well as
/// hybrid lqr with an adjusted reset map as described in Rijnen, Saccon,
/// Nijmeijer 2015. The contact info should be WorldPointEvaluators that
/// compute the distance of the foot with respect to the world frame
///
/// Note although the LQR gains must be integrated backwards. The contact
/// info, state_traj, and input_traj should be given in forward time
/// Also note that the size of impact_times must be one less than the size of
/// of the contact_info vector
static const double ALPHA = 1e-8;

class HybridLQRController : public drake::systems::LeafSystem<double> {
 public:
  HybridLQRController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const vector<multibody::WorldPointEvaluator<double>> contact_info,
      const vector<multibody::WorldPointEvaluator<drake::AutoDiffXd>>
          contact_info_ad,
      const std::vector<
          std::shared_ptr<drake::trajectories::PiecewisePolynomial<double>>>&
          state_trajs,
      const std::vector<
          std::shared_ptr<drake::trajectories::PiecewisePolynomial<double>>>&
          input_trajs,
      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& Qf, const double buffer_time,
      bool adjusted_reset_map, std::string folder_path, bool recalculateP,
      bool recalculateL);

  const drake::systems::OutputPort<double>& get_output_port_control() const {
    return this->get_output_port(control_output_port_);
  }

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

  const drake::systems::InputPort<double>& get_contact_input_port() const {
    return this->get_input_port(contact_port_);
  }

 private:
  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;
  Eigen::VectorXd calcLdot(double t, const Eigen::VectorXd& l,
                           const Eigen::VectorXd&);
  Eigen::VectorXd calcPdot(double t, const Eigen::VectorXd& P,
                           const Eigen::VectorXd&);
  Eigen::MatrixXd calcJumpMap(Eigen::MatrixXd& s_pre_impact, int contact_mode,
                              double t);
  Eigen::MatrixXd calcJumpMapNaive(const Eigen::MatrixXd& S_pre,
                                   int contact_mode, double t);
  void calcLinearizedDynamics(double t, int contact_mode, Eigen::MatrixXd* A,
                              Eigen::MatrixXd* B);
  void calcLinearResetMap(double t, int contact_mode, Eigen::MatrixXd* R);
  // Precompute the minimal coordinate basis for all the contact modes
  void calcMinimalCoordBasis();

  void calcCostToGo(const Eigen::MatrixXd& S_f);

  Eigen::MatrixXd getPAtTime(double t, int contact_mode) const;
  Eigen::MatrixXd getSAtTime(double t, int contact_mode) const;

  // This is in reverse time. As in the reverse time will fetch
  // the corresponding reverse contact mode
  int getContactModeAtTime(double t) const;
  // Get the corresponding reverse time. Only for use with trajectories
  double getReverseTime(double t) const;
  // Get the corresponding reverse mode
  int getReverseMode(int mode) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  const vector<multibody::KinematicEvaluator<double>> contact_info_;
  const vector<multibody::KinematicEvaluator<drake::AutoDiffXd>>
      contact_info_ad_;

  const std::vector<
      std::shared_ptr<drake::trajectories::PiecewisePolynomial<double>>>
      state_trajs_;
  const std::vector<
      std::shared_ptr<drake::trajectories::PiecewisePolynomial<double>>>
      input_trajs_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Qf_;
  const std::vector<double> impact_times_;
  const std::string folder_path_;
  bool adjusted_reset_map_;
  std::vector<double> impact_times_rev_;

  std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>> l_t_;
  std::vector<std::unique_ptr<drake::systems::DenseOutput<double>>> p_t_;
  std::vector<drake::trajectories::PiecewisePolynomial<double>> p_traj_;
  std::vector<drake::trajectories::PiecewisePolynomial<double>> l_traj_;

  Eigen::Matrix<double, 2, 3> TXZ_;
  const int n_q_;
  const int n_v_;
  const int n_x_;
  const int n_u_;
  const int n_c_;
  const int n_d_;
  const int num_modes_;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;
  double buffer_time_;

  std::string l_traj_filepath_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex contact_port_;
  drake::systems::OutputPortIndex control_output_port_;

};

}  // namespace dairlib::systems
