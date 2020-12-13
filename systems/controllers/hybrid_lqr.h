#pragma once

#include <stdlib.h>

#include "lcm/lcm_trajectory.h"
#include "multibody/kinematic/kinematic_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/initial_value_problem.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/event_status.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems {

/// This is an implementation of the hybrid lqr controller as well as
/// hybrid lqr with an adjusted reset map as described in Rijnen, Saccon,
/// Nijmeijer 2015. The contact info should be WorldPointEvaluators that
/// compute the distance of the foot with respect to the world frame
///
/// Note although the LQR gains are integrated backwards. The contact
/// info, state_traj, and input_traj should be supplied in forward time

class HybridLQRController : public drake::systems::LeafSystem<double> {
 public:
  HybridLQRController(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
      const vector<multibody::KinematicEvaluatorSet<drake::AutoDiffXd>*>&
          contact_info,
      const std::vector<drake::trajectories::PiecewisePolynomial<double>>&
          state_trajs,
      const std::vector<drake::trajectories::PiecewisePolynomial<double>>&
          input_trajs,
      const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& Qf, double buffer_time, bool adjusted_reset_map,
      std::string folder_path, bool recalculateP, bool recalculateL);


  const drake::systems::OutputPort<double>& get_output_port_control() const {
    return this->get_output_port(efforts_port_);
  }

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(input_port_fsm_);
  }

  const drake::systems::InputPort<double>& get_contact_input_port() const {
    return this->get_input_port(input_port_contact_);
  }

 private:

  static constexpr double ALPHA = 1e-8;
  static const int RESOLUTION = 1000;

  void CalcControl(const drake::systems::Context<double>& context,
                   TimestampedVector<double>* output) const;

  /*
   * Retrieve the minimal coordinate basis for a particular contact_mode at
   * time t
   */
  Eigen::MatrixXd getPAtTime(double t, int contact_mode) const;
  /*
   * Retrieve the quadratic cost term S for a particular contact_mode at
   * time t
   */
  Eigen::MatrixXd getSAtTime(double t, int contact_mode) const;

  // Computes the minimal coordinate basis P(t) for all the contact modes
  void calcMinimalCoordBasis();

  // Calculate L(t) for all modes given S_f, where S(t) = L(t) L(t)^T
  void calcCostToGo(const Eigen::MatrixXd& S_f);

  // Calculates matrices A, B for linearized dynamics of the form f(x) = Ax + Bu
  void calcLinearizedDynamics(double t, int contact_mode, Eigen::MatrixXd* A,
                              Eigen::MatrixXd* B);
  // Maps S_post to S_pre using the nominal reset map
  Eigen::MatrixXd calcJumpMap(const Eigen::MatrixXd& S_post, int contact_mode,
                              double t);
  // Maps S_post to S_pre using the Rijnen 2015 reset map
  Eigen::MatrixXd calcAdjustedJumpMap(Eigen::MatrixXd& S_post, int contact_mode,
                                      double t);
  // Calculates matrix R, such that the no-slip instantaneous reset map is given
  // by x_post = (I + R) x_pre
  void calcLinearResetMap(double t, int contact_mode, Eigen::MatrixXd* R);

  Eigen::VectorXd calcPdot(double t, const Eigen::VectorXd& P,
                           const Eigen::VectorXd&);

  Eigen::VectorXd calcLdot(double t, const Eigen::VectorXd& l,
                           const Eigen::VectorXd&);

  /*
   * Helper functions for integrating backwards in time
   */
  // This is in reverse time. As in the reverse time will fetch
  // the corresponding reverse contact mode
  int getContactModeAtTime(double t) const;
  // Get the corresponding reverse time. Only for use with trajectories
  double getReverseTime(double t) const;
  // Get the corresponding reverse mode
  int getReverseMode(int mode) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_;
  drake::systems::Context<double>* context_;
  drake::systems::Context<drake::AutoDiffXd>* context_ad_;
  const vector<multibody::KinematicEvaluatorSet<drake::AutoDiffXd>*>
      contact_info_;

  /*
   * Nominal/Target trajectories
   */
  const std::vector<drake::trajectories::PiecewisePolynomial<double>>
      state_trajs_;
  const std::vector<drake::trajectories::PiecewisePolynomial<double>>
      input_trajs_;
  /*
   * Trajectories of the minimal coordinate basis and S
   */
  std::vector<drake::trajectories::PiecewisePolynomial<double>> p_traj_;
  std::vector<drake::trajectories::PiecewisePolynomial<double>> l_traj_;

  // LQR cost matrices
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Qf_;
  Eigen::Matrix<double, 2, 3> TXZ_;

  /*
   * relevant dimensions
   * q: positions
   * v: velocities
   * x: q + v
   * u: actuators
   * c: contact constraints
   * d: constrained states
   */
  const int n_q_;
  const int n_v_;
  const int n_x_;
  const int n_u_;
  const int n_c_;
  const int n_d_;

  const int num_modes_;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  // Duration of zero effort around the nominal impact window
  double buffer_time_;

  std::vector<double> impact_times_;
  std::vector<double> impact_times_rev_;

  bool adjusted_reset_map_;
  const std::string folder_path_;

  // Store the l_traj in a different file depending on whether the gains are
  // calculated using the nominal or adjusted reset map
  std::string l_traj_filepath_;

  // Leaf system ports
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex input_port_fsm_;
  drake::systems::InputPortIndex input_port_contact_;
  drake::systems::OutputPortIndex efforts_port_;
};

}  // namespace dairlib::systems
