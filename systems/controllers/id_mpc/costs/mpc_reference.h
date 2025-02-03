#pragma once

#include "drake/common/trajectories/piecewise_polynomial.h"

// TODO (Brian-Acosta) - Support more general reference
//  trajectories, including task-space references

class MPCReference {
 public:
  /*!
   * Reference trajectories for various quantities
   */
  drake::trajectories::PiecewisePolynomial<double> q_traj_;
  drake::trajectories::PiecewisePolynomial<double> quat_traj_;
  drake::trajectories::PiecewisePolynomial<double> v_traj_;
  drake::trajectories::PiecewisePolynomial<double> lambda_traj_;
  drake::trajectories::PiecewisePolynomial<double> u_traj_;

  /*! timestamp of each MPC knot */
  std::vector<double> knot_times_;

  /*! which contacts are active at each knot point */
  std::vector<std::vector<std::string>> active_contacts_;

  /*!
   * List of the names of end effectors which are making contact
   * at each knot point. If a knot point does not contain a touchdown event,
   * the name should be an empty string
   */
  std::vector<std::string> touchdown_ee_names_;
  std::vector<Eigen::Vector3d> touchdown_ee_points_;
};
