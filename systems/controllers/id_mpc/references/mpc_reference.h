#pragma once

#include "dairlib/lcmt_id_mpc_reference.hpp"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace dairlib::systems::controllers::id_mpc {

class MPCReference {
 public:
  /*!
   * Default reference trajectories for a floating base robot
   */
  drake::trajectories::PiecewisePolynomial<double> q_traj_;
  drake::trajectories::PiecewisePolynomial<double> quat_traj_;
  drake::trajectories::PiecewisePolynomial<double> v_traj_;
  drake::trajectories::PiecewisePolynomial<double> lambda_traj_;
  drake::trajectories::PiecewisePolynomial<double> u_traj_;

  /*!
   * Stash the desired velocity for locomotion controllers
   */
   Eigen::Vector2d vdes_;

  /*!
   * Reference trajectories in task space
   */
  std::unordered_map<
      std::string,
      drake::trajectories::PiecewisePolynomial<double>> task_space_trajs_;

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
  std::vector<std::string> touchdown_ee_names_to_update_;
  std::vector<double> single_stance_phase_;

  void AppendContactsToKnot(int i, const std::vector<std::string> &contacts) {
    active_contacts_.at(i).insert(active_contacts_.at(i).end(),
                                  contacts.begin(), contacts.end());
  }

};

lcmt_id_mpc_reference ConvertToLcm(const MPCReference& ref, double timestamp);

}