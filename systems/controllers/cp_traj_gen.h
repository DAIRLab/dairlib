#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

#include "systems/framework/output_vector.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"


using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::EventStatus;
using drake::systems::BasicVector;

using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;

namespace dairlib {
namespace systems {



class CPTrajGenerator : public LeafSystem<double> {
 public:
  CPTrajGenerator(RigidBodyTree<double> * tree,
                  double mid_foot_height,
                  double max_CoM_to_CP_dist,
                  double stance_duration_per_leg,
                  int left_stance_state,
                  int right_stance_state,
                  int left_foot_idx,
                  int right_foot_idx,
                  int pelvis_idx,
                  bool is_walking_position_control,
                  bool is_feet_collision_avoid,
                  bool is_print_info);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_FSM() const {
    return this->get_input_port(FSM_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_com() const {
    return this->get_input_port(com_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fp() const {
    return this->get_input_port(fp_port_);
  }

 private:
  EventStatus DiscreteVariableUpdate(const Context<double>& context,
                                     DiscreteValues<double>* discrete_state) const;
  void CalcTrajs(const Context<double>& context,
                 PiecewisePolynomial<double>* traj) const;

  int state_port_;
  int FSM_port_;
  int com_port_;
  int fp_port_;

  int prev_td_swing_foot_idx_;
  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  bool is_quaternion_;

  RigidBodyTree<double> * tree_;
  double mid_foot_height_;
  double max_CoM_to_CP_dist_;
  double stance_duration_per_leg_;
  int left_stance_;
  int right_stance_;
  int left_foot_idx_;
  int right_foot_idx_;
  int pelvis_idx_;
  bool is_walking_position_control_;
  bool is_feet_collision_avoid_;
  bool is_print_info_;
};







} //namespace systems
} //namespace dairlib


