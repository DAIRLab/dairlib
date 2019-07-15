#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "systems/controllers/operational_space_control/osc_tracking_data.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace systems {
namespace controllers {


class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(
    RigidBodyTree<double>* tree_w_spr,
    RigidBodyTree<double>* tree_wo_spr);

  // Input/output ports
  const drake::systems::InputPort<double>& get_robot_output_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_tracking_data_input_port(
    std::string name) const {
    // int port_idx = traj_name_to_port_index_map_.find(name)->second;
    return this->get_input_port(traj_name_to_port_index_map_.at(name));
  }

  // Cost methods
  void SetInputCost(Eigen::MatrixXd W) {W_input_ = W;}
  void SetAccelerationCostForAllJoints(Eigen::MatrixXd W) {W_joint_accel_ = W;}
  void AddAccelerationCost(int joint_vel_idx, double w);

  // Constraint methods
  void DisableAcutationConstraint() {with_input_constraints_ = false;}
  void SetContactFriction(double mu) {mu_ = mu;}
  void SetWeightOfSoftContactConstraint(double w_soft_constraint) {
    w_soft_constraint_ = w_soft_constraint;
  }
  void AddContactPoint(int body_index, Eigen::VectorXd pt_on_body);
  void AddContactPoints(std::vector<int> body_index,
                        std::vector<Eigen::VectorXd> pt_on_body);

  // Tracking data methods
  void AddTrackingData(OscTrackingData* tracking_data){
    tracking_data_vec_->push_back(tracking_data);
  }
  std::vector<OscTrackingData*>* GetAllTrackingData() {
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index);
  }

  // Osc constructor
  void ConstructOSC();

 private:
  // Osc checkers and constructor related methods
  void CheckCostSettings();
  void CheckConstraintSettings();
  drake::solvers::MathematicalProgram SetUpQp() const;

  // Discrete update that stores the previous state transition time
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  // Output function
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        systems::TimestampedVector<double>* control) const;

  // Input/Output ports
  int state_port_;
  int fsm_port_;

  // Discrete update
  int prev_fsm_state_idx_;
  int prev_event_time_idx_;

  // Map from trajectory names to input port indices
  std::map<std::string, int> traj_name_to_port_index_map_;

  // RBT's. OSC calculates feedback position/velocity from tree with springs,
  // but in the optimization it uses tree without springs. The reason of using
  // the one without spring is that the OSC cannot track desired acceleration
  // instantaneously when springs exist. (relative degrees of 4)
  // The springs here refer to the compliant components in the robots.
  RigidBodyTree<double>* tree_w_spr_;
  RigidBodyTree<double>* tree_wo_spr_;
  //TODO: You'll send tree's in the function of CheckOscTrackingData to
  //calculate posistion, etc.:

  // Size of position, velocity and input
  int n_q_;
  int n_v_;
  int n_u_;

  // OSC cost members
  Eigen::MatrixXd W_input_;  // Input cost weight
  Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight

  // OSC constraint members
  bool with_input_constraints_ = true;
  // std::vector<double> u_min_;
  // std::vector<double> u_max_;
  // Actually you can get the input limit from tree:
  //    umax_ = tree_without_->actuators[1].effort_limit_max_;

  // (flat ground) Contact constraints and friction cone cnostraints
  std::vector<int> body_index_ = {};
  std::vector<Eigen::VectorXd> pt_on_body_ = {};
  double mu_ = -1;  // Friction coefficients
  double w_soft_constraint_ = -1;
  // TODO(yminchen): Can you get contact points from RBT???

  // OSC tracking data member (store pointer because of caching)
  std::unique_ptr<std::vector<OscTrackingData*>> tracking_data_vec_ =
        std::make_unique<std::vector<OscTrackingData*>>();
};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
