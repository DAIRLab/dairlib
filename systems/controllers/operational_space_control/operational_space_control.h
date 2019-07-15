#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"
#include "systems/controllers/operational_space_control/osc_tracking_data.h"

namespace dairlib {
namespace systems {
namespace controllers {


class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(
    std::vector<OscTrackingData*>* tracking_data_vec,
    RigidBodyTree<double>* tree_with_springs,
    RigidBodyTree<double>* tree_without_springs);

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

  // Cost methods //////////////////////////////////////////////////////////////
  // Setters
  // TODO: when you set the joint accel cost, you should call a function which
  // checks if the user enter the right index.
  void SetAccelerationCost(int joint_velocity_index, double weight);
  void SetAccelerationCostForAllJoints(Eigen::MatrixXd weight);

  void checkCostSettings();

  // Constraint methods ////////////////////////////////////////////////////////
  // Setters

  void checkConstraintSettings();

  // Tracking data methods /////////////////////////////////////////////////////
  void AddTrackingData(OscTrackingData* tracking_data);

  std::vector<OscTrackingData*>* GetAllTrackingData() {
    return tracking_data_vec_.get();
  }
  OscTrackingData* GetTrackingDataByIndex(int index) {
    return tracking_data_vec_->at(index);
  }

  //////////////////////////////////////////////////////////////////////////////

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        systems::TimestampedVector<double>* control) const;

  // Input/Output ports
  int state_port_;
  int fsm_port_;

  // Discrete update
  int prev_fsm_state_idx_;
  int prev_event_time_idx_;

  //
  std::map<std::string, int> traj_name_to_port_index_map_;

  // RBT's. OSC calculates feedback position/velocity from tree with springs,
  // but in the optimization it uses tree without springs. The reason of using
  // the one without spring is that the OSC cannot track desired acceleration
  // instantaneously when springs exist. (relative degrees of 4)
  // The springs here refer to the compliant components in the robots.
  RigidBodyTree<double>* tree_with_springs_;
  RigidBodyTree<double>* tree_without_springs_;
  //TODO: You'll send tree's in the function of CheckOscTrackingData to
  //calculate posistion, etc.:

  int n_q_;
  int n_v_;
  int n_u_;

  // Cost settings /////////////////////////////////////////////////////////////
  // Input cost
  Eigen::MatrixXd
  W_input_;  //TODO: you can check the size of the MatrixXd to know if the user want to add the cost

  // Joint acceleration cost
  std::vector<int> joint_velocity_index_;
  std::vector<double> w_joint_acceleration_;  // cost weight

  // Constraint settings ///////////////////////////////////////////////////////
  // Input constraint
  bool with_input_constraints_ = true;
  // std::vector<double> u_min_;
  // std::vector<double> u_max_;
  // Actually you can get the input limit from tree:
  //    umax_ = tree_without_->actuators[1].effort_limit_max_;

  // TODO(yminchen): Can you get it from RBT???
  // (flat ground) Contact constraints and friction cone cnostraints
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;
  double mu_;  // Friction coefficients
  Eigen::MatrixXd W_soft_constraints_;

  // Tracking data settings ////////////////////////////////////////////////////
  std::unique_ptr<std::vector<OscTrackingData*>> tracking_data_vec_ =
    std::make_unique<std::vector<OscTrackingData*>>(); // pointer because of caching
  int num_tracking_data_;

  //////////////////////////////////////////////////////////////////////////////

};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
