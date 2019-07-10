#pragma once

#include <vector>
#include <string>
#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {


class OperationalSpaceControl : public drake::systems::LeafSystem<double> {
 public:
  OperationalSpaceControl(OscTrackingDataSet* tracking_data_set);

  // Input/output ports
  const drake::systems::InputPort<double>& get_robot_output_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(FSM_port_);
  }
  const drake::systems::InputPort<double>& get_tracking_data_input_port(
    std::string name) const {
    return this->get_input_port(traj_name_to_port_index_map_.find(name));
  }

  // Cost methods //////////////////////////////////////////////////////////////
  // Setters
  // TODO: when you set the joint accel cost, you should call a function which
  // checks if the user enter the right index.
  void SetAccelerationCost(int joint_velocity_index, double weight);
  void SetAccelerationCostForAllJoints(MatrixXd weight);

  void checkCostSettings();

  // Constraint methods ////////////////////////////////////////////////////////
  // Setters

  void checkConstraintSettings();

  // Tracking data methods /////////////////////////////////////////////////////
  OscTrackingDataSet* GetTrackingDataSet() {
    return tracking_data_set_;
  }

  // I don't think we need this
  // API for the user to add input ports if they create the traj source themselves
  void AddTrackingDataInputPort(string name);

 private:
  void CalcOptimalInput(const drake::systems::Context<double>& context,
                        dairlab::systems::TimestampedVector<double>* control) const;

  // Input/Output ports
  int state_port_;
  int FSM_port_;

  //
  map<string, int> traj_name_to_port_index_map_;

  // RBT's. OSC calculates feedback position/velocity from tree with springs,
  // but in the optimization it uses tree without springs. The reason of using
  // the one without spring is that the OSC cannot track desired acceleration
  // instantaneously when springs exist. (relative degrees of 4)
  // The springs here refer to the compliant components in the robots.
  RigidBodyTree<double>* tree_with_springs_;
  RigidBodyTree<double>* tree_without_springs_;
  //TODO: You'll send tree's in the function of CheckOscTrackingData to
  //calculate posistion, etc.:

  // Cost settings /////////////////////////////////////////////////////////////
  // Input cost
  MatrixXd W_input_;  //TODO: you can check the size of the MatrixXd to know if the user want to add the cost

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
  MatrixXd W_soft_constraints_;

  // Tracking data settings ////////////////////////////////////////////////////
  OscTrackingDataSet* tracking_data_set_;  // pointer because of caching

  //////////////////////////////////////////////////////////////////////////////

};


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
