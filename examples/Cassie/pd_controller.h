#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"

namespace drake{
using Eigen::VectorXd;
using dairlib::systems::TimestampedVector;


class CassiePDController : public systems::LeafSystem<double> {
 public:
  CassiePDController();

  int config_input_port() {return config_input_port_;};
  int state_input_port() {return state_input_port_;};

 private:
  void CalcControl(const systems::Context<double>& context, TimestampedVector<double>* output) const;

  int state_input_port_;
  int config_input_port_;
  int num_joints_;

  VectorXd q_des_;
  VectorXd v_des_;
  VectorXd kp_;
  VectorXd kd_;
};
}