#pragma once

#include <cmath>
#include <deque>
#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <drake/math/rotation_matrix.h>
#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/vector_base.h>

#include "dairlib/lcmt_ball_position.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/franka_ball_rolling/parameters/state_estimator_params.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "yaml-cpp/yaml.h"

#include "drake/common/yaml/yaml_io.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

// A class that estimates the balls position and velocity
// from noisy position data and appends it to the robot output
// of the franka arm.  Intended use case is for C3 experiments.
class StateEstimator : public LeafSystem<double> {
 public:
  StateEstimator(const std::vector<double>& p_FIR_values,
                 const std::vector<double>& v_FIR_values);

  const drake::systems::OutputPort<double>& get_output_port_franka_state()
      const {
    return this->get_output_port(franka_state_output_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_object_state()
      const {
    return this->get_output_port(ball_state_output_port_);
  }

 private:
  // update discrete states
  drake::systems::EventStatus UpdateHistory(
      const Context<double>& context,
      drake::systems::State<double>* state) const;

  // estimates state
  void EstimateState(const drake::systems::Context<double>& context,
                     BasicVector<double>* output) const;
  void EstimateFrankaState(const drake::systems::Context<double>& context,
                           BasicVector<double>* output) const;
  void EstimateObjectState(const drake::systems::Context<double>& context,
                           BasicVector<double>* output) const;
  void OutputEfforts(const drake::systems::Context<double>& context,
                     BasicVector<double>* output) const;
  drake::math::RotationMatrix<double> RodriguesFormula(const Vector3d& axis,
                                                       double theta) const;

  StateEstimatorParams state_estimate_param_;

  // deques for tracking history, use drake state (indices)
  int p_idx_;
  int orientation_idx_;
  int v_idx_;
  int w_idx_;
  int p_history_idx_;
  int v_history_idx_;
  int prev_time_idx_;
  int prev_id_idx_;

  // port indices
  int franka_input_port_;
  int ball_input_port_;

  drake::systems::OutputPortIndex franka_state_output_port_;
  drake::systems::OutputPortIndex ball_state_output_port_;

  // FIR filter design
  std::vector<double> p_FIR_values_;
  std::vector<double> v_FIR_values_;
  const int p_filter_length_;
  const int v_filter_length_;

  // useful variables
  // franka is fully actuated 7 DOF arm
  const int num_franka_positions_{7};
  const int num_franka_velocities_{7};
  const int num_franka_efforts_{7};
  // ball is isolated rigid body, orientation use quaternion
  const int num_ball_positions_{7};
  const int num_ball_velocities_{6};
  const int num_ball_efforts_{0};
};

// A class that converts robot output lcm messages
// from simulation of the franka-ball plant to a
// ball position lcm message.
// This is designed to mimic the behaviour of the true camera system
// i.e. it only outputs updated ball positions every 'period' seconds
// and adds random guassian noise to the x and y positions
class TrueBallToEstimatedBall : public LeafSystem<double> {
 public:
  TrueBallToEstimatedBall(double stddev, double period);

 private:
  drake::systems::EventStatus UpdateBallPosition(
      const Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void ConvertOutput(const drake::systems::Context<double>& context,
                     dairlib::lcmt_ball_position* output) const;

  StateEstimatorParams state_estimate_param_;

  // tracking position, time and id history, use drake state (indices)
  int p_idx_;
  int id_idx_;
  int utime_idx_;
  int true_ball_input_port_;

  const double stddev_;
  const double period_;
  const int num_ball_positions_{7};
  const int num_ball_velocities{6};
};

}  // namespace systems
}  // namespace dairlib