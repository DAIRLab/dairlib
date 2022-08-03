#pragma once

#include <vector>
#include <utility>
#include <iostream>
#include <deque>


#include <Eigen/Core>
#include <Eigen/Dense>

#include "systems/framework/output_vector.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/framework/context.h"

#include <drake/systems/framework/continuous_state.h>
#include <drake/systems/framework/vector_base.h>
#include "examples/franka_trajectory_following/c3_parameters.h"
#include "yaml-cpp/yaml.h"
#include "drake/common/yaml/yaml_io.h"

using drake::systems::Context;
using drake::systems::LeafSystem;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

namespace dairlib {
namespace systems {

// A class that estimates the balls position and velocity
// from noisey position data and appends it to the robot output
// of the franka arm.  Intended use case is for C3 experiments.
class C3StateEstimator : public LeafSystem<double> {
 public:
  C3StateEstimator(const std::vector<double>& p_FIR_values,
                   const std::vector<double>& v_FIR_values);
  
 private: 
  // update discrete states
  drake::systems::EventStatus UpdateHistory(
    const Context<double>& context,
    drake::systems::State<double>* state) const;

  // estimates state
  void EstimateState(const drake::systems::Context<double>& context,
                    OutputVector<double>* output) const;
  
  C3Parameters param_;

  // deques for tracking history
  int p_idx_;
  int v_idx_;
  int w_idx_;
  int p_history_idx_;
  int v_history_idx_;
  int prev_time_idx_;

  // port indices
  int franka_input_port_;
  int ball_input_port_;

  std::vector<double> p_FIR_values_;
  std::vector<double> v_FIR_values_;
  const int p_filter_length_;
  const int v_filter_length_;

  // useful variables
  const int num_franka_positions_{7};
  const int num_franka_velocities_{7};
  const int num_franka_efforts_{7};
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
class FrankaBallToBallPosition : public LeafSystem<double> {
 public:
   FrankaBallToBallPosition(double stddev, double period);
 
 private:
  drake::systems::EventStatus UpdateBallPosition(
      const Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void ConvertOutput(const drake::systems::Context<double>& context,
                    dairlib::lcmt_ball_position* output) const;

  int p_idx_;
  int id_idx_;
  int utime_idx_;

  // noise generating member variables
  C3Parameters param_;
  std::random_device rd_{};
  std::mt19937 gen_{rd_()};
  std::normal_distribution<> distribution_{};

  const double stddev_;
  const double period_;
}

}  // namespace systems
}  // namespace dairlib