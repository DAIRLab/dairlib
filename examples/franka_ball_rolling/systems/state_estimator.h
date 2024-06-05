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
#include "examples/franka_ball_rolling/parameters/trajectory_params.h"
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

class StateEstimator : public LeafSystem<double> {
 public:
  /// A class that estimates the balls position and velocity from noisy position
  /// data and appends it to the robot output of the franka arm.  Intended use
  /// case is for C3 hardware experiments, where ball's position is derived from
  /// vision algorithms
  /// @param p_FIR_values A standard vector containing the FIR filter parameters
  /// for position filter
  /// @param v_FIR_values A standard vector containing the FIR filter parameters
  /// for velocity filter
  StateEstimator(const StateEstimatorParams& state_estimate_param_,
                 const BallRollingTrajectoryParams traj_param);

  /// the first input port take in franka state and input
  const drake::systems::InputPort<double>& get_input_port_franka() const {
    return this->get_input_port(franka_input_port_);
  }

  /// the second input port take in ball position
  const drake::systems::InputPort<double>& get_input_port_ball() const {
    return this->get_input_port(ball_input_port_);
  }

  /// the first output port send out franka states
  const drake::systems::OutputPort<double>& get_output_port_franka_state()
      const {
    return this->get_output_port(franka_state_output_port_);
  }
  /// the second output port send out franka actuation torques
  const drake::systems::OutputPort<double>& get_output_port_franka_effort()
      const {
    return this->get_output_port(franka_torque_output_port_);
  }
  /// the third output port send out ball states
  const drake::systems::OutputPort<double>& get_output_port_object_state()
      const {
    return this->get_output_port(ball_state_output_port_);
  }

 private:
  /// Do most of the calculation, including updating previous object position,
  /// performing finite differencing to get object velocity, and FIR filtering
  drake::systems::EventStatus UpdateHistory(
      const Context<double>& context,
      drake::systems::State<double>* state) const;

  /// Output the franka states, the actual calculation process is mainly done
  /// together with other calculation in UpdateHistory Event
  void EstimateFrankaState(const drake::systems::Context<double>& context,
                           BasicVector<double>* output) const;

  /// Output the franka actuation torques, the actual calculation process is
  /// mainly done together with other calculation in UpdateHistory Event
  void OutputFrankaEfforts(const drake::systems::Context<double>& context,
                           BasicVector<double>* output) const;

  /// Output the ball states, the actual calculation process is mainly done
  /// together with other calculation in UpdateHistory Event
  void EstimateObjectState(const drake::systems::Context<double>& context,
                           BasicVector<double>* output) const;

  /// Convenient function to convert angle-axis to rotation matrix, used for
  /// calculating orientation of the ball for its displacement (since we assume
  /// pure rolling)
  /// @param axis Vector3d axis
  /// @param axis double, the rotation angle
  drake::math::RotationMatrix<double> RodriguesFormula(const Vector3d& axis,
                                                       double theta) const;

  /// TODO: try to unplug StateEstimatorParams becuase it is only used to set
  /// initial estimation right now
  StateEstimatorParams state_estimate_param_;
  BallRollingTrajectoryParams traj_param_;

  /// estimated state, use drake discrete state to do calculation in Update
  /// history event
  drake::systems::DiscreteStateIndex p_idx_;
  drake::systems::DiscreteStateIndex orientation_idx_;
  drake::systems::DiscreteStateIndex v_idx_;
  drake::systems::DiscreteStateIndex w_idx_;

  /// deques for estimated state history and filtering, use drake abstract state
  /// to do calculation in Update history event
  drake::systems::AbstractStateIndex p_history_idx_;
  drake::systems::AbstractStateIndex v_history_idx_;
  drake::systems::AbstractStateIndex prev_time_idx_;
  drake::systems::AbstractStateIndex prev_id_idx_;

  /// Input and output ports index
  drake::systems::InputPortIndex franka_input_port_;
  drake::systems::InputPortIndex ball_input_port_;
  drake::systems::OutputPortIndex franka_state_output_port_;
  drake::systems::OutputPortIndex franka_torque_output_port_;
  drake::systems::OutputPortIndex ball_state_output_port_;

  /// FIR filter design values
  std::vector<double> p_FIR_values_;
  std::vector<double> v_FIR_values_;
  const int p_filter_length_;
  const int v_filter_length_;

  /// useful variables
  /// franka is fully actuated 7 DOF arm
  const int num_franka_positions_{7};
  const int num_franka_velocities_{7};
  const int num_franka_efforts_{7};
  /// ball is isolated rigid body, orientation use quaternion
  const int num_ball_positions_{7};
  const int num_ball_velocities_{6};
};

class TrueBallToEstimatedBall : public LeafSystem<double> {
 public:
  /// A class that takes in ball state and output the noisy-added ball state to
  /// the downstream estimator block. This is designed to mimic the behaviour of
  /// the true camera vision system i.e. it only outputs updated ball positions
  /// every 'period' seconds and adds random (zero meam) gaussian noise to the x
  /// and y positions
  /// @param stddev gaussian noise standard deviation
  /// @param period camera system update period (1 / frequency)
  TrueBallToEstimatedBall(const StateEstimatorParams state_estimate_param,
                          const BallRollingTrajectoryParams traj_param);

  /// the input port take in true ball state
  const drake::systems::InputPort<double>& get_input_port_true_ball() const {
    return this->get_input_port(true_ball_input_port_);
  }
  /// the output port send out noisy estimated ball state
  const drake::systems::OutputPort<double>& get_output_port_estimated_ball()
      const {
    return this->get_output_port(estimated_ball_output_port_);
  }

 private:
  /// Adding (zero-mean) gaussian noise to the ball estimation
  drake::systems::EventStatus UpdateBallPosition(
      const Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  /// Converting noisy estimation to corresponding lcmt_ball_position data
  /// structure, mimicking real hardware camera setting
  void ConvertOutput(const drake::systems::Context<double>& context,
                     dairlib::lcmt_ball_position* output) const;

  /// TODO: try to unplug StateEstimatorParams becuase it is only used to set
  /// initial estimation right now
  StateEstimatorParams state_estimate_param_;
  BallRollingTrajectoryParams traj_param_;

  /// position, time and id, use drake discrete state to do calculation in
  /// Update history event
  drake::systems::DiscreteStateIndex p_idx_;
  drake::systems::DiscreteStateIndex id_idx_;
  drake::systems::DiscreteStateIndex utime_idx_;

  /// Input and output ports index
  drake::systems::InputPortIndex true_ball_input_port_;
  drake::systems::OutputPortIndex estimated_ball_output_port_;

  /// noise parameters, derived from
  const double stddev_;
  const double period_;

  /// useful variables
  /// ball is isolated rigid body, orientation use quaternion
  const int num_ball_positions_{7};
  const int num_ball_velocities{6};
};

}  // namespace systems
}  // namespace dairlib