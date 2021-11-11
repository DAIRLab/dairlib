#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc {

class PelvisTransTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisTransTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::trajectories::PiecewisePolynomial<double>& traj,
      const std::unordered_map<
          int, std::vector<std::pair<const Eigen::Vector3d,
                                     const drake::multibody::Frame<double>&>>>&
          feet_contact_points);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_clock_input_port() const {
    return this->get_input_port(clock_port_);
  }

  void SetSLIPParams(double rest_length, double k_leg, double b_leg){
    rest_length_ = rest_length;
    k_leg_ = k_leg;
    b_leg_ = rest_length;
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> GeneratePelvisTraj(
      const Eigen::VectorXd& x, double t, int fsm_state) const;

  drake::trajectories::PiecewisePolynomial<double> GenerateSLIPTraj(
      const Eigen::VectorXd& x, double t, int fsm_state) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;
  const drake::multibody::BodyFrame<double>& pelvis_frame_;

  //  drake::systems::DiscreteStateIndex prev_fsm_idx_;

  // pelvis trajectory
  drake::trajectories::PiecewisePolynomial<double> traj_;

  // A list of pairs of contact body frame and contact point
  const std::unordered_map<
      int, std::vector<std::pair<const Eigen::Vector3d,
                                 const drake::multibody::Frame<double>&>>>&
      feet_contact_points_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex clock_port_;

  // SLIP parameters
  double rest_length_ = 0.8;
  double k_leg_ = 100.0;
  double b_leg_ = 5.0;
};

}  // namespace dairlib::examples::osc
