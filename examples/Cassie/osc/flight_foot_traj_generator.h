#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc {

class FlightFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FlightFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      double period, bool is_left_ft);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> generateSwingTraj(
      const Eigen::VectorXd& x, double t) const;
  drake::trajectories::PiecewisePolynomial<double> generateStanceTraj(
      const Eigen::VectorXd& x, double t) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::Frame<double>& world_;

  double period_;
  double center_line_offset_;
  bool is_left_ft_;
  int state_port_;
  int fsm_port_;
};

}  // namespace dairlib::examples::osc
