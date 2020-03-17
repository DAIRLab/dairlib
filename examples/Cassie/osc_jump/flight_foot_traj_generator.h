#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"
#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"


namespace dairlib::examples::Cassie::osc_jump {

class FlightFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FlightFootTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                 const std::string& hip_name,
                 bool isLeftFoot,
                 const drake::trajectories::PiecewisePolynomial<double>&
                     foot_traj,
                 double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> generateFlightTraj(
      const drake::systems::Context<double>& context,
      const Eigen::VectorXd& x,
      double t) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const std::string hip_name_;
  drake::trajectories::PiecewisePolynomial<double> foot_traj_;
//  double height_;

  int state_port_;
  int fsm_port_;

  // Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  // Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);
};

}  // namespace dairlib

