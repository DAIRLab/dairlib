#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/Cassie/kinematic_centroidal_planner/cassie_kinematic_centroidal_solver.h"
#include "systems/framework/output_vector.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kcmpc_reference_generator.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/kinematic_centroidal_gains.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class KinematicCentroidalMPC : public drake::systems::LeafSystem<double> {
 public:
  KinematicCentroidalMPC(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::multibody::MultibodyPlant<double>& plant_wo_spr,
      drake::systems::Context<double>* context, KinematicCentroidalGains gains);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_clock() const {
    return this->get_input_port(clock_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_target_vel() const {
    return this->get_input_port(target_vel_port_);
  }
  void SetSpringMaps(Eigen::MatrixXd& pos_map, Eigen::MatrixXd& vel_map) {
    map_position_from_spring_to_no_spring_ = pos_map;
    map_velocity_from_spring_to_no_spring_ = vel_map;
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                dairlib::lcmt_timestamped_saved_traj* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;
  drake::systems::Context<double>* context_wo_spr_;
  const drake::multibody::BodyFrame<double>& world_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex clock_port_;
  drake::systems::InputPortIndex target_vel_port_;

  std::unique_ptr<CassieKinematicCentroidalSolver> solver_;
  std::unique_ptr<KcmpcReferenceGenerator> reference_generator_;

  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  Eigen::VectorXd reference_state_;
  const int n_q_;
  const int n_v_;
  Gait stand_;
  Gait left_step_;
  Gait right_step_;
  Gait jump_;
  Eigen::VectorXd time_points_;
  int n_knot_points_ = 10;
  double ipopt_tol_ = 1e-2;
};

}  // namespace dairlib
