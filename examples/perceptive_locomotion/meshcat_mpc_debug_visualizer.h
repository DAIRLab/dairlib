#pragma once
#include "dairlib/lcmt_mpc_debug.hpp"
#include "geometry/convex_foothold_set.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {
namespace perceptive_locomotion {

class MeshcatMPCDebugVisualizer : public drake::systems::LeafSystem<double> {
 public:
  MeshcatMPCDebugVisualizer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      const drake::multibody::MultibodyPlant<double>& plant);
  const drake::systems::InputPort<double>& get_input_port_state() {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_mpc() {
    return this->get_input_port(mpc_debug_input_port_);
  }
 private:
  static std::string make_path(int i) {
    return "/foothold_meshes/" + std::to_string(i);
  }


  void DrawFootholds(geometry::ConvexFootholdSet& footholds,
                     int n_prev_footholds) const;

  // Matrix from robot yaw frame to world frame
  static Eigen::Matrix3d R_WB(const Eigen::Vector4d& wxyz);

  void DrawComTrajSolution(
      const std::string& path,
      const dairlib::lcmt_mpc_solution& com_traj_solution,
      const Eigen::Matrix3d& R_yaw,
      const double z_com) const;

  void DrawFootsteps(const dairlib::lcmt_mpc_solution& solution,
                     const Eigen::Matrix3d& R_yaw) const;

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  drake::systems::InputPortIndex state_input_port_;
  drake::systems::InputPortIndex mpc_debug_input_port_;
  drake::systems::DiscreteStateIndex n_footholds_idx_;
  mutable std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};

}
}

