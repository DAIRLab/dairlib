#pragma once

// lcmtypes
#include "dairlib/lcmt_mpc_debug.hpp"

// dairlib
#include "geometry/convex_polygon_set.h"

// drake
#include "drake/systems/framework/leaf_system.h"
#include "drake/geometry/meshcat.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib {
namespace perceptive_locomotion {

/*!
 * LeafSystem for visualizing foothold constraints, footstep solutions, and CoM
 * traj solutions for the alip_mpfc controller
 */
class AlipMPFCMeshcatVisualizer : public drake::systems::LeafSystem<double> {
 public:

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AlipMPFCMeshcatVisualizer);

  /*!
   * Constructor.
   * @param meshcat Shared pointer to a meshcat instance. Cannot be NULL
   * @param plant
   */
  AlipMPFCMeshcatVisualizer(
      std::shared_ptr<drake::geometry::Meshcat> meshcat,
      const drake::multibody::MultibodyPlant<double>& plant);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_mpc() const {
    return this->get_input_port(mpc_debug_input_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_terrain() const {
    return this->get_input_port(foothold_input_port_);
  }

 private:
  /*!
   * @brief Returns a meshcat path for the foothold visualizations
   * @param i index of the foothold
   */
  static std::string make_foothold_path(int i) {
    return "/foothold_meshes/" + std::to_string(i);
  }

  void DrawFootholds(geometry::ConvexPolygonSet& footholds,
                     int n_prev_footholds,
                     const std::string& prefix="") const;

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
  drake::systems::InputPortIndex foothold_input_port_;
  drake::systems::DiscreteStateIndex n_footholds_idx_;
  mutable std::shared_ptr<drake::geometry::Meshcat> meshcat_;

};

}
}

