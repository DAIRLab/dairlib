#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

struct IKSetting {
  int rom_option;
  int iter;

  int knots_per_mode;

  double feas_tol;
  double opt_tol;

  bool use_ipopt;
  bool log_solver_info;

  // Cost weight

  // Files parameters
  std::string dir_model;  // location of the model files
  std::string dir_data;   // location to store the opt result
  std::string init_file;
};

class RomInverseKinematics : public drake::systems::LeafSystem<double> {
 public:
  RomInverseKinematics(
      const drake::multibody::MultibodyPlant<double>& plant_controls,
      const IKSetting& param, bool debug_mode);

  const drake::systems::InputPort<double>& get_input_port_rom_traj_lcm() const {
    return this->get_input_port(rom_traj_lcm_port_);
  }

 private:
  void CalcIK(const drake::systems::Context<double>& context,
              dairlib::lcmt_saved_traj* q_traj_msg) const;

  int rom_traj_lcm_port_;

  const drake::multibody::MultibodyPlant<double>& plant_control_;
  bool debug_mode_;

  int nq_;
  int nx_;

  std::map<std::string, int> pos_map_;

  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Frame<double>& world_frame_;
  const drake::multibody::Frame<double>& pelvis_frame_;
  const drake::multibody::Frame<double>& toe_left_frame_;
  const drake::multibody::Frame<double>& toe_right_frame_;

  std::unique_ptr<ReducedOrderModel> rom_;
  std::unique_ptr<MirroredReducedOrderModel> mirrored_rom_;
  StateMirror state_mirror_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
