#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

// One use case of ExponentialPlusPiecewisePolynomial: if we want to use a guard
// that merge optimal ROM traj with LIPM traj.

class SavedTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  SavedTrajReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                    bool use_exp, bool both_pos_vel_in_traj);

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;
  void CalcSwingFootTraj(const drake::systems::Context<double>& context,
                         drake::trajectories::Trajectory<double>* traj) const;

  int saved_traj_lcm_port_;

  const drake::multibody::MultibodyPlant<double>& plant_control_;
  int nq_;
  int nv_;
  int nx_;
  bool use_exp_;
  bool both_pos_vel_in_traj_;
};

// We have IKTrajReceiver beside SavedTrajReceiver, because it also extracts the
// rows of the trajectory matrix that we want to track.
class IKTrajReceiver : public drake::systems::LeafSystem<double> {
 public:
  IKTrajReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                 const std::vector<std::string>& ordered_pos_names);

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  int saved_traj_lcm_port_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::vector<int> ordered_indices_;

  int nq_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
