#include "pelvis_orientation_traj_generator.h"

#include "multibody/multibody_utils.h"

using dairlib::systems::OutputVector;
using drake::systems::BasicVector;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::Cassie::osc_jump {

PelvisOrientationTrajGenerator::PelvisOrientationTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    const PiecewisePolynomial<double>& orientation_traj,
    std::string traj_name, double time_offset)
    : plant_(plant), traj_(orientation_traj) {
  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;

  this->set_name(traj_name);
  this->DeclareAbstractOutputPort(traj_name, traj_inst,
                                  &PelvisOrientationTrajGenerator::CalcTraj);

  // Shift trajectory by time_offset
  traj_.shiftRight(time_offset);
}

void PelvisOrientationTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* traj) const {
  // Read in current state
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);

  *casted_traj = traj_;
}

}  // namespace dairlib::examples::Cassie::osc_jump
