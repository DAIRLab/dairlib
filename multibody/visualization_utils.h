#include "drake/multibody/plant/multibody_plant.h"
#include "drake/common/trajectories/trajectory.h"

namespace dairlib {
namespace multibody {

void visualizeTrajectory(const drake::multibody::MultibodyPlant<double>& plant,
    const drake::trajectories::Trajectory<double>& trajectory,
    double playback_speed = 1);

}  // namespace multibody
}  // namespace dairlib
