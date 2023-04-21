#pragma once
#include "dairlib/lcmt_gps_signal.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::simulation {

/// system to mimic a GPS receiver rigidly attached to the robot
class GpsReceiver : public drake::systems::LeafSystem<double> {
 public:
  GpsReceiver(const drake::multibody::MultibodyPlant<double>& plant,
                    drake::systems::Context<double>* context,
                    const drake::multibody::Body<double>& body,
                    const Eigen::Vector3d& p_GB);

 private:

  void CalcReceiverPosition(const drake::systems::Context<double>& context,
                            lcmt_gps_signal* gps) const;

  const Eigen::Vector3d& p_GB_;
  const drake::multibody::Body<double>& receiver_body_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
};
}