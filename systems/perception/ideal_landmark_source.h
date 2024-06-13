#pragma once

#include "dairlib/lcmt_landmark_array.hpp"

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"


namespace dairlib {
namespace perception {

class IdealLandmarkSource : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IdealLandmarkSource);
  IdealLandmarkSource(const drake::multibody::MultibodyPlant<double>& plant,
                      drake::systems::Context<double>* context,
                      const drake::multibody::RigidBodyFrame<double>& landmark_frame);

 private:
  void CalcLandmarks(const drake::systems::Context<double>& context,
                     lcmt_landmark_array* out) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::RigidBodyFrame<double>& landmark_frame_;
  drake::systems::Context<double>* context_;

};

}
}

