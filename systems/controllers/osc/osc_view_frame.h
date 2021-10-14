#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace dairlib::systems::controllers {

/// OscViewFrame is used to rotate the frame for translational task space
/// tracking data (including ComTrackingData, TransTaskSpaceTrackingData,
/// RelativeTranslationTrackingData).
/// One use case: we can set the swing foot tracking gains in the local frame
/// instead of the global frame.
class OscViewFrame {
 public:
  OscViewFrame() {}
  virtual Eigen::Matrix3d CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::systems::Context<double>& context_w_spr) const = 0;
};

class WorldYawOscViewFrame : public OscViewFrame {
 public:
  explicit WorldYawOscViewFrame(const drake::multibody::Body<double>& body)
      : OscViewFrame(), body_(body) {}

  Eigen::Matrix3d CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::systems::Context<double>& context_w_spr) const override;

 protected:
  const drake::multibody::Body<double>& body_;
};

class IdentityViewFrame : public OscViewFrame {
 public:
  explicit IdentityViewFrame() : OscViewFrame() {}

  Eigen::Matrix3d CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<double>& plant_w_spr,
      const drake::systems::Context<double>& context_w_spr) const override;
};
}  // namespace dairlib::systems::controllers