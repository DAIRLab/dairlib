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
template <typename T>
class OscViewFrame {
 public:
  OscViewFrame() {}
  virtual drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T>& plant_w_spr,
      const drake::systems::Context<T>& context_w_spr) const = 0;
};

template <typename T>
class WorldYawOscViewFrame : public OscViewFrame<T> {
 public:
  explicit WorldYawOscViewFrame(const drake::multibody::Body<T>& body)
      : OscViewFrame<T>(), body_(body) {}

  drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T>& plant_w_spr,
      const drake::systems::Context<T>& context_w_spr) const override;

 protected:
  const drake::multibody::Body<T>& body_;
};

template <typename T>
class IdentityViewFrame : public OscViewFrame<T> {
 public:
  explicit IdentityViewFrame() : OscViewFrame<T>() {}

  drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T>& plant_w_spr,
      const drake::systems::Context<T>& context_w_spr) const override;
};
}  // namespace dairlib::systems::controllers