#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace dairlib {
namespace multibody {

/// ViewFrame is used to rotate the frame where we view a vector. It's used in,
/// for example, translational task space tracking data (including
/// ComTrackingData, TransTaskSpaceTrackingData,
/// RelativeTranslationTrackingData).
/// One use case: we can set the swing foot tracking gains in the local frame
/// instead of the global frame.
template<typename T>
class ViewFrame {
 public:
  virtual ~ViewFrame() = 0;
  virtual drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T> &plant,
      const drake::systems::Context<T> &context) const = 0;
};

template<typename T>
class WorldYawViewFrame : public ViewFrame<T> {
 public:
  ~WorldYawViewFrame() = default;
  explicit WorldYawViewFrame(const drake::multibody::Body<T> &body)
      : ViewFrame<T>(), body_(body) {}

  drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T> &plant,
      const drake::systems::Context<T> &context) const override;

 protected:
  const drake::multibody::Body<T> &body_;
};

template<typename T>
class RotationFromBodyViewFrame : public ViewFrame<T> {
 public:
  ~RotationFromBodyViewFrame() = default;
  explicit RotationFromBodyViewFrame(
      const drake::multibody::Body<T>& body,
      const Eigen::Matrix3d& R_BF) : ViewFrame<T>(), body_(body), R_BF_(R_BF){}

  drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T> &plant,
      const drake::systems::Context<T> &context) const override;

 protected:
  const drake::multibody::Body<T> &body_;
  const Eigen::Matrix3d R_BF_;
};

template<typename T>
class IdentityViewFrame : public ViewFrame<T> {
 public:
  explicit IdentityViewFrame() : ViewFrame<T>() {}
  ~IdentityViewFrame() = default;

  drake::Matrix3X<T> CalcWorldToFrameRotation(
      const drake::multibody::MultibodyPlant<T> &plant,
      const drake::systems::Context<T> &context) const override;
};
}
}// namespace dairlib::multibody