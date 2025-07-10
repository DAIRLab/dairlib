#pragma once

#include "systems/franka_kinematics_vector.h"

namespace dairlib {
namespace systems {

/// TrifingerKinematicsVector stores the robot output as a TimestampedVector
///    * positions
///    * velocities
template <typename T>
class TrifingerKinematicsVector : public FrankaKinematicsVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrifingerKinematicsVector)

  TrifingerKinematicsVector() = default;

  explicit TrifingerKinematicsVector(int num_end_effector_positions,
                                     int num_object_positions,
                                     int num_end_effector_velocities,
                                     int num_object_velocities)
      : FrankaKinematicsVector<T>(
            num_end_effector_positions, num_object_positions,
            num_end_effector_velocities, num_object_velocities) {}

  /// Constructs a OutputVector with the specified positions and velocities.
  explicit TrifingerKinematicsVector(
      const drake::VectorX<T>& end_effector_positions,
      const drake::VectorX<T>& object_positions,
      const drake::VectorX<T>& end_effector_velocities,
      const drake::VectorX<T>& object_velocities)
      : FrankaKinematicsVector<T>(end_effector_positions, object_positions,
                                  end_effector_velocities, object_velocities) {}

  void SetFingertip0Positions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_single_fingertip_positions_);
    this->get_mutable_data().segment(fingertip_0_positions_start_,
                                     num_single_fingertip_positions_) =
        positions;
  }

  void SetFingertip120Positions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_single_fingertip_positions_);
    this->get_mutable_data().segment(fingertip_120_positions_start_,
                                     num_single_fingertip_positions_) =
        positions;
  }

  void SetFingertip240Positions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_single_fingertip_positions_);
    this->get_mutable_data().segment(fingertip_240_positions_start_,
                                     num_single_fingertip_positions_) =
        positions;
  }

  void SetFingertip0Velocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_single_fingertip_velocities_);
    this->get_mutable_data().segment(fingertip_0_velocities_start_,
                                     num_single_fingertip_velocities_) =
        velocities;
  }

  void SetFingertip120Velocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_single_fingertip_velocities_);
    this->get_mutable_data().segment(fingertip_120_velocities_start_,
                                     num_single_fingertip_velocities_) =
        velocities;
  }

  void SetFingertip240Velocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_single_fingertip_velocities_);
    this->get_mutable_data().segment(fingertip_240_velocities_start_,
                                     num_single_fingertip_velocities_) =
        velocities;
  }

  /// Returns a const positions vector for fingertip 0
  const drake::VectorX<T> GetFingertip0Positions() const {
    return this->get_data().segment(fingertip_0_positions_start_,
                                    num_single_fingertip_positions_);
  }

  /// Returns a const positions vector for fingertip 120
  const drake::VectorX<T> GetFingertip120Positions() const {
    return this->get_data().segment(fingertip_120_positions_start_,
                                    num_single_fingertip_positions_);
  }

  /// Returns a const positions vector for fingertip 240
  const drake::VectorX<T> GetFingertip240Positions() const {
    return this->get_data().segment(fingertip_240_positions_start_,
                                    num_single_fingertip_positions_);
  }

  /// Returns a const velocities vector for fingertip 0
  const drake::VectorX<T> GetFingertip0Velocities() const {
    return this->get_data().segment(fingertip_0_velocities_start_,
                                    num_single_fingertip_velocities_);
  }

  /// Returns a const velocities vector for fingertip 120
  const drake::VectorX<T> GetFingertip120Velocities() const {
    return this->get_data().segment(fingertip_120_velocities_start_,
                                    num_single_fingertip_velocities_);
  }

  /// Returns a const velocities vector for fingertip 240
  const drake::VectorX<T> GetFingertip240Velocities() const {
    return this->get_data().segment(fingertip_240_velocities_start_,
                                    num_single_fingertip_velocities_);
  }

  /// Returns a mutable positions vector for fingertip 0
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip0Positions() {
    auto data = this->get_mutable_data().segment(
        fingertip_0_positions_start_, num_single_fingertip_positions_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable positions vector for fingertip 120
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip120Positions() {
    auto data = this->get_mutable_data().segment(
        fingertip_120_positions_start_, num_single_fingertip_positions_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable positions vector for fingertip 240
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip240Positions() {
    auto data = this->get_mutable_data().segment(
        fingertip_240_positions_start_, num_single_fingertip_positions_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector for fingertip 0
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip0Velocities() {
    auto data = this->get_mutable_data().segment(
        fingertip_0_velocities_start_, num_single_fingertip_velocities_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector for fingertip 120
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip120Velocities() {
    auto data = this->get_mutable_data().segment(
        fingertip_120_velocities_start_, num_single_fingertip_velocities_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector for fingertip 240
  Eigen::Map<drake::VectorX<T>> GetMutableFingertip240Velocities() {
    auto data = this->get_mutable_data().segment(
        fingertip_240_velocities_start_, num_single_fingertip_velocities_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

 protected:
  virtual TrifingerKinematicsVector<T>* DoClone() const override {
    return new TrifingerKinematicsVector<T>(
        this->GetEndEffectorPositions().size(),
        this->GetObjectPositions().size(),
        this->GetEndEffectorVelocities().size(),
        this->GetObjectVelocities().size());
  }

 private:
  const int num_single_fingertip_positions_ = 3;
  const int num_single_fingertip_velocities_ = 3;

  const int fingertip_0_positions_start_ = 0;
  const int fingertip_120_positions_start_ = 3;
  const int fingertip_240_positions_start_ = 6;
  const int fingertip_0_velocities_start_ = 16;
  const int fingertip_120_velocities_start_ = 19;
  const int fingertip_240_velocities_start_ = 22;
};

}  // namespace systems
}  // namespace dairlib
