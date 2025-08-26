#pragma once

#include <string>
#include <vector>

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
using systems::TimestampedVector;
namespace examples {
namespace plate_balancing {
namespace systems {

/**
 * @brief Stores the robot and object kinematics as a timestamped vector.
 *
 * Contains positions and velocities for both the end effector and the object.
 */
template <typename T>
class FrankaKinematicsVector : public TimestampedVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrankaKinematicsVector)

  FrankaKinematicsVector() = default;

  /**
   * @brief Constructs a vector with specified sizes for each component.
   */
  explicit FrankaKinematicsVector(int num_end_effector_positions,
                                  int num_object_positions,
                                  int num_end_effector_velocities,
                                  int num_object_velocities);

  /**
   * @brief Constructs a vector from given position and velocity vectors.
   */
  explicit FrankaKinematicsVector(
      const drake::VectorX<T>& end_effector_positions,
      const drake::VectorX<T>& object_positions,
      const drake::VectorX<T>& end_effector_velocities,
      const drake::VectorX<T>& object_velocities);

  /// Setters for each component
  void SetEndEffectorPositions(drake::VectorX<T> positions);
  void SetObjectPositions(drake::VectorX<T> positions);
  void SetEndEffectorVelocities(drake::VectorX<T> velocities);
  void SetObjectVelocities(drake::VectorX<T> velocities);
  void SetState(drake::VectorX<T> state);

  /// Getters for each component
  const drake::VectorX<T> GetState() const;
  const drake::VectorX<T> GetEndEffectorPositions() const;
  const drake::VectorX<T> GetObjectPositions() const;
  const drake::VectorX<T> GetEndEffectorVelocities() const;
  const drake::VectorX<T> GetObjectVelocities() const;
  const drake::VectorX<T> GetVelocities() const;
  const drake::VectorX<T> GetPositions() const;

  /// Mutable accessors
  Eigen::Map<drake::VectorX<T>> GetMutablePositions();
  Eigen::Map<drake::VectorX<T>> GetMutableVelocities();
  Eigen::Map<drake::VectorX<T>> GetMutableState();

 protected:
  virtual FrankaKinematicsVector<T>* DoClone() const;

 private:
  const int num_end_effector_positions_;
  const int num_object_positions_;
  const int num_end_effector_velocities_;
  const int num_object_velocities_;
  const int end_effector_positions_start_;
  const int object_positions_start_;
  const int end_effector_velocities_start_;
  const int object_velocities_start_;
  const int num_positions_;
  const int num_velocities_;
};

}  // namespace systems
}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib
