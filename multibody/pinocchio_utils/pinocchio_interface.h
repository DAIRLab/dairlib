#pragma once

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "drake/systems/framework/context.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::multibody {

/*!
 * wrapper for using pinocchio and drake together more gracefully than full
 * inheritance
 */
class PinocchioInterface {
 public:
  explicit PinocchioInterface(
      const drake::multibody::MultibodyPlant<double>& plant,
      const std::string& urdf);

  template <typename T>
  drake::VectorX<T> MapPositionsToPinocchio(
      const drake::VectorX<T>& drake_positions) const;

  template <typename T>
  drake::VectorX<T> MapPositionsToDrake(
      const drake::VectorX<T>& pinocchio_positions) const;

  template <typename T>
  drake::VectorX<T> MapVelocitiesToPinocchio(
      const drake::VectorX<T>& drake_positions,
      const drake::VectorX<T>& drake_velocities) const;

  template <typename T>
  drake::VectorX<T> MapVelocitiesToDrake(
      const drake::VectorX<T>& drake_positions,
      const drake::VectorX<T>& pinocchio_velocities) const;

  template <typename T>
  drake::VectorX<T> MapVDotToPinocchio(
      const drake::VectorX<T>& drake_positions,
      const drake::VectorX<T>& drake_velocities,
      const drake::VectorX<T>& drake_vdot) const;


  /*!
   * Given a Jacobian, J, representing a task y(q), where ydot = Jv, where
   * v is the generalized velocity in pinocchio coordinates, map it to the
   * jacobian such that ydot = Jv , where v is in drake coordinates.
   * @param drake_positions
   * @param pinocchio_jacobian
   */
  template <typename T, int rows>
  void MapJvToDrake(
      const drake::VectorX<T>& drake_positions,
      Eigen::Matrix<T, rows, Eigen::Dynamic>* Jv_pin) const;

  template <typename T, int rows>
  void MapJvToDrake(
      const drake::VectorX<T>& drake_positions,
      drake::EigenPtr<Eigen::Matrix<T, rows, Eigen::Dynamic>> Jv_pin) const;

  template <typename T, int rows>
  void MapJqToDrake(Eigen::Matrix<T, rows, Eigen::Dynamic>* Jq_pin) const;

  pinocchio::Data PinocchioData() {
    return pinocchio::Data(pinocchio_model_);
  }

  const pinocchio::Model& get_model() const {
    return pinocchio_model_;
  }

  const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& q_perm()
  const {
    return q_perm_p2d_;
  }
  const Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>& v_perm()
  const {
    return v_perm_p2d_;
  }

 private:

  void BuildPermutations();
  void CopyReflectedInertiaToPinocchioModel();

  bool is_floating_base_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> q_perm_p2d_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> v_perm_p2d_;

  pinocchio::Model pinocchio_model_;
};

}