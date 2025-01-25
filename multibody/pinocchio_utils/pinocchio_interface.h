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
  void MapPositionsToPinocchio(drake::VectorX<T>& drake_positions) const;

  template <typename T>
  void MapPositionsToDrake(drake::VectorX<T>& pinocchio_positions) const;

  template <typename T>
  void MapVelocitiesToPinocchio(
      const drake::VectorX<T>& drake_positions,
      drake::VectorX<T>& drake_velocities) const;

  template <typename T>
  void MapVelocitiesToDrake(
      const drake::VectorX<T>& drake_positions,
      drake::VectorX<T>& pinocchio_velocities) const;


  /*!
   * Given a Jacobian, J, representing a task y(q), where ydot = Jv, where
   * v is the generalized velocity in pinochio coordinates, map it to the
   * jacobian such that ydot = Jv , where v is in drake coordinates.
   * @param drake_positions
   * @param pinocchio_jacobian
   */
  template <typename T>
  void MapJacobianToDrake(
      const drake::VectorX<T>& drake_positions,
      drake::MatrixX<T>& pinocchio_jacobian) const;

  pinocchio::Data PinocchioData() {
    return pinocchio::Data(pinocchio_model_);
  }

  const pinocchio::Model& get_model() const {
    return pinocchio_model_;
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