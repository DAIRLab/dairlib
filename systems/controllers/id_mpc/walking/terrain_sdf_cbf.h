#pragma once

#include "multibody/box_set.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

class TerrainSDFCBF {
 public:
  TerrainSDFCBF(const drake::multibody::MultibodyPlant<double>& plant,
                multibody::BoxSet* box_set);

  /*
   * return [A, b] such that the constraint A * v1 >= b
   * is equivalent to the relative-degree 2 exponential CBF
   *
   *    J * dv + Jdot * v + (a1 + a2)J * v + (a1a2) h >= 0
   *
   * where h is the cbf, J is the jacobian of h, and dv = (v1 - v) / dt
   */
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetConstraintMatrices(
      const std::string& frame, const Eigen::Vector3d& point,
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      double a1, double a2, double dt) const;

 private:

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  multibody::BoxSet* box_set_;

  Eigen::VectorXd y_;

};

}