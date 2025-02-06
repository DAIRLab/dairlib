#pragma once
#include "systems/controllers/id_mpc/core/knot_point.h"
#include "solvers/nonlinear_constraint.h"


namespace dairlib::systems::controllers::id_mpc {

/*!
 * Point position constraint representing phi(q) = p
 * where phi(q) is the forward kinematics function for the position of the
 * point of interest and p is a 3D decision variable vector for the position of
 * that point in the world frame
 */
template<typename T>
class PointPositionConstraint : public solvers::NonlinearConstraint<T> {
 public:
  PointPositionConstraint(const ConstrainedDynamicsInfo& dynamics,
                          std::string frame,
                          const Eigen::Vector3d& point_in_frame);

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  void set_point(const std::string& body, const Eigen::Vector3d& point) {
    DRAKE_DEMAND(body.empty() || plant_.HasBodyNamed(body));
    body_name_ = body;
    point_ = point;
  }

 private:

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::string body_name_;
  Eigen::Vector3d point_;

  std::unique_ptr<drake::systems::Context<double>> context_;
};

}