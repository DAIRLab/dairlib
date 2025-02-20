#pragma once

#include "solvers/nonlinear_constraint.h"
#include "systems/controllers/id_mpc/core/constrained_inverse_dynamics_info.h"

namespace dairlib::systems::controllers::id_mpc {

class ALIPMappingConstraint : public solvers::NonlinearConstraint<double> {
 public:

  ALIPMappingConstraint(const ConstrainedDynamicsInfo& dynamics);

  /*!
   * Set the contact point used to calculate the ALIP mapping
   * @param body name of the link to which the contact point is attached
   * @param point position of the contact point in the body frame
   */
  void set_contact_point(const std::string& body, const Eigen::Vector3d& point) {
    DRAKE_DEMAND(body.empty() || plant_.HasBodyNamed(body));
    body_name_ = body;
    point_ = point;
  }

  void EvaluateConstraint(const Eigen::Ref<const Eigen::VectorXd>& x,
                          Eigen::VectorXd* y) const override;

 private:

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::string body_name_{};
  Eigen::Vector3d point_ = Eigen::Vector3d::Zero();

  std::unique_ptr<drake::systems::Context<double>> context_;

};

}