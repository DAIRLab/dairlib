#pragma once

#include "solvers/sqp/nonlinear_least_squares_cost.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "multibody/box_set.h"

namespace dairlib::solvers::sqp {

/*!
 *
 * Clamped cost on the SDF from some point to a grid map,
 * || smin(phi(q), phi_des) - phi_des ||^2, where smax is similar to softmax
 * but gives the actual value of the maximum.
 */
class TerrainSDFCost : public NonlinearLeastSquaresCost<double>{

 public:
   TerrainSDFCost(const Eigen::MatrixXd& Q, const Eigen::VectorXd& yref,
                  const drake::multibody::MultibodyPlant<double>& plant,
                  const std::string& frame,
                  const Eigen::Vector3d& point,
                  multibody::BoxSet* box_set,
                  const std::string& description="");

  void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                         drake::AutoDiffVecXd *y) const override;

  void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd> &x,
                         Eigen::VectorXd *y) const override;

  void UpdateReference(const Eigen::VectorXd& y) override {
    DRAKE_ASSERT(y.rows() == 1);
    y_ = y;
  }

  void set_front_point(const Eigen::Vector3d& pt) {
    front_point_ = pt;
  }

  void set_rear_point(const Eigen::Vector3d& pt) {
    rear_point_ = pt;
  }

  void set_frame(const std::string& frame_name) {
    frame_ = &plant_.GetBodyByName(frame_name).body_frame();
  }

 private:

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  const drake::multibody::Frame<double>* frame_;
  Eigen::Vector3d front_point_;
  Eigen::Vector3d rear_point_;

  multibody::BoxSet* box_set_;

  Eigen::VectorXd y_;
};

}