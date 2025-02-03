#pragma once
#include "nonlinear_least_squares_cost.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib::systems::controllers::id_mpc {

class RelativePositionCost : public NonlinearLeastSquaresCost<double> {
 public:
  RelativePositionCost(const Eigen::MatrixXd& Q, const Eigen::Vector3d& yref,
                       const drake::multibody::MultibodyPlant<double>& plant_,
                       const std::string& from_frame,
                       const std::string& to_frame,
                       const Eigen::Vector3d& from_point,
                       const Eigen::Vector3d& to_point,
                       const std::string& description="");


  void EvaluateInnerTerm(const Eigen::Ref<const drake::AutoDiffVecXd> &x,
                         drake::AutoDiffVecXd *y) const override;

  void EvaluateInnerTerm(const Eigen::Ref<const Eigen::VectorXd> &x,
                         Eigen::VectorXd *y) const override;

  void UpdateReference(const Eigen::VectorXd& y) override {
    DRAKE_ASSERT(y.rows() == 3);
    y_ = y;
  }

 private:

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;

  const drake::multibody::Frame<double>& frame_from_;
  const drake::multibody::Frame<double>& frame_to_;
  const Eigen::Vector3d& point_from_;
  const Eigen::Vector3d& point_to_;

  Eigen::Vector3d y_;


};

}