#include "examples/goldilocks_models/reduced_order_models.h"

namespace dairlib {
namespace goldilocks_models {

// Constructors of RomData
RomData::RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot)
    : RomData(n_y, n_tau, n_feature_y, n_feature_yddot,
              MatrixXd::Zero(n_y, n_tau), VectorXd::Zero(n_y * n_feature_y),
              VectorXd::Zero(n_y * n_feature_yddot)){};

RomData::RomData(int n_y, int n_tau, int n_feature_y, int n_feature_yddot,
                 const MatrixXd& B_tau, const VectorXd& theta_y,
                 const VectorXd& theta_yddot)
    : n_y_(n_y),
      n_yddot_(n_y),
      n_tau_(n_tau),
      n_feature_y_(n_feature_y),
      n_feature_yddot_(n_feature_yddot),
      B_tau_(B_tau),
      theta_y_(theta_y),
      theta_yddot_(theta_yddot) {
  CheckDataConsistency();
};

// Methods of RomData
void RomData::CheckDataConsistency() const {
  DRAKE_DEMAND(B_tau_.rows() == n_y_);
  DRAKE_DEMAND(B_tau_.cols() == n_tau_);
  DRAKE_DEMAND(theta_y_.size() == n_y_ * n_feature_y_);
  DRAKE_DEMAND(theta_yddot_.size() == n_y_ * n_feature_yddot_);
};
void RomData::SetB(const MatrixXd& B_tau) {
  if ((B_tau_.rows() == B_tau.rows()) && (B_tau_.cols() == B_tau.cols())) {
    B_tau_ = B_tau;
  } else {
    B_tau_.resizeLike(B_tau);
    B_tau_ = B_tau;
  }
};
void RomData::SetThetaY(const VectorXd& theta_y) {
  if (theta_y_.size() == theta_y.size()) {
    theta_y_ = theta_y;
  } else {
    theta_y_.resizeLike(theta_y);
    theta_y_ = theta_y;
  }
};
void RomData::SetThetaYddot(const VectorXd& theta_yddot) {
  if (theta_yddot_.size() == theta_yddot.size()) {
    theta_yddot_ = theta_yddot;
  } else {
    theta_yddot_.resizeLike(theta_yddot);
    theta_yddot_ = theta_yddot;
  }
};
void RomData::SetTheta(const VectorXd& theta) {
  DRAKE_DEMAND(theta.size() == theta_y_.size() + theta_yddot_.size());
  theta_y_ = theta.head(theta_y_.size());
  theta_yddot_ = theta.tail(theta_yddot_.size());
};

}  // namespace goldilocks_models
}  // namespace dairlib
