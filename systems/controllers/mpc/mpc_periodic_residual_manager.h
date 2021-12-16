#pragma once
#include "drake/common/eigen_types.h"


namespace dairlib::systems {

typedef struct residual_dynamics {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd b;
} residual_dynamics;

class MpcPeriodicResidualManager {
 public:
  MpcPeriodicResidualManager(int nknots, const Eigen::MatrixXd& Aref,
                     const Eigen::MatrixXd& Bref, const Eigen::VectorXd& bref);

  residual_dynamics GetResidualForKnotFromCurrent(int i);

  void AddResidualToDynamics(const residual_dynamics& res1,
                             const residual_dynamics& res2,
                             drake::EigenPtr<Eigen::MatrixXd> A,
                             drake::EigenPtr<Eigen::MatrixXd> B,
                             drake::EigenPtr<Eigen::MatrixXd> b);

  void SetResidualForCurrentKnot(const residual_dynamics& dyn);
  void CycleCurrentKnot() {idx_ = (idx_ + 1) % n_;}

 private:
  residual_dynamics GetResidualForKnot(int i) {return buf_.at(i);}
  int idx_ = 0;
  int n_;
  std::vector<residual_dynamics> buf_;

};
}