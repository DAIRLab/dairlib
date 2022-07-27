#pragma once
#include "Eigen/Dense"

namespace dairlib::systems::controllers {

 class ConvexFoothold {
  public:
   ConvexFoothold(Eigen::MatrixXd A, Eigen::VectorXd b,
                  Eigen::MatrixXd Aeq, Eigen::VectorXd beq);

   Eigen::VectorXd ProjectPointToSet(const Eigen::VectorXd& p) const;

  private:
   Eigen::MatrixXd A_;
   Eigen::MatrixXd Aeq_;
   Eigen::VectorXd b_;
   Eigen::VectorXd beq_;
 };

}
