#include "drake/multibody/plant/multibody_plant.h"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/parsers/urdf.hpp"
namespace dairlib {
namespace multibody{


class PinocchioModel {
public:
  PinocchioModel(const drake::multibody::MultibodyPlant<double>& plant,
                         std::string urdf, bool floating_base);

  Eigen::MatrixXd GetPositionMapDrakeToPinocchio() {
    return qMbpToPin_;
  }

  Eigen::MatrixXd GetPositionMapPinocchioToDrake() {
    return qPinToMbp_;
  }

  Eigen::MatrixXd GetVelocityMapDrakeToPinocchio() {
    return vMbpToPin_;
  }

  Eigen::MatrixXd GetVelocityMapPinocchioToDrake() {
    return vPinToMbp_;
  }

private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  pinocchio::Model p_model_;
  pinocchio::Data p_data_;


  Eigen::MatrixXd qMbpToPin_;
  Eigen::MatrixXd qPinToMbp_;
  Eigen::MatrixXd vMbpToPin_;
  Eigen::MatrixXd vPinToMbp_;

  int nq_;
  int nv_;
  int nu_;

  void makeTransformationMatrices(bool floating_base);
};
}
}
