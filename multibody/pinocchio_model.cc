//
// Created by brian on 1/6/21.
//
#include "pinocchio_model.h"

#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/common/drake_assert.h"
#include "common/find_resource.h"

namespace dairlib{
namespace multibody{


PinocchioModel::PinocchioModel(const drake::multibody::MultibodyPlant<double>& plant,
                               std::string urdf, bool floating_base) : plant_(plant) {
  pinocchio::urdf::buildModel(dairlib::FindResourceOrThrow(urdf), p_model_);
  p_data_ = pinocchio::Data(p_model_);

  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
  nu_ = plant_.num_actuators();

  DRAKE_ASSERT(nq_ == p_model_.nq);
  DRAKE_ASSERT(nv_ == p_model_.nv);

  qPinToMbp_ = Eigen::MatrixXd::Zero(nq_, nq_);
  vPinToMbp_ = Eigen::MatrixXd::Zero(nv_, nv_);
  qMbpToPin_ = Eigen::MatrixXd::Zero(nq_, nq_);
  vMbpToPin_ = Eigen::MatrixXd::Zero(nv_, nv_);

  this->makeTransformationMatrices(floating_base);

}

void PinocchioModel::makeTransformationMatrices(bool floating_base) {
  std::vector<drake::multibody::JointIndex> plant_joints;
  for (pinocchio::JointIndex k(1); k < p_model_.njoints; k++) {
    auto name = p_model_.names[k];
    plant_joints.push_back(plant_.GetJointByName(name).index());
    std::cout << name << std::endl;
  }
  Eigen::MatrixXd select = plant_.MakeStateSelectorMatrix(plant_joints);

  qMbpToPin_.block(0, 0, 4, 4) =
      Eigen::MatrixXd::Identity(4, 4);
  qMbpToPin_.block(4, 4, nq_ - 4, nq_ - 4) =
      select.block(0, 7, nq_ - 4, nq_ - 4);

  vMbpToPin_.block(0, 0, 3, 3) =
      Eigen::MatrixXd::Identity(3, 3);


  std::cout << "Constructed Matrices" << std::endl;
}


}
}

