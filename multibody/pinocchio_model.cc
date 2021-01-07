//
// Created by brian on 1/6/21.
//

#include "pinocchio_model.h"
#include "common/find_resource.h"
namespace dairlib{
namespace multibody{


PinocchioModel::PinocchioModel(const drake::multibody::MultibodyPlant<double>& plant,
                               std::string urdf, bool floating_base_) : plant_(plant) {
  pinocchio::urdf::buildModel(dairlib::FindResourceOrThrow(urdf), p_model_);
  p_data_ = pinocchio::Data(p_model_);

  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
  nu_ = plant_.num_actuators();

  DRAKE_ASSERT(nq_ == p_model_.nq);
  DRAKE_ASSERT(nv_ == p_model_.nv);


  this->makeTransformationMatrices();

}

void PinocchioModel::makeTransformationMatrices() {

}
}
}

