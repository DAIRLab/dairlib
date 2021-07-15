#pragma once

#include "drake/multibody/plant/multibody_plant.h"

#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace dairlib {
namespace multibody {
template <typename T>
class PinocchioPlant : public drake::multibody::MultibodyPlant<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PinocchioPlant)
  
  explicit PinocchioPlant(double time_step, const std::string& urdf) :
      drake::multibody::MultibodyPlant<T>(time_step) {
    pinocchio::urdf::buildModel(urdf, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  }

  void CalcMassMatrix(const drake::systems::Context<T>& context,
                      drake::EigenPtr<drake::MatrixX<T>> M) const;

  private:
   pinocchio::Model pinocchio_model_;
   mutable pinocchio::Data pinocchio_data_;;
};
}  // namespace multibody
}  // namespace dairlib


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
