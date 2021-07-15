#include "multibody/pinocchio_plant.h"

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using Eigen::VectorXd;

template <>
void PinocchioPlant<double>::CalcMassMatrix(
    const drake::systems::Context<double>& context,
    drake::EigenPtr<Eigen::MatrixXd> M) const {
  VectorXd q = GetPositions(context);
  // probably not the right num. joints for fixed/floating
  // definitely not the right ordering  
  pinocchio::crba(pinocchio_model_, pinocchio_data_, q);
  *M = pinocchio_data_.M;
}

template<>
void PinocchioPlant<AutoDiffXd>::CalcMassMatrix(
    const drake::systems::Context<AutoDiffXd>& context, 
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> M) const {
  
}

}  // namespace multibody
}  // namespace dairlib
