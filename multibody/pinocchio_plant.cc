#include "multibody/pinocchio_plant.h"
#include "multibody/multibody_utils.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace dairlib {
namespace multibody {

using drake::AutoDiffXd;
using drake::VectorX;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::map;
using std::string;

template <typename T>
PinocchioPlant<T>::PinocchioPlant(double time_step, const std::string& urdf) :
      MultibodyPlant<T>(time_step) {
    pinocchio::urdf::buildModel(urdf, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
}

template<typename T>
void PinocchioPlant<T>::Finalize() {
  MultibodyPlant<T>::Finalize();

  BuildPermutations();

  // Check that models match
  int nq = this->num_positions();
  int nv = this->num_velocities();
  int nu = this->num_actuators();

  VectorX<T> x =  VectorX<T>::Random(nq + nv);
  VectorX<T> u =  VectorX<T>::Random(nu);

  auto context = createContext<T>(*this, x, u);
  TestMassMatrix(*context, 1e-6);
}

template<typename T>
void PinocchioPlant<T>::BuildPermutations() {
  map<string, int> pos_map = makeNameToPositionsMap(*this);
  map<string, int> vel_map = makeNameToVelocitiesMap(*this);
  int nq = this->num_positions();
  int nv = this->num_velocities();
  Eigen::VectorXi pos_indices(nq);
  Eigen::VectorXi vel_indices(nv);

    // for (auto const &pair: vel_map) {
    //     std::cout << "{" << pair.first << ": " << pair.second << "}\n";
    // }

  for (int i = 1; i < pinocchio_model_.names.size(); i++) {
    // TODO: floating base options
    // Skipping i=0 for the world (TODO--doesn't handle floating base yet)
    // Assumes that URDF root is welded to the world
    // TODO: new, stripped down benchmark using a single class
    const auto& name = pinocchio_model_.names[i];

    if (pos_map.count(name) == 0) {
      throw std::runtime_error("PinocchioPlant::BuildPermutations: " + name +
                               " was not found in the position map.");
    }

    if (vel_map.count(name + "dot") == 0) {
      throw std::runtime_error("PinocchioPlant::BuildPermutations: " + name +
                               " was not found in the velocity map.");
    }

    int pos_ind = pos_map[name];
    int vel_ind = vel_map[name + "dot"];
    pos_indices(i - 1) = pos_ind;
    vel_indices(i - 1) = vel_ind;
  }

  q_perm_.indices() = pos_indices;
  v_perm_.indices() = vel_indices;
}

template <>
void PinocchioPlant<double>::CalcMassMatrix(const Context<double>& context,
    drake::EigenPtr<Eigen::MatrixXd> M) const {
  VectorXd q = GetPositions(context);
  pinocchio::crba(pinocchio_model_, pinocchio_data_, q_perm_.inverse() * q);

  // Pinocchio builds an upper triangular matrix, skipping the parts
  // below the diagonal. Fill those in here.
  *M = pinocchio_data_.M;
  for (int i = 0; i < M->cols(); i++) {
    for (int j = i + 1; j < M->rows(); j++) {
      (*M)(j, i) = (*M)(i, j);
    }
  }
  *M = v_perm_ * (*M) * v_perm_.inverse();
}

template<>
void PinocchioPlant<AutoDiffXd>::CalcMassMatrix(
    const Context<AutoDiffXd>& context, 
    drake::EigenPtr<drake::MatrixX<AutoDiffXd>> M) const {
  throw std::domain_error("CalcMassMatrix not implemented with AutoDiffXd");
}

template<>
::testing::AssertionResult PinocchioPlant<double>::TestMassMatrix(
    const Context<double>& context, double tol) const {
  int nv = num_velocities();

  MatrixXd M(nv, nv);
  MatrixXd pin_M(nv, nv);

  MultibodyPlant<double>::CalcMassMatrix(context, &M);

  CalcMassMatrix(context, &pin_M);

  return drake::CompareMatrices(M, pin_M, tol);
}

template<>
::testing::AssertionResult PinocchioPlant<AutoDiffXd>::TestMassMatrix(
    const Context<AutoDiffXd>& context, double tol) const {
  throw std::domain_error("TestMassMatrix not implemented with AutoDiffXd");
}

}  // namespace multibody
}  // namespace dairlib


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class dairlib::multibody::PinocchioPlant)
