#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/constraint.h"
#include "drake/math/autodiff_gradient.h"

using std::map;
using std::vector;
using std::is_same;
using std::string;

using Eigen::Map;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;

using drake::AutoDiffXd;
using drake::VectorX;
using drake::MatrixX;
using drake::math::AutoDiffMatrixType;
using drake::math::initializeAutoDiff;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::systems::RigidBodyPlant;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;

namespace dairlib {

/// Construct and create a unique pointer to a RigidBodyTree<double>
/// for the fixed base version of Cassie.
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
std::unique_ptr<RigidBodyTree<double>> makeFloatBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

/// Builds the rigid body tree for a fixed base Cassie
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
void buildFloatBaseCassieTree(RigidBodyTree<double>& tree,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

std::unique_ptr<RigidBodyTree<double>> makeFixedBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

void buildFixedBaseCassieTree(RigidBodyTree<double>& tree,
std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

    
/// Solves the position constraints for a position that satisfies them
Eigen::VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         std::vector<int> fixed_joints);


VectorXd ComputeCassieControlInputAnalytical(const RigidBodyTree<double>& tree, VectorXd x);

int GetBodyIndexFromName(const RigidBodyTree<double>& tree, 
                         string name);

VectorXd ComputeCassieJointLimitForces(RigidBodyPlant<double>* plant, 
                                       VectorXd x_init);

bool CassieJointsWithinLimits(const RigidBodyTree<double>& tree, 
                              VectorXd x,
                              double tolerance = 0.0,
bool print_debug_messages = true);

class TreePositionConstraint : public drake::solvers::Constraint {
 public:
  TreePositionConstraint(const RigidBodyTree<double>& tree,
                         const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
    const RigidBodyTree<double>* tree_;
};

template<typename T>
class CassiePlant {

  public:
    CassiePlant(RigidBodyPlant<T>* plant):plant_(plant),
                                          tree_(plant_->get_rigid_body_tree()) {}

    void CalcTimeDerivativesCassie(VectorX<T> x,
                                   VectorX<T> u, 
                                   ContinuousState<T>* xdot) const;

    VectorX<T> CalcTimeDerivativesCassie(VectorX<T> x,
                                         VectorX<T> u) const;

    void CalcTimeDerivativesCassieDuringContact(VectorX<T> x, 
                                                VectorX<T> u, 
                                                VectorX<T> lambda,
                                                ContinuousState<T>* xdot) const;

    RigidBodyPlant<T>* plant_;
    const RigidBodyTree<double>& tree_;

};

}  // namespace dairlib
