#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/constraint.h"
#include "drake/math/autodiff_gradient.h"

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
std::unique_ptr<RigidBodyTree<double>> makeFixedBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

/// Builds the rigid body tree for a fixed base Cassie
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
void buildFixedBaseCassieTree(RigidBodyTree<double>& tree,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

std::unique_ptr<RigidBodyTree<double>> makeFloatingBaseCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

void buildFloatingBaseCassieTree(RigidBodyTree<double>& tree,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");


int GetBodyIndexFromName(const RigidBodyTree<double>& tree, 
                         string name);


// The RigidBodyPlant<AutoDiff> that is passed to the constructor needs to be
// created using the RigidBodyTree<double> and not using the scalar conversion
// Contact information is lost the AutoDiffXd scalar conversion is used`
template<typename T>
class CassiePlant {

  public:
    CassiePlant(RigidBodyPlant<T>* plant):plant_(plant),
                                          tree_(plant_->get_rigid_body_tree()) {}

    void CalcTimeDerivativesCassieDuringContact(VectorX<T> x, 
                                                VectorX<T> u, 
                                                VectorX<T> lambda,
                                                ContinuousState<T>* x_dot) const;

    RigidBodyPlant<T>* plant_;
    const RigidBodyTree<double>& tree_;

};


}  // namespace dairlib
