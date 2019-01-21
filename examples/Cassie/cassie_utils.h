#pragma once

#include <string>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/math/autodiff_gradient.h"

#include "drake/multibody/rigid_body_tree_construction.h"

namespace dairlib {


/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param filename the URDF or SDF file to use for Cassie 
///        omit to use default value
void addFixedBaseCassieMultibody(
    drake::multibody::multibody_plant::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

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

/// Solves the position constraints for a position that satisfies them
Eigen::VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         std::vector<int> fixed_joints);


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

}  // namespace dairlib
