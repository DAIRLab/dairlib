#pragma once

#include <string>
#include <vector>

#include "common/find_resource.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

#include "drake/multibody/rigid_body_tree_construction.h"

namespace dairlib {


/// Add a fixed base cassie to the given multibody plant and scene graph
/// These methods are to be used rather that direct construction of the plant
/// from the URDF to centralize any modeling changes or additions
/// @param plant a pointer to the MultibodyPlant
/// @param scene_graph a pointer to the SceneGraph--may be nullptr (or omitted)
/// @param filename the URDF or SDF file to use for Cassie
///        omit to use default value
void addCassieMultibody(drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph = nullptr,
    bool floating_base = true,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf");

/// Construct and create a unique pointer to a RigidBodyTree<double>
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
std::unique_ptr<RigidBodyTree<double>> makeCassieTreePointer(
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf",
    drake::multibody::joints::FloatingBaseType base_type =
        drake::multibody::joints::kFixed);

/// Builds the rigid body tree for any Cassie base type
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions
void buildCassieTree(
    RigidBodyTree<double>& tree,
    std::string filename = "examples/Cassie/urdf/cassie_v2.urdf",
    drake::multibody::joints::FloatingBaseType base_type =
        drake::multibody::joints::kFixed);

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

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  const RigidBodyTree<double>* tree_;
};

}  // namespace dairlib
