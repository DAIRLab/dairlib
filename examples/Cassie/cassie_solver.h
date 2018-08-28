#pragma once

#include <string>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/constraint.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/snopt_solver.h"
#include "examples/Cassie/cassie_utils.h"

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
//first part: build floating and fixed base Cassie tree

/// Construct and create a unique pointer to a RigidBodyTree<double>
/// for the fixed base version of Cassie.
/// These methods are to be used rather that direct construction of the tree
/// from the URDF to centralize any modeling changes or additions

    
/// Solves the position constraints for a position that satisfies them
Eigen::VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         std::vector<int> fixed_joints);

/// Cassie is (1: standing on the ground  2: satisfy the tree constraint)
Eigen::VectorXd solveCassieStandingConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         std::vector<int> fixed_joints);

// Cassie is (1:fixed, acceleration is zero 2: standing on the ground  3: satisfy the tree constraint)
Eigen::VectorXd solveCassieStandingFixedConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         Eigen::VectorXd u_init,
                                         Eigen::VectorXd lambda_init,
                                         int num_constraint_forces,
                                         std::vector<int> fixed_joints);


// second part, different constraint
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

class CassieStandingConstraint : public drake::solvers::Constraint{
    public:
    CassieStandingConstraint(const RigidBodyTree<double>& tree,
                          const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
               Eigen::VectorXd* y) const override;

     void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;

      void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

    private:
    const RigidBodyTree<double>& tree_;
};

class CassieFeetDistanceConstraint : public drake::solvers::Constraint{
    public:
    CassieFeetDistanceConstraint(const RigidBodyTree<double>& tree,
                          const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
               Eigen::VectorXd* y) const override;

     void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;

      void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

    private:
    const RigidBodyTree<double>& tree_;
};

class CassieFixedConstraint : public drake::solvers::Constraint{
    public:
    CassieFixedConstraint(const RigidBodyTree<double>& tree,
                          int num_forces,
                          const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
               Eigen::VectorXd* y) const override;

     void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;

      void DoEval(const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
              drake::VectorX<drake::symbolic::Expression>* y) const override;

    private:
    const RigidBodyTree<double>& tree_;
    const int num_forces_;
};

}  // namespace dairlib
