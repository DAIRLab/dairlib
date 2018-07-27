#pragma once

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

#include "cassie_utils.h"
#include "multibody/solve_multibody_constraints.h"

using std::map;
using std::string;
using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;
using std::isnan;
using std::isinf;

using Eigen::Dynamic;
using Eigen::AutoDiffScalar;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;
using drake::systems::RigidBodyPlant;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::CompliantContactModel;
using drake::solvers::to_string;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::Constraint;
using drake::solvers::VariableRefList;
using drake::solvers::Binding;
using drake::symbolic::Variable;
using drake::symbolic::Expression;

using dairlib::multibody::TreeConstraint;
using dairlib::GetBodyIndexFromName;

namespace dairlib {

VectorXd SolveCassieStandingConstraints(const RigidBodyTree<double>& tree, 
                                        VectorXd q_init, 
                                        vector<int> fixed_joints = {});


class CassieContactConstraint : public Constraint {
  public:
    CassieContactConstraint(const RigidBodyTree<double>& tree,
                            const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                drake::AutoDiffVecXd* y) const override;

    void DoEval(const Eigen::Ref<const VectorX<Variable>>& x, 
                VectorX<Expression>*y) const override;
  
  private:
    const RigidBodyTree<double>& tree_;

};

} // namespace dairlib

