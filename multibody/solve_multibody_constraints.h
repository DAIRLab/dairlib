#pragma once

#include "drake/common/drake_throw.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/math/autodiff_gradient.h"

using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;
using std::isnan;
using std::isinf;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;
using drake::systems::RigidBodyPlant;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::CompliantContactModel;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::Constraint;
using drake::solvers::VariableRefList;
using drake::solvers::Binding;
using drake::symbolic::Variable;
using drake::symbolic::Expression;

namespace dairlib {
namespace multibody{


/*
Solves tree position constraints for the given tree.
@param tree RigidBodyTree for which the constraints needs to be solved
@param q_init Initial value of the positions that need to be given to the solver
@params fixed_joints Joints that need to have the exact values as that in q_init
@return std::vector of 'q' that satisfy the constraints
*/
VectorXd SolveTreeConstraints(const RigidBodyTree<double>& tree, 
                              VectorXd q_init,
                              vector<int> fixed_joints = {});

/*
Checks if the given position satisfies constraints of the tree
@param tree RigidBodyTree for which the constraints need to be checked
@param q_check Position at which the constraints need to be checked
@return Boolean value that corresponds to whether the constraints are satisfied or not
 */
bool CheckTreeConstraints(const RigidBodyTree<double>& tree,
                          VectorXd q_check);

/*
Solves fixed point constraints for the given tree.
@param plant RigidBodyPlant for which the constraints needs to be solved
@param x_init Initial value of the state given to the solver
@param u_init Initial value of the control inputs
@params fixed_joints Joints that need to have the exact values as that in q_init
@return std::vector of q, v, and u solutions as found by the solver
*/
vector<VectorXd> SolveFixedPointConstraints(RigidBodyPlant<double>* plant,
                                            VectorXd x_init,
                                            VectorXd u_init,
                                            vector<int> fixed_joints = {});

/*
Checks if the given state and inputs satisfies fixed point constraints
@param plant RigidBodyPlant input
@param x_check State at which the fixed point constraints need to be checked
@param u_check Inputs at which the fixed point constraints need to be checked
@return Boolean value that corresponds to whether the fixed point constraints are satisfied or not
 */

bool CheckFixedPointConstraints(RigidBodyPlant<double>* plant,
                                VectorXd x_check,
                                VectorXd u_check);


vector<VectorXd> SolveFixedPointConstraintsApproximate(RigidBodyPlant<double>* plant, 
                                                       VectorXd x_init, 
                                                       VectorXd u_init);



/*
Solves tree and fixed point constraints for the given tree.
@param plant RigidBodyPlant for which the constraints needs to be solved
@param x_init Initial value of the state given to the solver
@param u_init Initial value of the control inputs
@params fixed_joints Joints that need to have the exact values as that in q_init
@return std::vector of q, v, and u solutions as found by the solver
*/
vector<VectorXd> SolveTreeAndFixedPointConstraints(RigidBodyPlant<double>* plant,
                                                   VectorXd x_init,
                                                   VectorXd u_init,
                                                   std::vector<int> fixed_joints = {});

/*
Checks if the given state and inputs satisfies tree and fixed point constraints
@param plant RigidBodyPlant input
@param x_check State at which the fixed point constraints need to be checked
@param u_check Inputs at which the fixed point constraints need to be checked
@return Boolean value that corresponds to whether the tree and fixed point constraints are satisfied or not
 */
bool CheckTreeAndFixedPointConstraints(RigidBodyPlant<double>* plant,
                                       VectorXd x_check,
                                       VectorXd u_check);


/*
Finds a feasible control input for the given state such that the pair satisfies tree and fixed point constraints
@param plant RigidBodyPlant input
@param x0 Given input state that must satisfy tree constraints
@param u_init Initial value of the control inputs given to the solver
@return std::vector of solution 'u'
 */
vector<VectorXd> SolveFixedPointFeasibilityConstraints(RigidBodyPlant<double>* plant,
                                                       VectorXd x0,
                                                       VectorXd u_init);



class TreeConstraint : public Constraint {
  public:
    TreeConstraint(const RigidBodyTree<double>& tree,
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

class FixedPointConstraint : public Constraint {
  public:
    FixedPointConstraint(RigidBodyPlant<double>* plant,
                         const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x_u,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x_u,
                AutoDiffVecXd* y) const override;
    void DoEval(const Eigen::Ref<const VectorX<Variable>>& x_u, 
                VectorX<Expression>*y) const override;
  
  private:
    RigidBodyPlant<double>* plant_;
    const RigidBodyTree<double>& tree_;
    unique_ptr<RigidBodyPlant<AutoDiffXd>> plant_autodiff_;

};

class FixedPointConstraintApproximate : public Constraint {
  public:
    FixedPointConstraintApproximate(RigidBodyPlant<double>* plant,
                                    VectorXd x0, 
                                    const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x_dot_u,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x_dot_u,
                AutoDiffVecXd* y) const override;
    void DoEval(const Eigen::Ref<const VectorX<Variable>>& x_dot_u, 
                VectorX<Expression>*y) const override;
    VectorXd CalcXDot(VectorXd u);
  
  private:
    RigidBodyPlant<double>* plant_;
    const RigidBodyTree<double>& tree_;
    VectorXd x0_;
    unique_ptr<RigidBodyPlant<AutoDiffXd>> plant_autodiff_;

};


class FixedPointFeasibilityConstraint : public Constraint {
  public:
    FixedPointFeasibilityConstraint(RigidBodyPlant<double>* plant,
                                    VectorXd x0,
                                    const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& u,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& u,
                AutoDiffVecXd* y) const override;

    void DoEval(const Eigen::Ref<const VectorX<Variable>>& u, 
                VectorX<Expression>*y) const override;
  
  private:
    RigidBodyPlant<double>* plant_;
    const RigidBodyTree<double>& tree_;
    VectorXd x0_;
    unique_ptr<RigidBodyPlant<AutoDiffXd>> plant_autodiff_;

};


}//namespace multibody
}//namespace dairlib
