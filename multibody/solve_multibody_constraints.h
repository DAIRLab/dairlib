#pragma once

#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/math/autodiff_gradient.h"

using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::autoDiffToValueMatrix;
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

namespace dairlib {
namespace multibody{

VectorXd SolveTreePositionConstraints(const RigidBodyTree<double>& tree, VectorXd x_init, vector<int> fixed_joints = {});

vector<VectorXd> SolveFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, vector<int> fixed_joints = {});

vector<VectorXd> SolveTreePositionAndFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, std::vector<int> fixed_joints = {});

vector<VectorXd> SlveFixedPointFeasibilityConstraints(RigidBodyPlant<double>* plant, VectorXd x0, VectorXd u_init, vector<int> fixed_joints = {});

class TreePositionConstraint : public Constraint
{
    public:
        TreePositionConstraint(const RigidBodyTree<double>& tree,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
    private:
        const RigidBodyTree<double>& tree_;

};

class FixedPointConstraint : public Constraint
{
    public:
        FixedPointConstraint(RigidBodyPlant<double>* plant,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
    private:
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;
        unique_ptr<RigidBodyPlant<AutoDiffXd>> plant_autodiff_;

};


class FixedPointFeasibilityConstraint : public Constraint
{
    public:
        FixedPointConstraint(RigidBodyPlant<double>* plant,
                VectorXd x0,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
    private:
        VectorXd x0_;
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;
        unique_ptr<RigidBodyPlant<AutoDiffXd>> plant_autodiff_;

};


}//namespace multibody
}//namespace dairlib
