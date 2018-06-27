#pragma once

//#include "dra
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
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

class SolveMultibodyConstraints
{
    public:
        SolveMultibodyConstraints(RigidBodyPlant<double>* plant);
        VectorXd solveTP(VectorXd x_init, std::vector<int> fixed_joints = std::vector<int>());
        VectorXd solveFP(VectorXd xu_init, std::vector<int> fixed_joints = std::vector<int>());
        VectorXd solveTPFP(VectorXd x_init, std::vector<int> fixed_joints = std::vector<int>());
    private:
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;
        const int num_positions_;
        const int num_velocities_;
        const int num_states_;
        const int num_efforts_;
        const int num_tree_position_constraints_;
};

class TreePositionConstraint : public Constraint
{
    public:
        TreePositionConstraint(RigidBodyPlant<double>* plant,
                int num_constraints,
                int num_variables,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
    private:
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;

};

class FixedPointConstraint : public Constraint
{
    public:
        FixedPointConstraint(RigidBodyPlant<double>* plant,
                int num_constraints,
                int num_variables,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
    private:
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;

};


}//namespace multibody
}//namespace dairlib
