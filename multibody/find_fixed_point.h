#pragma once

//#include "dra
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/constraint.h"
#include "drake/math/autodiff_gradient.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::systems::RigidBodyPlant;
using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::ContinuousState;
using drake::systems::CompliantContactModel;
using drake::solvers::Constraint;
using drake::solvers::MathematicalProgram;
using drake::solvers::Constraint;

namespace dairlib {

class SolveFixedPoint
{
    public:
        SolveFixedPoint(RigidBodyPlant<double>* plant, CompliantContactModel<double>* compliant_contact_model);
        VectorXd solve(VectorXd xu_init, std::vector<int> fixed_joints);
    private:
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;
        CompliantContactModel<double>* compliant_contact_model_;
        const int num_positions_;
        const int num_velocities_;
        const int num_states_;
        const int num_efforts_;
        const int num_variables_;
};



class FixedPointConstraint : public Constraint
{
    public:
        FixedPointConstraint(RigidBodyPlant<double>* plant,
                CompliantContactModel<double>* compliant_contact_model,
                int num_constraints,
                int num_variables,
                const std::string& description = "");
        void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override;
  
        void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
                    drake::AutoDiffVecXd& y) const override;
  
        VectorXd solve(VectorXd x_init, std::vector<int> fixed_joints);
  
    private:
        void calcTimeDerivatives(const Context<double>& context, ContinuousState<double>* der, VectorX<double> u) const;
        RigidBodyPlant<double>* plant_;
        const RigidBodyTree<double>& tree_;
        CompliantContactModel<double>* compliant_contact_model_;

};

}
