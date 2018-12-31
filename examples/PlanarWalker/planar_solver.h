#pragma once

#include "drake/common/drake_throw.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

#include "multibody/rbp_utils.h"
#include "multibody/solve_multibody_constraints.h"
#include "planar_utils.h"

using std::map;
using std::string;
using std::vector;
using std::list;
using std::unique_ptr;
using std::make_unique;
using std::make_shared;
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
using dairlib::multibody::CheckTreeConstraints;
using dairlib::multibody::utils::CalcMVdot;
using dairlib::GetBodyIndexFromNamePlanar;

namespace dairlib {


VectorXd SolvePlanarStandingConstraints(const RigidBodyTree<double>& tree, 
                                        VectorXd q_init, 
                                        vector<int> fixed_joints = {},
                                        bool print_debug = false,
                                        string snopt_output_filename = "multibody/log_files/snopt_planar_standing.out");




vector<VectorXd> SolvePlanarFixedPointConstraints(const RigidBodyPlant<double>& plant, 
                                                         int num_constraint_forces,
                                                         VectorXd q_init, 
                                                         VectorXd u_init, 
                                                         VectorXd lambda_init,
                                                         vector<int> fixed_joints = {},
                                                         bool print_debug = false,
                                                         string snopt_output_filename = "multibody/log_files/snopt_planar_fp.out");

bool CheckPlanarFixedPointConstraints(const RigidBodyPlant<double>& plant,
                                      VectorXd x_check,
                                      VectorXd u_check, 
                                      VectorXd lambda_check);

vector<VectorXd> SolvePlanarFixedPointAndStandingConstraints(const RigidBodyPlant<double>& plant, 
                                                                 const RigidBodyPlant<AutoDiffXd>& plant_autodiff,
                                                                 int num_constraint_forces,
                                                                 VectorXd q_init, 
                                                                 VectorXd u_init, 
                                                                 VectorXd lambda_init,
                                                                 vector<int> fixed_joints = {},
                                                                 bool print_debug = false,
                                                                 string snopt_output_filename = "multibody/log_files/snopt_planar_fp_standing.out");



class PlanarStandingConstraint : public Constraint {
  public:
    PlanarStandingConstraint(const RigidBodyTree<double>& tree,
                            const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
                drake::AutoDiffVecXd* y) const override;

    void DoEval(const Eigen::Ref<const VectorX<Variable>>& q, 
                VectorX<Expression>*y) const override;
  
  private:
    const RigidBodyTree<double>& tree_;

};


class PlanarFixedPointConstraint : public Constraint {
  public:
    PlanarFixedPointConstraint(const RigidBodyPlant<double>& plant,
                               int num_constraint_forces,
                               bool print_debug = false,
                               const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& q_u_l,
                AutoDiffVecXd* y) const override;
    void DoEval(const Eigen::Ref<const VectorX<Variable>>& q_u_l, 
                VectorX<Expression>*y) const override;
  
  private:
    const RigidBodyPlant<double>& plant_;
    const RigidBodyTree<double>& tree_;
    int num_constraint_forces_;
    bool print_debug_;

};


class PlanarStandingFixedPointConstraint : public Constraint {
  public:
    PlanarStandingFixedPointConstraint(const RigidBodyPlant<double>& plant,
                                       const RigidBodyPlant<AutoDiffXd>& plant_autodiff,
                                       int num_constraint_forces,
                                       bool print_debug = false,
                                       const std::string& description = "");
    void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
                Eigen::VectorXd* y) const override;
  
    void DoEval(const Eigen::Ref<const AutoDiffVecXd>& q_u_l,
                AutoDiffVecXd* y) const override;
    void DoEval(const Eigen::Ref<const VectorX<Variable>>& q_u_l, 
                VectorX<Expression>*y) const override;
  
  private:
    const RigidBodyPlant<double>& plant_;
    const RigidBodyPlant<AutoDiffXd>& plant_autodiff_;
    const RigidBodyTree<double>& tree_;
    int num_constraint_forces_;
    bool print_debug_;

};




} // namespace dairlib

