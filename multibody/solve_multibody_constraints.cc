#include "multibody/solve_multibody_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{
namespace multibody{

SolveMultibodyConstraints::SolveMultibodyConstraints(RigidBodyPlant<double>* plant): plant_(plant), tree_(plant->get_rigid_body_tree()), num_positions_(tree_.get_num_positions()), num_velocities_(tree_.get_num_velocities()), num_states_(num_positions_ + num_velocities_), num_efforts_(tree_.get_num_actuators()), num_tree_position_constraints_(tree_.getNumPositionConstraints())
{  

}

VectorXd SolveMultibodyConstraints::solveTP(VectorXd x_init, std::vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    int num_variables = num_states_;
    auto x = prog.NewContinuousVariables(num_variables, "x");
    auto constraint = std::make_shared<TreePositionConstraint>(plant_, num_tree_position_constraints_, num_variables);
    prog.AddConstraint(constraint, x);
    for (uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(x(j) == x_init(j));
    }
    prog.AddQuadraticCost((x - x_init).dot(x - x_init));
    prog.SetInitialGuessForAllVariables(x_init);
    prog.Solve();
    return prog.GetSolution(x);
}


VectorXd SolveMultibodyConstraints::solveFP(VectorXd xu_init, std::vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    int num_variables = num_states_ + num_efforts_;
    auto xu = prog.NewContinuousVariables(num_variables, "xu");
    auto constraint = std::make_shared<FixedPointConstraint>(plant_, num_states_, num_variables);
    prog.AddConstraint(constraint, xu);
    for (uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(xu(j) == xu_init(j));
    }
    prog.AddQuadraticCost((xu - xu_init).dot(xu - xu_init));
    prog.SetInitialGuessForAllVariables(xu_init);
    prog.Solve();
    return prog.GetSolution(xu);
}


VectorXd SolveMultibodyConstraints::solveTPFP(VectorXd xu_init, std::vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    int num_variables = num_states_ + num_efforts_;
    auto xu = prog.NewContinuousVariables(num_variables, "xu");
    auto constraint_tp = std::make_shared<TreePositionConstraint>(plant_, num_tree_position_constraints_, num_variables);
    auto constraint_fp = std::make_shared<FixedPointConstraint>(plant_, num_states_, num_variables);
    prog.AddConstraint(constraint_tp, xu);
    prog.AddConstraint(constraint_fp, xu);
    for (uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(xu(j) == xu_init(j));
    }
    prog.AddQuadraticCost((xu - xu_init).dot(xu - xu_init));
    prog.SetInitialGuessForAllVariables(xu_init);
    prog.Solve();
    return prog.GetSolution(xu);
}


TreePositionConstraint::TreePositionConstraint(RigidBodyPlant<double>* plant, int num_constraints, int num_variables, const std::string& description):
    Constraint(num_constraints,
        num_variables,
        VectorXd::Zero(num_constraints),
        VectorXd::Zero(num_constraints),
        description),
    plant_(plant), tree_(plant->get_rigid_body_tree()) 
{
}


void TreePositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd& y) const 
{

    AutoDiffVecXd y_t;
    Eval(drake::math::initializeAutoDiff(x), y_t);
    y = drake::math::autoDiffToValueMatrix(y_t);

}

void TreePositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd& y) const 
{

    const AutoDiffVecXd q = x.head(tree_.get_num_positions());
    KinematicsCache<AutoDiffXd> cache = tree_.doKinematics(q);
    y = tree_.positionConstraints(cache);

    
}


FixedPointConstraint::FixedPointConstraint(RigidBodyPlant<double>* plant, int num_constraints, int num_variables, const std::string& description):
    Constraint(num_constraints,
        num_variables,
        VectorXd::Zero(num_constraints),
        VectorXd::Zero(num_constraints),
        description),
    plant_(plant), tree_(plant->get_rigid_body_tree()) 
{
}


void FixedPointConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& xu,
                                    Eigen::VectorXd& y) const 
{
    AutoDiffVecXd y_t;
    Eval(drake::math::initializeAutoDiff(xu), y_t);
    y = drake::math::autoDiffToValueMatrix(y_t);
 
}

void FixedPointConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& xu,
                                    AutoDiffVecXd& y) const 
{
    
    const int num_positions = tree_.get_num_positions();
    const int num_velocities = tree_.get_num_velocities();
    const int num_states = num_positions + num_velocities;
    const int num_efforts = tree_.get_num_actuators();

    RigidBodyPlant<AutoDiffXd> plant_autodiff(*plant_);

    auto context_autodiff = plant_autodiff.CreateDefaultContext();
    const AutoDiffVecXd x = xu.head(num_states);
    const AutoDiffVecXd u = xu.tail(num_efforts); 
    
    context_autodiff->set_continuous_state(std::make_unique<ContinuousState<AutoDiffXd>>(BasicVector<AutoDiffXd>(x).Clone(), num_positions, num_velocities, 0));
    context_autodiff->FixInputPort(0, std::make_unique<BasicVector<AutoDiffXd>>(u));
    ContinuousState<AutoDiffXd> cstate_output_autodiff(BasicVector<AutoDiffXd>(x).Clone(), num_positions, num_velocities, 0);
    plant_autodiff.CalcTimeDerivatives(*context_autodiff, &cstate_output_autodiff);

    y = cstate_output_autodiff.CopyToVector();

}

}//namespace multibody
}//namespace drake
