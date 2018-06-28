#include "multibody/solve_multibody_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{
namespace multibody{


vector<VectorXd> SolveTreePositionConstraints(const RigidBodyTree<double>& tree, VectorXd x_init, vector<int> fixed_joints) 
{

    const int num_states = tree.get_num_positions() + tree.get_num_velocities();
    const int num_constraints = tree.getNumPositionConstraints();
    const int num_variables = num_states;

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(num_variables, "x");
    auto constraint = std::make_shared<TreePositionConstraint>(tree, num_constraints, num_variables);
    prog.AddConstraint(constraint, x);
    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(x(j) == x_init(j));
    }
    prog.AddQuadraticCost((x - x_init).dot(x - x_init));
    prog.SetInitialGuessForAllVariables(x_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(x));
    return sol;
}

vector<VectorXd> SolveFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, std::vector<int> fixed_joints) 
{
    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

    const int num_states = tree.get_num_positions() + tree.get_num_velocities();
    const int num_efforts = tree.get_num_actuators();
    const int num_constraints = num_states;
    const int num_variables = num_states + num_efforts;

    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(num_states, "x");
    auto u = prog.NewContinuousVariables(num_efforts, "u");
    auto constraint = std::make_shared<FixedPointConstraint>(plant, num_constraints, num_variables);
    prog.AddConstraint(constraint, {x, u});

    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(x(j) == x_init(j));
    }

    prog.AddQuadraticCost((x - x_init).dot(x - x_init) + (u - u_init).dot(u - u_init));
    prog.SetInitialGuess(x, x_init);
    prog.SetInitialGuess(u, u_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(x));
    sol.push_back(prog.GetSolution(u));
    return sol;
}


vector<VectorXd> SolveTreePositionAndFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, std::vector<int> fixed_joints) 
{

    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

    const int num_states = tree.get_num_positions() + tree.get_num_velocities();
    const int num_efforts = tree.get_num_actuators();
    const int num_constraints_tree_position = tree.getNumPositionConstraints();
    const int num_constraints_fixed_point = num_states;
    const int num_variables_tree_position = num_states;
    const int num_variables_fixed_point = num_states + num_efforts;


    MathematicalProgram prog;
    auto x = prog.NewContinuousVariables(num_states, "x");
    auto u = prog.NewContinuousVariables(num_efforts, "u");
    auto constraint_tree_position = std::make_shared<TreePositionConstraint>(tree, num_constraints_tree_position, num_variables_tree_position);
    auto constraint_fixed_point = std::make_shared<FixedPointConstraint>(plant, num_constraints_fixed_point, num_variables_fixed_point);
    prog.AddConstraint(constraint_tree_position, x);
    prog.AddConstraint(constraint_fixed_point, {x, u});
    
    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(x(j) == x_init(j));
    }

    prog.AddQuadraticCost((x - x_init).dot(x - x_init) + (u - u_init).dot(u - u_init));
    prog.SetInitialGuess(x, x_init);
    prog.SetInitialGuess(u, u_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(x));
    sol.push_back(prog.GetSolution(u));
    return sol;
}


TreePositionConstraint::TreePositionConstraint(const RigidBodyTree<double>& tree, int num_constraints, int num_variables, const std::string& description):
    Constraint(num_constraints,
        num_variables,
        VectorXd::Zero(num_constraints),
        VectorXd::Zero(num_constraints),
        description),
        tree_(tree) 
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
