#include "multibody/solve_multibody_constraints.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{
namespace multibody{

/*
Solves tree position constraints for the given tree.
@param tree RigidBodyTree for which the constraints needs to be solved
@param q_init Initial value of the positions that need to be given to the solver
@params fixed_joints Joints that need to have the exact values as that in q_init
@param return Generalized positions that satisfy the constraints
*/

VectorXd SolveTreePositionConstraints(const RigidBodyTree<double>& tree, VectorXd q_init, vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
    auto constraint = std::make_shared<TreePositionConstraint>(tree);
    prog.AddConstraint(constraint, q);

    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(q(j) == q_init(j));
    }

    prog.AddQuadraticCost((q - q_init).dot(q - q_init));
    prog.SetInitialGuessForAllVariables(q_init);
    prog.Solve();

    return prog.GetSolution(q);
}

bool CheckTreePositionConstraints(const RigidBodyTree<double>& tree, VectorXd q_check)
{
    auto constraint = std::make_shared<TreePositionConstraint>(tree);
    return constraint->CheckSatisfied(q_check);
}

/*
Solves fixed point constraints for the given tree.
@param plant RigidBodyPlant for which the constraints needs to be solved
@param x_init Initial value of the state given to the solver
@param u_init Initial value of the control inputs
@params fixed_joints Joints that need to have the exact values as that in q_init
@param return Vector of q, v, and u solutions as found by the solver
*/

vector<VectorXd> SolveFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, std::vector<int> fixed_joints) 
{

    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
    VectorXd u_zero = VectorXd::Zero(u_init.size());

    MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
    auto v = prog.NewContinuousVariables(tree.get_num_velocities(), "v");
    auto u = prog.NewContinuousVariables(tree.get_num_actuators(), "u");
    auto constraint_fixed_point = std::make_shared<FixedPointConstraint>(plant);
    prog.AddConstraint(constraint_fixed_point, {q, v, u});
    
    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(q(j) == x_init(j));
    }

    VectorXd q_init = x_init.head(tree.get_num_positions());
    VectorXd v_init = x_init.head(tree.get_num_velocities());

    prog.AddQuadraticCost((q - q_init).dot(q - q_init) + (v - v_init).dot(v - v_init) + (u - u_zero).dot(u - u_zero));
    prog.SetInitialGuess(q, q_init);
    prog.SetInitialGuess(v, v_init);
    prog.SetInitialGuess(u, u_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(q));
    sol.push_back(prog.GetSolution(v));
    sol.push_back(prog.GetSolution(u));
    return sol;
}

bool CheckFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_check, VectorXd u_check)
{
    auto constraint = std::make_shared<FixedPointConstraint>(plant);
    VectorXd xu_check(x_check.size() + u_check.size());
    xu_check << x_check, u_check;

    return constraint->CheckSatisfied(xu_check);
}

/*
Solves tree position and fixed point constraints for the given tree.
@param plant RigidBodyPlant for which the constraints needs to be solved
@param x_init Initial value of the state given to the solver
@param u_init Initial value of the control inputs
@params fixed_joints Joints that need to have the exact values as that in q_init
@param return Vector of q, v, and u solutions as found by the solver
*/

vector<VectorXd> SolveTreePositionAndFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_init, VectorXd u_init, std::vector<int> fixed_joints) 
{

    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
    VectorXd u_zero = VectorXd::Zero(u_init.size());

    MathematicalProgram prog;
    auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
    auto v = prog.NewContinuousVariables(tree.get_num_velocities(), "v");
    auto u = prog.NewContinuousVariables(tree.get_num_actuators(), "u");
    auto constraint_tree_position = std::make_shared<TreePositionConstraint>(tree);
    auto constraint_fixed_point = std::make_shared<FixedPointConstraint>(plant);
    prog.AddConstraint(constraint_tree_position, q);
    prog.AddConstraint(constraint_fixed_point, {q, v, u});
    
    for(uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(q(j) == x_init(j));
    }

    VectorXd q_init = x_init.head(tree.get_num_positions());
    VectorXd v_init = x_init.head(tree.get_num_velocities());

    prog.AddQuadraticCost((q - q_init).dot(q - q_init) + (v - v_init).dot(v - v_init) + (u - u_zero).dot(u - u_zero));
    prog.SetInitialGuess(q, q_init);
    prog.SetInitialGuess(v, v_init);
    prog.SetInitialGuess(u, u_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(q));
    sol.push_back(prog.GetSolution(v));
    sol.push_back(prog.GetSolution(u));
    return sol;
}

bool CheckTreePositionAndFixedPointConstraints(RigidBodyPlant<double>* plant, VectorXd x_check, VectorXd u_check)
{
    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
    auto constraint_tree_position = std::make_shared<TreePositionConstraint>(tree);
    auto constraint_fixed_point = std::make_shared<FixedPointConstraint>(plant);
    VectorXd xu_check(x_check.size() + u_check.size());
    VectorXd q_check = x_check.head(tree.get_num_positions());
    xu_check << x_check, u_check;

    return constraint_tree_position->CheckSatisfied(q_check) && constraint_fixed_point->CheckSatisfied(xu_check);
}


vector<VectorXd> SolveFixedPointFeasibilityConstraints(RigidBodyPlant<double>* plant, VectorXd x0, VectorXd u_init) 
{
    const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();

    MathematicalProgram prog;
    auto u = prog.NewContinuousVariables(tree.get_num_actuators(), "u");
    auto constraint = std::make_shared<FixedPointFeasibilityConstraint>(plant, x0);
    prog.AddConstraint(constraint, u);

    prog.AddQuadraticCost(1.0);
    prog.SetInitialGuess(u, u_init);
    prog.Solve();

    vector<VectorXd> sol;
    sol.push_back(prog.GetSolution(u));
    return sol;
}


TreePositionConstraint::TreePositionConstraint(const RigidBodyTree<double>& tree, const std::string& description):
    Constraint(tree.getNumPositionConstraints(),
        tree.get_num_positions(),
        VectorXd::Zero(tree.getNumPositionConstraints()),
        VectorXd::Zero(tree.getNumPositionConstraints()),
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


FixedPointConstraint::FixedPointConstraint(RigidBodyPlant<double>* plant, const std::string& description):
    Constraint((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities(),
        (plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_actuators(),
        VectorXd::Zero((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities()),
        VectorXd::Zero((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities()),
        description),
    plant_(plant), tree_(plant->get_rigid_body_tree()) 
{
    plant_autodiff_ = make_unique<RigidBodyPlant<AutoDiffXd>>(*plant);
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

    auto context_autodiff = plant_autodiff_->CreateDefaultContext();

    const AutoDiffVecXd x = xu.head(num_states);
    const AutoDiffVecXd u = xu.tail(num_efforts); 
    
    context_autodiff->set_continuous_state(std::make_unique<ContinuousState<AutoDiffXd>>(BasicVector<AutoDiffXd>(x).Clone(), num_positions, num_velocities, 0));
    context_autodiff->FixInputPort(0, std::make_unique<BasicVector<AutoDiffXd>>(u));
    ContinuousState<AutoDiffXd> cstate_output_autodiff(BasicVector<AutoDiffXd>(x).Clone(), num_positions, num_velocities, 0);
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff, &cstate_output_autodiff);

    y = cstate_output_autodiff.CopyToVector();


}



FixedPointFeasibilityConstraint::FixedPointFeasibilityConstraint(RigidBodyPlant<double>* plant, VectorXd x0, const std::string& description):
    Constraint((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities(),
        (plant->get_rigid_body_tree()).get_num_actuators(),
        VectorXd::Zero((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities()),
        VectorXd::Zero((plant->get_rigid_body_tree()).get_num_positions() + (plant->get_rigid_body_tree()).get_num_velocities()),
        description),
    plant_(plant), tree_(plant->get_rigid_body_tree()), x0_(x0)
{
    plant_autodiff_ = make_unique<RigidBodyPlant<AutoDiffXd>>(*plant);
}


void FixedPointFeasibilityConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& u,
                                    Eigen::VectorXd& y) const 
{
    AutoDiffVecXd y_t;
    Eval(drake::math::initializeAutoDiff(u), y_t);
    y = drake::math::autoDiffToValueMatrix(y_t);
}

void FixedPointFeasibilityConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& u,
                                    AutoDiffVecXd& y) const 
{
    
    const int num_positions = tree_.get_num_positions();
    const int num_velocities = tree_.get_num_velocities();

    auto context_autodiff = plant_autodiff_->CreateDefaultContext();

    AutoDiffVecXd x0_autodiff = initializeAutoDiff(x0_);

    context_autodiff->set_continuous_state(std::make_unique<ContinuousState<AutoDiffXd>>(BasicVector<AutoDiffXd>(x0_autodiff).Clone(), num_positions, num_velocities, 0));
    context_autodiff->FixInputPort(0, std::make_unique<BasicVector<AutoDiffXd>>(u));
    ContinuousState<AutoDiffXd> cstate_output_autodiff(BasicVector<AutoDiffXd>(x0_autodiff).Clone(), num_positions, num_velocities, 0);
    plant_autodiff_->CalcTimeDerivatives(*context_autodiff, &cstate_output_autodiff);

    y = cstate_output_autodiff.CopyToVector();


}

}//namespace multibody
}//namespace drake
