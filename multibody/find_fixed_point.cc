#include "multibody/find_fixed_point.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{

SolveFixedPoint::SolveFixedPoint(RigidBodyPlant<double>* plant): plant_(plant), tree_(plant->get_rigid_body_tree()), num_positions_(tree_.get_num_positions()), num_velocities_(tree_.get_num_velocities()), num_states_(num_positions_ + num_velocities_), num_efforts_(tree_.get_num_actuators()), num_variables_(num_states_ + num_efforts_)
{  

}


VectorXd SolveFixedPoint::solve(VectorXd xu_init, std::vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    auto xu = prog.NewContinuousVariables(num_variables_, "xu");
    auto constraint = std::make_shared<FixedPointConstraint>(plant_, num_states_, num_variables_);
    prog.AddConstraint(constraint, xu);
    for (uint i = 0; i < fixed_joints.size(); i++)
    {
      int j = fixed_joints[i];
      prog.AddConstraint(xu(j) == xu_init(j));
    }
    prog.AddQuadraticCost((xu - xu_init).dot(xu - xu_init));
    prog.SetInitialGuessForAllVariables(xu_init);
    std::cout << "Pre solve" << std::endl;
    prog.Solve();
    std::cout << "Post solve" << std::endl;
    return prog.GetSolution(xu);
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

}//namespace drake
