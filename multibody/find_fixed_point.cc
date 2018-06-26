#include "multibody/find_fixed_point.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{

SolveFixedPoint::SolveFixedPoint(RigidBodyPlant<double>* plant, CompliantContactModel<double>* compliant_contact_model): plant_(plant), tree_(plant->get_rigid_body_tree()), compliant_contact_model_(compliant_contact_model), num_positions_(tree_.get_num_positions()), num_velocities_(tree_.get_num_velocities()), num_states_(num_positions_ + num_velocities_), num_efforts_(tree_.get_num_actuators()), num_variables_(num_states_ + num_efforts_)
{  

}


VectorXd SolveFixedPoint::solve(VectorXd xu_init, std::vector<int> fixed_joints) 
{

    MathematicalProgram prog;
    auto xu = prog.NewContinuousVariables(num_variables_, "xu");
    auto constraint = std::make_shared<FixedPointConstraint>(plant_, compliant_contact_model_, num_states_, num_variables_, "");
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


FixedPointConstraint::FixedPointConstraint(RigidBodyPlant<double>* plant, CompliantContactModel<double>* compliant_contact_model, int num_constraints, int num_variables, const std::string& description):
    Constraint(num_constraints,
        num_variables,
        VectorXd::Zero(num_constraints),
        VectorXd::Zero(num_constraints),
        description),
    RigidBodyPlant<double>(std::make_unique<RigidBodyTree<double>>()),
    plant_(plant), tree_(plant->get_rigid_body_tree()), 
    compliant_contact_model_(compliant_contact_model)
{
}

void FixedPointConstraint::calcTimeDerivatives(const Context<double>& context, ContinuousState<double>* der, VectorX<double> u) const
{

    auto x = dynamic_cast<const BasicVector<double>&>(context.get_continuous_state_vector()).get_value();

    const int nq = tree_.get_num_positions();
    const int nv = tree_.get_num_velocities();
    const int num_actuators = tree_.get_num_actuators();

    VectorX<double> q = x.topRows(nq);
    VectorX<double> v = x.bottomRows(nv);

    auto kinsol = tree_.doKinematics(q, v);

    const MatrixX<double> M = tree_.massMatrix(kinsol);

    const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;

    VectorX<double> right_hand_side =
        -tree_.dynamicsBiasTerm(kinsol, no_external_wrenches);

    if (num_actuators > 0) right_hand_side += tree_.B * u;
    {
      for (auto const& b : tree_.get_bodies()) {
        if (!b->has_parent_body()) continue;
        auto const& joint = b->getJoint();
        if (joint.get_num_positions() == 1 && joint.get_num_velocities() == 1) {
          const double limit_force =
              JointLimitForce(joint, q(b->get_position_start_index()),
                              v(b->get_velocity_start_index()));
          right_hand_side(b->get_velocity_start_index()) += limit_force;
        }
      }
    }

    right_hand_side += compliant_contact_model_->ComputeContactForce(tree_, kinsol);

    VectorX<double> vdot;
    if (tree_.getNumPositionConstraints()) {

      const double alpha = 5.0;
      auto phi = tree_.positionConstraints(kinsol);
      auto J = tree_.positionConstraintsJacobian(kinsol, false);
      auto Jdotv = tree_.positionConstraintsJacDotTimesV(kinsol);

      MatrixX<double> A(M.rows() + J.rows(),
                   M.cols() + J.rows());
      VectorX<double> b(M.rows() + J.rows());
      A << M, -J.transpose(),
           J, MatrixX<double>::Zero(J.rows(), J.rows());
      b << right_hand_side,
           -(Jdotv + 2 * alpha * J * v + alpha * alpha * phi);
      const VectorX<double> vdot_f =
          A.completeOrthogonalDecomposition().solve(b);
      vdot = vdot_f.head(tree_.get_num_velocities());
    } else {
      vdot = M.llt().solve(right_hand_side);
    }

    VectorX<double> xdot(tree_.get_num_positions() + tree_.get_num_velocities());
    xdot << tree_.transformVelocityToQDot(kinsol, v), vdot;
    der->SetFromVector(xdot);

}



void FixedPointConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& xu,
                                    Eigen::VectorXd& y) const 
{
    const int num_positions = tree_.get_num_positions();
    const int num_velocities = tree_.get_num_velocities();
    const int num_states = num_positions + num_velocities;
    const int num_efforts = tree_.get_num_actuators();

    auto context = plant_->CreateDefaultContext();
    VectorXd x = xu.head(num_states);
    VectorXd u = xu.tail(num_efforts);

    context->set_continuous_state(std::make_unique<ContinuousState<double>>(BasicVector<double>(x).Clone(), num_positions, num_velocities, 0));
    context->FixInputPort(0, std::make_unique<BasicVector<double>>(u));
   
    ContinuousState<double>* cstate_output;
    calcTimeDerivatives(*context, cstate_output, u);

    y = cstate_output->CopyToVector();

}

}//namespace drake
