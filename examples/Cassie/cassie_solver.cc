#include "cassie_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{


vector<VectorXd> SolveCassieTreeAndFixedPointConstraints(RigidBodyPlant<double>* plant, 
                                                         int num_constraint_forces,
                                                         VectorXd q_init, 
                                                         VectorXd u_init, 
                                                         VectorXd lambda_init,
                                                         vector<int> fixed_joints,
                                                         string snopt_output_filename) {

  MathematicalProgram prog;
  //Setting log file
  
  //prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file", snopt_output_filename);


  auto q = prog.NewContinuousVariables(plant->get_num_positions(), "q");
  auto u = prog.NewContinuousVariables(plant->get_num_actuators(), "u");
  auto lambda = prog.NewContinuousVariables(num_constraint_forces, "lambda");
  auto constraint_tree_position = make_shared<TreeConstraint>(
      plant->get_rigid_body_tree());
  auto constraint_fixed_point = make_shared<CassieFixedPointConstraint>(
      plant, num_constraint_forces);
  prog.AddConstraint(constraint_tree_position, q);
  prog.AddConstraint(constraint_fixed_point, {q, u, lambda});
  
  for(uint i = 0; i < fixed_joints.size(); i++)
  {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }

  // Zero velocity constraints
  //for(uint i = 0; i < v.size(); i++)
  //{
  //  prog.AddConstraint(v(i) == 0.0);
  //}

  //for(uint i = 0; i < lambda.size(); i++) {
  //  prog.AddConstraint(lambda(i) >= 0.01);
  //}


  prog.AddQuadraticCost(
      (q - q_init).dot(q - q_init));
  //prog.AddQuadraticCost(1.0);
  prog.SetInitialGuess(q, q_init);
  //prog.SetInitialGuess(v, v_init);
  prog.SetInitialGuess(u, u_init);
  prog.SetInitialGuess(lambda, lambda_init);

  auto solution = prog.Solve();

  std::cout << "********** Fixed Point Solver Result **********" << std::endl;
  std::cout << to_string(solution) << std::endl;


  VectorXd q_sol = prog.GetSolution(q);
  VectorXd u_sol = prog.GetSolution(u);
  VectorXd lambda_sol = prog.GetSolution(lambda);

  DRAKE_DEMAND(q_sol.size() == q_init.size());
  DRAKE_DEMAND(u_sol.size() == u_init.size());
  DRAKE_DEMAND(lambda_sol.size() == lambda_init.size());

  for(int i=0; i<q_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(q_sol(i)) && !isinf(q_sol(i)));
  }
  for(int i=0; i<u_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(u_sol(i)) && !isinf(u_sol(i)));
  }
  for(int i=0; i<lambda_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(lambda_sol(i)) && !isinf(lambda_sol(i)));
  }


  //Checking if the Tree position constraints are satisfied
  DRAKE_DEMAND(CheckTreeConstraints(plant->get_rigid_body_tree(), q_sol));
  //DRAKE_DEMAND(CheckCassieFixedPointConstraints(plant, q_sol, u_sol, lambda_sol));

  vector<VectorXd> sol;
  sol.push_back(q_sol);
  sol.push_back(u_sol);
  sol.push_back(lambda_sol);

  return sol;
}



bool CheckCassieFixedPointConstraints(RigidBodyPlant<double>* plant,
                                      VectorXd q_check,
                                      VectorXd u_check, 
                                      VectorXd lambda_check) {

  auto constraint = std::make_shared<CassieFixedPointConstraint>(plant, 2);
  VectorXd q_u_l_check(q_check.size() + u_check.size() + lambda_check.size());
  q_u_l_check << q_check, u_check, lambda_check;
  return constraint->CheckSatisfied(q_u_l_check);
}



VectorXd SolveCassieStandingConstraints(const RigidBodyTree<double>& tree, 
                                        VectorXd q_init, 
                                        vector<int> fixed_joints) {
  
  MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  auto constraint_tree = std::make_shared<TreeConstraint>(tree);
  auto constraint_contact = std::make_shared<CassieContactConstraint>(tree);
  
  prog.AddConstraint(constraint_tree, q);
  prog.AddConstraint(constraint_contact, q);

  for(uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }

  prog.AddQuadraticCost(1.0);
  prog.SetInitialGuess(q, q_init);
  auto solution = prog.Solve();
  std::cout << to_string(solution) << std::endl;
  VectorXd q_sol = prog.GetSolution(q);

  return q_sol;

}


CassieFixedPointConstraint::CassieFixedPointConstraint(RigidBodyPlant<double>* plant,
                                                       int num_constraint_forces,
                                                       const std::string& description):
  Constraint(plant->get_num_velocities(),
             plant->get_num_positions() + plant->get_num_actuators() + num_constraint_forces,
             VectorXd::Zero(plant->get_num_velocities()),
             VectorXd::Zero(plant->get_num_velocities()),
             description),
  plant_(plant), tree_(plant->get_rigid_body_tree()),
  num_constraint_forces_(num_constraint_forces) 
{
  plant_autodiff_ = make_unique<RigidBodyPlant<AutoDiffXd>>(*plant);
}


void CassieFixedPointConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
                                        Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(q_u_l), &y_t);
  *y = autoDiffToValueMatrix(y_t);
 
}

void CassieFixedPointConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& q_u_l,
                                        AutoDiffVecXd* y) const {


  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_efforts = tree_.get_num_actuators();


  const AutoDiffVecXd q = q_u_l.head(num_positions);
  const AutoDiffVecXd u = q_u_l.segment(num_positions, num_efforts); 
  const AutoDiffVecXd lambda = q_u_l.tail(num_constraint_forces_);

  *y = CalcMVdot(plant_,
                 q,
                 initializeAutoDiff(VectorXd::Zero(num_velocities)),
                 u,
                 lambda);
  //std::cout << *y << std::endl;
  //std::cout << "------------------------------------" << std::endl;


}

void CassieFixedPointConstraint::DoEval(const Eigen::Ref<const VectorX<Variable>>& x_u, 
                            VectorX<Expression>*y) const {

  throw std::logic_error(
      "FixedPointConstraint does not support symbolic evaluation.");
}



CassieContactConstraint::CassieContactConstraint(const RigidBodyTree<double>& tree,
                                                 const std::string& description):
  Constraint(4,
             tree.get_num_positions(),
             VectorXd::Zero(4),
             VectorXd::Zero(4),
             description),
  tree_(tree) 
{
}


void CassieContactConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                     Eigen::VectorXd* y) const {

  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(x), &y_t);
  *y = autoDiffToValueMatrix(y_t);

}

void CassieContactConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                     AutoDiffVecXd* y) const {

  const AutoDiffVecXd q = x.head(tree_.get_num_positions());
  const VectorXd q_double = DiscardGradient(q);

  KinematicsCache<AutoDiffXd> k_cache = tree_.doKinematics(q);
  KinematicsCache<double> k_cache_double = tree_.doKinematics(q_double);

  // Collision
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;

  // This (const cast) is an ugly way of doing it. Change it later if a better method is available
  const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
      k_cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  const int num_total_contacts = normal_total.cols();
  // 4 contacts for Cassie (2 in each toe)
  const int num_contacts = 4;

  //Getting the indices of the world and toes
  const int world_ind = GetBodyIndexFromName(tree_, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree_, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree_, "toe_right");

  vector<int> contact_ind(num_contacts);
  int k=0;
  for (int i=0; i<num_total_contacts; i++) {
    int ind_a = idxA_total.at(i);
    int ind_b = idxB_total.at(i);
    if ((ind_a == world_ind && ind_b == toe_left_ind) ||
        (ind_a == world_ind && ind_b == toe_right_ind) ||
        (ind_a == toe_left_ind && ind_b == world_ind) ||
        (ind_a == toe_right_ind && ind_b == world_ind)) {

      contact_ind.at(k) = i;
      k++;

    }
  }

  auto y_tmp = initializeAutoDiff(VectorXd::Zero(4));

  // Contact points on body A and B

  auto contact_A_pt_1 = tree_.transformPoints(k_cache,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_2 = tree_.transformPoints(k_cache,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_3 = tree_.transformPoints(k_cache,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_4 = tree_.transformPoints(k_cache,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);

  auto contact_B_pt_1 = tree_.transformPoints(k_cache,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_2 = tree_.transformPoints(k_cache,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_3 = tree_.transformPoints(k_cache,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_4 = tree_.transformPoints(k_cache,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);

  // Computing distance

  y_tmp(0) = (contact_A_pt_1 - contact_B_pt_1).dot(contact_A_pt_1 - contact_B_pt_1);
  y_tmp(1) = (contact_A_pt_2 - contact_B_pt_2).dot(contact_A_pt_2 - contact_B_pt_2);
  y_tmp(2) = (contact_A_pt_3 - contact_B_pt_3).dot(contact_A_pt_3 - contact_B_pt_3);
  y_tmp(3) = (contact_A_pt_4 - contact_B_pt_4).dot(contact_A_pt_4 - contact_B_pt_4);

  std::cout << y_tmp.transpose() << std::endl;

  *y = y_tmp;


}

void CassieContactConstraint::DoEval(const Eigen::Ref<const VectorX<Variable>>& x, 
                                     VectorX<Expression>*y) const {

  throw std::logic_error(
      "TreeConstraint does not support symbolic evaluation.");
}


} // namespace dairlib
