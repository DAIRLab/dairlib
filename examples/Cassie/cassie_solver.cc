#include "cassie_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{


vector<VectorXd> SolveCassieTreeAndFixedPointConstraints(RigidBodyPlant<double>* plant, 
                                                         VectorXd x_init, 
                                                         VectorXd u_init, 
                                                         vector<int> fixed_joints) {

  MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(plant->get_num_positions(), "q");
  auto v = prog.NewContinuousVariables(plant->get_num_velocities(), "v");
  auto u = prog.NewContinuousVariables(plant->get_num_actuators(), "u");
  auto constraint_tree_position = make_shared<TreeConstraint>(
      plant->get_rigid_body_tree());
  auto constraint_fixed_point = make_shared<CassieFixedPointConstraint>(plant);
  prog.AddConstraint(constraint_tree_position, q);
  prog.AddConstraint(constraint_fixed_point, {q, v, u});
  
  for(uint i = 0; i < fixed_joints.size(); i++)
  {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == x_init(j));
  }

  VectorXd q_init = x_init.head(plant->get_num_positions());
  VectorXd v_init = x_init.tail(plant->get_num_velocities());

  prog.AddQuadraticCost(
      (q - q_init).dot(q - q_init) + (v - v_init).dot(v - v_init));
  prog.SetInitialGuess(q, q_init);
  prog.SetInitialGuess(v, v_init);
  prog.SetInitialGuess(u, u_init);
  auto solution = prog.Solve();

  std::cout << "Solver Result: " << std::endl;
  std::cout << to_string(solution) << std::endl;

  VectorXd q_sol = prog.GetSolution(q);
  VectorXd v_sol = prog.GetSolution(v);
  VectorXd u_sol = prog.GetSolution(u);
  VectorXd x_sol(q_sol.size() + v_sol.size());
  x_sol << q_sol, v_sol;

  DRAKE_DEMAND(q_sol.size() == q_init.size());
  DRAKE_DEMAND(v_sol.size() == v_init.size());
  DRAKE_DEMAND(u_sol.size() == u_init.size());

  for(int i=0; i<q_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(q_sol(i)) && !isinf(q_sol(i)));
  }
  for(int i=0; i<v_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(v_sol(i)) && !isinf(v_sol(i)));
  }
  for(int i=0; i<u_sol.size(); i++)
  {
    DRAKE_DEMAND(!isnan(u_sol(i)) && !isinf(u_sol(i)));
  }
  //DRAKE_DEMAND(CheckTreeAndFixedPointConstraints(plant, x_sol, u_sol));

  vector<VectorXd> sol;
  sol.push_back(q_sol);
  sol.push_back(v_sol);
  sol.push_back(u_sol);
  return sol;
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
                                                       const std::string& description):
  Constraint(plant->get_num_states(),
             plant->get_num_states() + plant->get_num_actuators(),
             VectorXd::Zero(plant->get_num_states()),
             VectorXd::Zero(plant->get_num_states()),
             description),
  plant_(plant), tree_(plant->get_rigid_body_tree()) 
{
    plant_autodiff_ = make_unique<RigidBodyPlant<AutoDiffXd>>(*plant);
}


void CassieFixedPointConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x_u,
                                        Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(initializeAutoDiff(x_u), &y_t);
  *y = autoDiffToValueMatrix(y_t);
 
}

void CassieFixedPointConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x_u,
                                        AutoDiffVecXd* y) const {
    
  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_efforts = tree_.get_num_actuators();

  auto context_autodiff = plant_autodiff_->CreateDefaultContext();

  const AutoDiffVecXd x = x_u.head(num_states);
  const AutoDiffVecXd u = x_u.tail(num_efforts); 

  context_autodiff->get_mutable_continuous_state().SetFromVector(x);
  
  context_autodiff->FixInputPort(0, std::make_unique<BasicVector<AutoDiffXd>>(u));
  ContinuousState<AutoDiffXd> cstate_output_autodiff(
      BasicVector<AutoDiffXd>(x).Clone(), num_positions, num_velocities, 0);

  // Using CalcTimeDerivatives from Cassie Plant
  RigidBodyPlant<AutoDiffXd> plant_autodiff(*plant_);
  CassiePlant<AutoDiffXd> cassie_plant(&plant_autodiff);
  cassie_plant.CalcTimeDerivativesCassie(
      x, u, &cstate_output_autodiff);

  *y = cstate_output_autodiff.CopyToVector();


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
