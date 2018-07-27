#include "cassie_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace dairlib{

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
