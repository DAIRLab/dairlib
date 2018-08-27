#include "examples/Cassie/cassie_solver.h"
#include "common/find_resource.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/joints/revolute_joint.h"


namespace dairlib {
using Eigen::VectorXd;
using Eigen::Vector3d;
using drake::solvers::Constraint;
using drake::AutoDiffVecXd;
using drake::solvers::MathematicalProgram;
using namespace std;

// function used in this file

// check if Cassie's feet are on the ground
bool checkCassieOnGround(const RigidBodyTree<double>& tree,VectorXd q_sol);

bool checkCassieFixed(const RigidBodyTree<double>& tree, VectorXd q_sol, VectorXd u_sol, VectorXd lambda_sol);

// calculate the Mvdot based on the system states, inputs and contact forces
template<typename T>
VectorX<T> CalcMVdotCassieStanding(const RigidBodyTree<double>& tree, VectorX<T> x, VectorX<T> u, VectorX<T> lambda);

// calculate the contact jacobian for Cassie
template<typename T>
MatrixX<T> CalCassieContactJacobian(const RigidBodyTree<double>& tree,VectorX<T> q, VectorX<T> v,int num_contact_constraints);

VectorXd solvePositionConstraints(const RigidBodyTree<double>& tree,
                                  VectorXd q_init,
                                  std::vector<int> fixed_joints) {
 
  MathematicalProgram prog;
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  Eigen::MatrixXd lb = tree.joint_limit_min;
  Eigen::MatrixXd ub = tree.joint_limit_max;
  //Eigen::VectorXd lb = -Eigen::VectorXd::Ones(tree.get_num_positions());
  // Eigen::VectorXd ub = Eigen::VectorXd::Ones(tree.get_num_positions());
  // what is this constrant
  auto constraint = std::make_shared<TreePositionConstraint>(tree);
  prog.AddConstraint(constraint, q);
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) <= ub(i));
  }
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) >= lb(i));
  }
  prog.AddQuadraticCost((q - q_init).dot(q - q_init));
  prog.SetInitialGuessForAllVariables(q_init);
  prog.Solve();
  VectorXd result = prog.GetSolution(q);
  for(int i=0;i<tree.get_num_positions();i++){
    if(result(i)>ub(i)||result(i)<lb(i)){
      cout << "*************** solution are out of the limits! *********************" << endl;
      getchar();
    }
  }
  return prog.GetSolution(q);
}

Eigen::VectorXd solveCassieStandingConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         std::vector<int> fixed_joints) {

  MathematicalProgram prog;
  // set snopt options
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", 1.0e-10);
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor feasibility tolerance", 1.0e-10);

  //
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  Eigen::MatrixXd lb = tree.joint_limit_min;
  Eigen::MatrixXd ub = tree.joint_limit_max;
  auto tree_constraint = std::make_shared<TreePositionConstraint>(tree);
  auto standing_constraint = std::make_shared<CassieStandingConstraint>(tree);
  prog.AddConstraint(tree_constraint, q);
  prog.AddConstraint(standing_constraint, q);

  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) <= ub(i));
  }
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) >= lb(i));
  }
  prog.AddQuadraticCost((q - q_init).dot(q - q_init));
  prog.SetInitialGuessForAllVariables(q_init);
  prog.Solve();
  VectorXd result = prog.GetSolution(q);

  if(!checkCassieOnGround(tree,result)){
    cout << "************* cassie is not standing on the ground *************" << endl;
  }
  for(int i=0;i<tree.get_num_positions();i++){
    if(result(i)>ub(i)||result(i)<lb(i)){
      cout << "*************** solution are out of the limits! *********************" << endl;
      getchar();
    }
  }

  return prog.GetSolution(q);
}

Eigen::VectorXd solveCassieStandingFixedConstraints(const RigidBodyTree<double>& tree,
                                         Eigen::VectorXd q_init,
                                         Eigen::VectorXd u_init,
                                         Eigen::VectorXd lambda_init,
                                         int num_constraint_forces,
                                         std::vector<int> fixed_joints){

  MathematicalProgram prog;
  // set snopt options
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", 1.0e-7);
  prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor feasibility tolerance", 1.0e-7);
  const int num_tree_constraints = 2;
  const int num_contacts = 4;
  //
  auto q = prog.NewContinuousVariables(tree.get_num_positions(), "q");
  auto u = prog.NewContinuousVariables(tree.get_num_actuators(), "u");
  auto lambda = prog.NewContinuousVariables(num_constraint_forces, "lambda");

  Eigen::MatrixXd lb = tree.joint_limit_min;
  Eigen::MatrixXd ub = tree.joint_limit_max;
  auto tree_constraint = std::make_shared<TreePositionConstraint>(tree);
  auto standing_constraint = std::make_shared<CassieStandingConstraint>(tree);
  auto feet_distance_constraint = std::make_shared<CassieFeetDistanceConstraint>(tree);
  auto fixed_constraint = std::make_shared<CassieFixedConstraint>(tree,num_constraint_forces);

  prog.AddConstraint(tree_constraint, q);
  prog.AddConstraint(standing_constraint, q);
  //prog.AddConstraint(feet_distance_constraint,q);
  prog.AddConstraint(fixed_constraint, {q, u, lambda});

  // add fixed state constraint
  for (uint i = 0; i < fixed_joints.size(); i++) {
    int j = fixed_joints[i];
    prog.AddConstraint(q(j) == q_init(j));
  }

  // add bounded constraints
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) <= ub(i));
  }
  for (int i = 0;i<tree.get_num_positions();i++){
        prog.AddConstraint(q(i) >= lb(i));
  }

  // other constraint on state
  // prog.AddConstraint(q(2)>= 0.5);
  // add Friction constraints
  const double mu = 0.7;
  for (int i = 0; i < num_contacts; i++) {

    //prog.AddConstraint(lambda(i*3 + num_tree_constraints) >= 0);
    
    // with tangent force
    
    prog.AddConstraint(lambda(i*3 + 1 + num_tree_constraints) <= mu*lambda(i*3 + num_tree_constraints));
    prog.AddConstraint(-lambda(i*3 + 1 + num_tree_constraints) <= mu*lambda(i*3 + num_tree_constraints));
    prog.AddConstraint(lambda(i*3 + 2 + num_tree_constraints) <= mu*lambda(i*3 + num_tree_constraints));
    prog.AddConstraint(-lambda(i*3 + 2 + num_tree_constraints) <= mu*lambda(i*3 + num_tree_constraints));
  
  
    // without tangent force
    prog.AddConstraint(lambda(i*3 + 1 + num_tree_constraints) == 0);
    prog.AddConstraint(lambda(i*3 + 2 + num_tree_constraints) == 0);
  }

  // contact force should balance
  //prog.AddConstraint(lambda(num_tree_constraints) + lambda(3 + num_tree_constraints) == lambda(6+num_tree_constraints) + lambda(9 + num_tree_constraints));
  //prog.AddConstraint(lambda(num_tree_constraints) + lambda(6 + num_tree_constraints) == lambda(3+num_tree_constraints) + lambda(9 + num_tree_constraints));
  //prog.AddQuadraticCost(0.1*(lambda(2) + lambda(8) - lambda(5) - lambda(11))*(lambda(2) + lambda(8) - lambda(5) - lambda(11)));
  //prog.AddQuadraticCost(0.1*(lambda(2) + lambda(5) - lambda(8) - lambda(11))*(lambda(2) + lambda(5) - lambda(8) - lambda(11)));

  // TODO: Cassie will fall down, what is the
  VectorXd u_zero = VectorXd::Zero(tree.get_num_actuators());
  prog.AddQuadraticCost(
      (q - q_init).dot(q - q_init) + (u - u_init).dot(u - u_init));
  prog.AddQuadraticCost(lambda(2)*lambda(2) + lambda(5)*lambda(5) + lambda(8)*lambda(8) + lambda(11)*lambda(11));


  prog.SetInitialGuess(q, q_init);
  prog.SetInitialGuess(u, u_init);
  prog.SetInitialGuess(lambda, lambda_init);

  auto solution = prog.Solve();

  std::cout << "********** Tree Fixed Point and Standing Solver Result **********" << std::endl;
  std::cout << to_string(solution) << std::endl;


  VectorXd q_sol = prog.GetSolution(q);
  VectorXd u_sol = prog.GetSolution(u);
  VectorXd lambda_sol = prog.GetSolution(lambda);



  // check if Cassie is standing on the ground
  if(!checkCassieOnGround(tree,q_sol)){
    cout << "************* cassie is not standing on the ground *************" << endl;
    getchar();
  }

  // check if Cassie Joint limit constraint is satisfied
  for(int i=0;i<tree.get_num_positions();i++){
    if(q_sol(i)>ub(i)||q_sol(i)<lb(i)){
      cout << "*************** solution are out of the limits! *********************" << endl;
      getchar();
    }
  }
  
  // check if Mvdot is zero
  if(!checkCassieFixed(tree, q_sol, u_sol,  lambda_sol)){
      cout << "*************** solution are not in a fixed pose! *********************" << endl;
      getchar();
  }

  cout << "***************** solution found *******************" << endl;
  cout << "**************** contact force from tree constraint ****************" << endl;
  cout << lambda_sol.head(num_tree_constraints) << endl;
  cout << "**************** contact force from ground constraint ****************" << endl;
  cout << lambda_sol.tail(12) << endl; 
  cout << "**************** state solver ****************" << endl;
  for(int i=0; i<tree.get_num_positions(); i++){
    cout << tree.getPositionName(i) << ":" << q_sol(i) << endl;
  }

  VectorXd sol = VectorXd::Zero(tree.get_num_positions() + tree.get_num_actuators() + lambda_sol.rows());
  sol << q_sol,u_sol,lambda_sol;
  return sol;
}

TreePositionConstraint::TreePositionConstraint(
    const RigidBodyTree<double>& tree, const std::string& description) :
    Constraint(tree.getNumPositionConstraints(),
               tree.get_num_positions(),
               VectorXd::Zero(tree.getNumPositionConstraints()),
               VectorXd::Zero(tree.getNumPositionConstraints()),
               description) {
  tree_ = &tree;
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(x), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void TreePositionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                    AutoDiffVecXd* y) const {
  const AutoDiffVecXd q = x.head(tree_->get_num_positions());
  KinematicsCache<drake::AutoDiffXd> cache = tree_->doKinematics(q);
  *y = tree_->positionConstraints(cache);
}

void TreePositionConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "TreePositionConstraint does not support symbolic evaluation.");
}

CassieStandingConstraint::CassieStandingConstraint(
    const RigidBodyTree<double>& tree, const std::string& description) :
    Constraint(4+2,
               tree.get_num_positions(),
               VectorXd::Zero(4+2),
               VectorXd::Zero(4+2),
               description),
               tree_(tree) {
}

void CassieStandingConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(q), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void CassieStandingConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& q,
                                    AutoDiffVecXd* y) const {
  // first get double and autodiff version of variable
  const AutoDiffVecXd q_autodiff = q.head(tree_.get_num_positions());
  const VectorXd q_double = DiscardGradient(q);
  // forward kinematics
  KinematicsCache<double> cache_double = tree_.doKinematics(q_double);
  KinematicsCache<AutoDiffXd> cache_autodiff = tree_.doKinematics(q_autodiff);
  // collision detect
  VectorXd phi_total; //distance from object to environment
  Matrix3Xd normal_total, xA_total, xB_total; // what does xA and xB represent
  vector<int> idxA_total, idxB_total; //
  const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
       cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  const int num_total_contacts = normal_total.cols();
  const int num_contacts = 4;
  const int world_ind = tree_.FindBodyIndex("world");
  const int toe_left_ind = tree_.FindBodyIndex("toe_left");
  const int toe_right_ind = tree_.FindBodyIndex("toe_right");
  vector<int> contact_ind(num_contacts);
  int k=0;
  for (int i=0; i<num_total_contacts; i++) {
      int ind_a = idxA_total.at(i);
      //cout << "a is " << tree.getBodyOrFrameName(ind_a) << endl;
      int ind_b = idxB_total.at(i);
      //cout << "b is " << tree.getBodyOrFrameName(ind_b) << endl;
      if ((ind_a == world_ind && ind_b == toe_left_ind) ||
          (ind_a == world_ind && ind_b == toe_right_ind) ||
          (ind_a == toe_left_ind && ind_b == world_ind) ||
          (ind_a == toe_right_ind && ind_b == world_ind)) {
              contact_ind.at(k) = i;
              k++;
      }
  }

  auto y_temp = initializeAutoDiff(VectorXd::Zero(6));
  // calculate the distance constraint
  auto contact_A_pt_1 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_2 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(1)), 
                                              idxA_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_A_pt_3 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(2)), 
                                              idxA_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_A_pt_4 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(3)), 
                                              idxA_total.at(contact_ind.at(3)), 
                                              world_ind);

  auto contact_B_pt_1 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_2 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(1)), 
                                              idxB_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_B_pt_3 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(2)), 
                                              idxB_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_B_pt_4 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(3)), 
                                              idxB_total.at(contact_ind.at(3)), 
                                              world_ind);

  // distance to the ground should be zero
  y_temp(0) = (contact_A_pt_1-contact_B_pt_1).dot(contact_A_pt_1-contact_B_pt_1);
  y_temp(1) = (contact_A_pt_2-contact_B_pt_2).dot(contact_A_pt_2-contact_B_pt_2);
  y_temp(2) = (contact_A_pt_3-contact_B_pt_3).dot(contact_A_pt_3-contact_B_pt_3);
  y_temp(3) = (contact_A_pt_4-contact_B_pt_4).dot(contact_A_pt_4-contact_B_pt_4);
  
  // feet should be flat on the ground
  y_temp(4) = contact_B_pt_1(2) - contact_B_pt_2(2);
  y_temp(5) = contact_B_pt_3(2) - contact_B_pt_4(2);

  *y = y_temp;
}

void CassieStandingConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "CassieStandingConstraint does not support symbolic evaluation.");
}

CassieFeetDistanceConstraint::CassieFeetDistanceConstraint(
    const RigidBodyTree<double>& tree, const std::string& description) :
    Constraint(4,
               tree.get_num_positions(),
               0.01*VectorXd::Ones(4),
               0.5*VectorXd::Ones(4),
               description),
               tree_(tree) {
}

void CassieFeetDistanceConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(q), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void CassieFeetDistanceConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& q,
                                    AutoDiffVecXd* y) const {
  // first get double and autodiff version of variable
  const AutoDiffVecXd q_autodiff = q.head(tree_.get_num_positions());
  const VectorXd q_double = DiscardGradient(q);
  // forward kinematics
  KinematicsCache<double> cache_double = tree_.doKinematics(q_double);
  KinematicsCache<AutoDiffXd> cache_autodiff = tree_.doKinematics(q_autodiff);
  // collision detect
  VectorXd phi_total; //distance from object to environment
  Matrix3Xd normal_total, xA_total, xB_total; // what does xA and xB represent
  vector<int> idxA_total, idxB_total; //
  const_cast<RigidBodyTree<double>&>(tree_).collisionDetect(
       cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  const int num_total_contacts = normal_total.cols();
  const int num_contacts = 4;
  const int world_ind = tree_.FindBodyIndex("world");
  const int toe_left_ind = tree_.FindBodyIndex("toe_left");
  const int toe_right_ind = tree_.FindBodyIndex("toe_right");
  vector<int> contact_ind(num_contacts);
  int k=0;
  for (int i=0; i<num_total_contacts; i++) {
      int ind_a = idxA_total.at(i);
      //cout << "a is " << tree.getBodyOrFrameName(ind_a) << endl;
      int ind_b = idxB_total.at(i);
      //cout << "b is " << tree.getBodyOrFrameName(ind_b) << endl;
      if ((ind_a == world_ind && ind_b == toe_left_ind) ||
          (ind_a == world_ind && ind_b == toe_right_ind) ||
          (ind_a == toe_left_ind && ind_b == world_ind) ||
          (ind_a == toe_right_ind && ind_b == world_ind)) {
      }
      else{
          contact_ind.at(k) = i;
          k++;
      }
  }

  auto y_temp = initializeAutoDiff(VectorXd::Zero(4));
  // calculate the distance constraint
  auto contact_A_pt_1 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_2 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(1)), 
                                              idxA_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_A_pt_3 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(2)), 
                                              idxA_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_A_pt_4 = tree_.transformPoints(cache_autodiff,
                                              xA_total.col(contact_ind.at(3)), 
                                              idxA_total.at(contact_ind.at(3)), 
                                              world_ind);

  auto contact_B_pt_1 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_2 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(1)), 
                                              idxB_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_B_pt_3 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(2)), 
                                              idxB_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_B_pt_4 = tree_.transformPoints(cache_autodiff,
                                              xB_total.col(contact_ind.at(3)), 
                                              idxB_total.at(contact_ind.at(3)), 
                                              world_ind);

  //add distance constraint
  y_temp(0) = (contact_A_pt_1-contact_B_pt_1).dot(contact_A_pt_1-contact_B_pt_1);
  y_temp(1) = (contact_A_pt_2-contact_B_pt_2).dot(contact_A_pt_2-contact_B_pt_2);
  y_temp(2) = (contact_A_pt_3-contact_B_pt_3).dot(contact_A_pt_3-contact_B_pt_3);
  y_temp(3) = (contact_A_pt_4-contact_B_pt_4).dot(contact_A_pt_4-contact_B_pt_4);
  //

  *y = y_temp;
}

void CassieFeetDistanceConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "CassieFeetDistanceConstraint does not support symbolic evaluation.");
}

CassieFixedConstraint::CassieFixedConstraint(
    const RigidBodyTree<double>& tree , int num_forces , const std::string& description ) :
    Constraint(tree.get_num_velocities(),
               tree.get_num_positions() + tree.get_num_actuators() + num_forces,
               VectorXd::Zero(tree.get_num_velocities()),
               VectorXd::Zero(tree.get_num_velocities()),
               description),
              tree_(tree),num_forces_(num_forces) {
}

void CassieFixedConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
                                    Eigen::VectorXd* y) const {
  AutoDiffVecXd y_t;
  Eval(drake::math::initializeAutoDiff(q_u_l), &y_t);
  *y = drake::math::autoDiffToValueMatrix(y_t);
}

void CassieFixedConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& q_u_l,
                                    AutoDiffVecXd* y) const {

  const int num_positions = tree_.get_num_positions();
  const int num_velocities = tree_.get_num_velocities();
  const int num_efforts = tree_.get_num_actuators();
  const AutoDiffVecXd q = q_u_l.head(num_positions);
  const AutoDiffVecXd v = VectorXd::Zero(num_velocities).template cast<AutoDiffXd>();
  AutoDiffVecXd x(num_positions + num_velocities);
  x << q, v;
  const AutoDiffVecXd u = q_u_l.segment(num_positions, num_efforts); 
  const AutoDiffVecXd lambda = q_u_l.tail(num_forces_);
  
  *y = CalcMVdotCassieStanding(tree_, x, u, lambda);
}

void CassieFixedConstraint::DoEval(
    const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
    drake::VectorX<drake::symbolic::Expression>* y) const {
  throw std::logic_error(
      "TreePositionConstraint does not support symbolic evaluation.");
}

bool checkCassieOnGround(const RigidBodyTree<double>& tree,VectorXd q_sol){
  KinematicsCache<double> cache = tree.doKinematics(q_sol);
  VectorXd phi_total; //distance from object to environment
  Matrix3Xd normal_total, xA_total, xB_total; // what does xA and xB represent
  vector<int> idxA_total, idxB_total; //
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
       cache, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);
  const int num_total_contacts = normal_total.cols();
  const int num_contacts = 4;
  const int world_ind = tree.FindBodyIndex("world");
  const int toe_left_ind = tree.FindBodyIndex("toe_left");
  const int toe_right_ind = tree.FindBodyIndex("toe_right");
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

  

  VectorXd y_temp = VectorXd::Zero(4);
  // calculate the distance constraint
  auto contact_A_pt_1 = tree.transformPoints(cache,
                                              xA_total.col(contact_ind.at(0)), 
                                              idxA_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_A_pt_2 = tree.transformPoints(cache,
                                              xA_total.col(contact_ind.at(1)), 
                                              idxA_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_A_pt_3 = tree.transformPoints(cache,
                                              xA_total.col(contact_ind.at(2)), 
                                              idxA_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_A_pt_4 = tree.transformPoints(cache,
                                              xA_total.col(contact_ind.at(3)), 
                                              idxA_total.at(contact_ind.at(3)), 
                                              world_ind);

  auto contact_B_pt_1 = tree.transformPoints(cache,
                                              xB_total.col(contact_ind.at(0)), 
                                              idxB_total.at(contact_ind.at(0)), 
                                              world_ind);
  auto contact_B_pt_2 = tree.transformPoints(cache,
                                              xB_total.col(contact_ind.at(1)), 
                                              idxB_total.at(contact_ind.at(1)), 
                                              world_ind);
  auto contact_B_pt_3 = tree.transformPoints(cache,
                                              xB_total.col(contact_ind.at(2)), 
                                              idxB_total.at(contact_ind.at(2)), 
                                              world_ind);
  auto contact_B_pt_4 = tree.transformPoints(cache,
                                              xB_total.col(contact_ind.at(3)), 
                                              idxB_total.at(contact_ind.at(3)), 
                                              world_ind);

  y_temp(0) = (contact_A_pt_1-contact_B_pt_1).dot(contact_A_pt_1-contact_B_pt_1);
  y_temp(1) = (contact_A_pt_2-contact_B_pt_2).dot(contact_A_pt_2-contact_B_pt_2);
  y_temp(2) = (contact_A_pt_3-contact_B_pt_3).dot(contact_A_pt_3-contact_B_pt_3);
  y_temp(3) = (contact_A_pt_4-contact_B_pt_4).dot(contact_A_pt_4-contact_B_pt_4);
  
  cout << "**********************distance to the ground ******************" << endl;
  cout << "********************from collisiondetection function**********" << endl;
  cout << phi_total << endl;
  cout << "***************** from distance calculation between two point *****" << endl;
  for(int i=0;i<num_contacts;i++){
    cout << y_temp(i) << endl;
    if(abs(y_temp(i))>1e-4){
      return false;
    }
  }
  return true;
}

bool checkCassieFixed(const RigidBodyTree<double>& tree, VectorXd q_sol, VectorXd u_sol, VectorXd lambda_sol){
  // check if -C + Bu + Jlambda is close to zero
  const int num_positions = tree.get_num_positions();
  const int num_velocities = tree.get_num_velocities();
  const int num_inputs = tree.get_num_actuators();
  const int num_tree_contact = 2;

  const int num_constraints_per_contact = 3;
  const int num_contacts = 4;
  const int num_contact_constraints = num_constraints_per_contact * num_contacts;
  
  VectorXd lambda_tree = lambda_sol.head(num_tree_contact);
  VectorXd lambda_ground = lambda_sol.tail(num_contact_constraints);

  VectorXd v = VectorXd::Zero(num_velocities);
  // do forward kinematics
  KinematicsCache<double> cache = tree.doKinematics(q_sol , v);

  // calculate Mvdot
  const typename RigidBodyTree<double>::BodyToWrenchMap no_external_wrenches;
  VectorXd Mvdot = VectorXd::Zero(num_velocities);
  Mvdot = tree.B*u_sol - tree.dynamicsBiasTerm(cache, no_external_wrenches);

  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;

  // This (const cast) is an ugly way of doing it. Change it later if a better method is available
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
      cache, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  const int num_total_contacts = normal_total.cols();

  //  Getting the indices of the world and toes
  const int world_ind = GetBodyIndexFromName(tree, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree, "toe_right");

  // Obtaining the indices of valid collisions
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

  Matrix3Xd normal = Matrix3Xd::Zero(normal_total.rows(), num_contacts);
  for (int i = 0; i < num_contacts; i++) {
    normal.col(i) = normal_total.col(contact_ind.at(i));
  }
  
  const Map<Matrix3Xd> normal_map(
      normal.data(), normal_total.rows(), num_contacts);

  vector<Map<Matrix3Xd>> tangents;

  Matrix3Xd tmp_mat1 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map1(tmp_mat1.data(), 3, 4);
  Matrix3Xd tmp_mat2 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map2(tmp_mat2.data(), 3, 4);
  tangents.push_back(tmp_map1);
  tangents.push_back(tmp_map2);

  tree.surfaceTangents(normal_map, tangents);


  //Computing the position Jacobian
  vector<MatrixXd> Jd(num_contacts);

  for (int i=0; i<num_contacts; i++) {
    auto tmp_JA = tree.transformPointsJacobian(cache,
                                                xA_total.col(contact_ind.at(i)), 
                                                idxA_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    auto tmp_JB = tree.transformPointsJacobian(cache,
                                                xB_total.col(contact_ind.at(i)), 
                                                idxB_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    Jd.at(i) = tmp_JA - tmp_JB;
  }


  //Computing the 3 jacobians for each contact point
  MatrixXd J(num_contact_constraints, tree.get_num_positions());

  for (int i=0; i<num_contacts; i++) {

    MatrixXd J_pt(num_contact_constraints, tree.get_num_positions());

    auto normal_pt = normal.col(contact_ind.at(i));
    auto tangent1_pt = tangents.at(0).col(contact_ind.at(i));
    auto tangent2_pt = tangents.at(1).col(contact_ind.at(i));

    J_pt.row(0) = normal_pt.transpose()*Jd.at(i);
    J_pt.row(1) = tangent1_pt.transpose()*Jd.at(i);
    J_pt.row(2) = tangent2_pt.transpose()*Jd.at(i);

    J.block(i*num_constraints_per_contact, 0, num_constraints_per_contact, tree.get_num_positions()) =  J_pt;
  }
  auto J_tree = tree.positionConstraintsJacobian(cache);
  Mvdot += J_tree.transpose()*lambda_tree + J.transpose()*lambda_ground;
  cout << "*************** Mvdot ***************" << endl;
  for(int i=0; i<num_velocities ; i++){
    cout << Mvdot(i) << endl;
    if(abs(Mvdot(i))>1e-4){
      return false;
    }
  }
  return true;
}

template<typename T>
VectorX<T> CalcMVdotCassieStanding(const RigidBodyTree<double>& tree, VectorX<T> x, VectorX<T> u, VectorX<T> lambda){
  // basic information
  const int num_positions = tree.get_num_positions();
  const int num_velocities = tree.get_num_velocities();
  const int num_efforts = tree.get_num_actuators();
  const int num_tree_constraints = tree.getNumPositionConstraints();
  const int num_total_constraints = lambda.size();
  const int num_contact_constraints = num_total_constraints - num_tree_constraints;

  VectorX<T> q = x.head(num_positions);
  VectorX<T> v = x.tail(num_velocities);
  VectorX<T> lambda_tree = lambda.head(num_tree_constraints);
  VectorX<T> lambda_ground = lambda.tail(num_contact_constraints);

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  KinematicsCache<T> k_cache = tree.doKinematics(q, v);

  const MatrixX<T> M = tree.massMatrix(k_cache);
  const typename RigidBodyTree<T>::BodyToWrenchMap no_external_wrenches;

  VectorX<T> right_hand_side = 
    -tree.dynamicsBiasTerm(k_cache, no_external_wrenches);

  if (num_efforts > 0) {
    right_hand_side += tree.B * u;
  }

  // contact force from rod
  if (num_tree_constraints) {
    auto J_tree = tree.positionConstraintsJacobian(k_cache);
    right_hand_side += J_tree.transpose()*lambda_tree;

  }
  // contact force from ground 
  MatrixX<T> J_contact = CalCassieContactJacobian(tree, q , v , num_contact_constraints);
  right_hand_side += J_contact.transpose()*lambda_ground;

  VectorX<T> vdot = M.completeOrthogonalDecomposition().solve(right_hand_side);

  return right_hand_side;
}

template<typename T>
MatrixX<T> CalCassieContactJacobian(const RigidBodyTree<double>& tree, VectorX<T> q, VectorX<T> v,int num_contact_constraints){
  const int num_contacts = 4;
  const int num_constraints_per_contact = num_contact_constraints/num_contacts;

  //Computing double versions
  VectorX<double> q_double = DiscardGradient(q);
  VectorX<double> v_double = DiscardGradient(v);

  // forward kinematics
  KinematicsCache<T> k_cache = tree.doKinematics(q, v);
  KinematicsCache<double> k_cache_double = tree.doKinematics(q_double, v_double);


  // Collision detect 
  VectorXd phi_total;
  Matrix3Xd normal_total, xA_total, xB_total;
  vector<int> idxA_total, idxB_total;

  // This (const cast) is an ugly way of doing it. Change it later if a better method is available
  const_cast<RigidBodyTree<double>&>(tree).collisionDetect(
      k_cache_double, phi_total, normal_total, xA_total, xB_total, idxA_total, idxB_total);

  const int num_total_contacts = normal_total.cols();

  //  Getting the indices of the world and toes
  const int world_ind = GetBodyIndexFromName(tree, "world");
  const int toe_left_ind = GetBodyIndexFromName(tree, "toe_left");
  const int toe_right_ind = GetBodyIndexFromName(tree, "toe_right");

  // Obtaining the indices of valid collisions
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

  Matrix3Xd normal = Matrix3Xd::Zero(normal_total.rows(), num_contacts);
  for (int i = 0; i < num_contacts; i++) {
    normal.col(i) = normal_total.col(contact_ind.at(i));
  }
  
  const Map<Matrix3Xd> normal_map(
      normal.data(), normal_total.rows(), num_contacts);

  vector<Map<Matrix3Xd>> tangents;

  Matrix3Xd tmp_mat1 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map1(tmp_mat1.data(), 3, 4);
  Matrix3Xd tmp_mat2 = Matrix3Xd::Zero(3, 4);
  Map<Matrix3Xd> tmp_map2(tmp_mat2.data(), 3, 4);
  tangents.push_back(tmp_map1);
  tangents.push_back(tmp_map2);

  tree.surfaceTangents(normal_map, tangents);


  //Computing the position Jacobian
  vector<MatrixX<T>> Jd(num_contacts);

  for (int i=0; i<num_contacts; i++) {
    auto tmp_JA = tree.transformPointsJacobian(k_cache,
                                                xA_total.col(contact_ind.at(i)), 
                                                idxA_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    auto tmp_JB = tree.transformPointsJacobian(k_cache,
                                                xB_total.col(contact_ind.at(i)), 
                                                idxB_total.at(contact_ind.at(i)),
                                                world_ind, 
                                                true);
    Jd.at(i) = tmp_JA - tmp_JB;
  }


  //Computing the 3 jacobians for each contact point
  MatrixX<T> J(num_contact_constraints, tree.get_num_positions());

  for (int i=0; i<num_contacts; i++) {

    MatrixX<T> J_pt(num_contact_constraints, tree.get_num_positions());

    auto normal_pt = normal.col(contact_ind.at(i));
    auto tangent1_pt = tangents.at(0).col(contact_ind.at(i));
    auto tangent2_pt = tangents.at(1).col(contact_ind.at(i));

    J_pt.row(0) = normal_pt.transpose()*Jd.at(i);
    J_pt.row(1) = tangent1_pt.transpose()*Jd.at(i);
    J_pt.row(2) = tangent2_pt.transpose()*Jd.at(i);

    J.block(i*num_constraints_per_contact, 0, num_constraints_per_contact, tree.get_num_positions()) =  J_pt;
            
  }

  return J;
}
}  // namespace dairlib
