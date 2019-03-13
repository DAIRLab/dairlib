#pragma once

#include "attic/multibody/contact_toolkit.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {
namespace multibody {

/*
 * PositionConstraint class for solving position constraints in the tree. The
 * tree is provided while creating an object of the class and all the required
 * information is taken from the tree. The required position constraint jacobian
 * is taken from the tree.
 * The constraint solves for the generalized positions (q) (Not the whole state)
 * that satisfies the position constraints of the tree.
 * Getter/Setter interfaces have not been provided as the Constraint class
 * objects would ideally be created inside a solver class and not created
 * outside.
 */
class PositionConstraint : public drake::solvers::Constraint {
 public:
  PositionConstraint(const RigidBodyTree<double>& tree,
                     const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  const RigidBodyTree<double>& tree_;
};

/*
 * ContactConstraint class for solving for point to plane contacts.
 * Specifically, the z-plane (ground plane). The constraint assumes that the
 * given contact information is for point-gound plane contacts and solves it
 * accordingly.
 * The constraint solves for the generalized positions (q) (Not the whole state)
 * that satisfies the contact constraints.
 * Getter/Setter interfaces have not been provided as the Constraint class
 * objects would ideally be created inside a solver class and not created
 * outside.
 */
class ContactConstraint : public drake::solvers::Constraint {
 public:
  ContactConstraint(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                    const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  std::unique_ptr<ContactToolkit<drake::AutoDiffXd>> contact_toolkit_;
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int num_position_forces_;
  const int num_contacts_;
  const int num_forces_;
};

/*
 * FixedPointConstraint class for solving for a fixed point of the system. This
 * may be computed with or without any contact information provided.
 * The constraint solves for the generalized positions (q) (Not the whole
 * state), control inputs (u) and constraint forces (lambda).
 * Contact information is provided through the ContactInfo parameter. All
 * contact information corresponds to point-ground plane contacts.
 * If no contact information is provided, q, u and lambda are computed for which
 * v and vdot are zero. In this case, lambda corresponds to the tree position
 * constraint forces.
 * If contact information is provided, contact forces are also taken into
 * account while computing q, u and lambda for which v and vdot are zero. The
 * solution of q then also makes sure that the required contacts with the ground
 * plane are made.
 * Getter/Setter interfaces have not been provided as the Constraint class
 * objects would ideally be created inside a solver class and not created
 * outside.
 */
class FixedPointConstraint : public drake::solvers::Constraint {
 public:
  FixedPointConstraint(const RigidBodyTree<double>& tree,
                       ContactInfo contact_info = ContactInfo(),
                       const std::string& description = "");
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& q_u_l,
              Eigen::VectorXd* y) const override;
  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& q_u_l,
              drake::AutoDiffVecXd* y) const override;
  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& q_u_l,
      drake::VectorX<drake::symbolic::Expression>* y) const override;

 private:
  std::unique_ptr<ContactToolkit<drake::AutoDiffXd>> contact_toolkit_;
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  const int num_positions_;
  const int num_velocities_;
  const int num_efforts_;
  const int num_position_forces_;
  const int num_forces_;
};

/*
 * PositionSolver class that acts as a solver interface to solve for tree
 * position constraints. The class handles creation and use of the
 * PositionConstraint object.
 */
class PositionSolver {
 public:
  // Disabling copy construction and assignment.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PositionSolver)

  PositionSolver(const RigidBodyTree<double>& tree);
  PositionSolver(const RigidBodyTree<double>& tree,
                 const Eigen::VectorXd q_desired);
  PositionSolver(const RigidBodyTree<double>& tree,
                 const Eigen::VectorXd q_desired, const Eigen::MatrixXd Q);

  /*
   * Function to set the initial guess of q.
   * If an initial guess needs to be specified, this function must be called
   * before calling Solve()
   * @param q The initial guess for the generalized positions q
   */
  void SetInitialGuessQ(Eigen::VectorXd q);
  /*
   * Function to add the quadratic cost to the program.
   * The cost that is added is of the form:
   * Cost = (q - q_desired)'*Q*(q - q_desired);
   * @param q_desired The desired q value in the cost.
   * @param Q The Q matrix in the cost (size num_positions x num_positions).
   */
  void AddProgramCost(const Eigen::VectorXd q_desired, const Eigen::MatrixXd Q);
  /*
   * Function to fix the values of certain joints. The joints to be fixed are
   * passed as a map, together with the values. The indices are with respect to
   * the generalized positions q.
   * map(joint_index_i -> joint_value_i)
   * @param fixed_joints Map of joint indices that are to be fixed to the mapped
   * joint values values.
   */
  void AddFixedJointsConstraint(std::map<int, double> fixed_joints);
  /*
   * Function to add joint limit constraints.
   * The joint limits are obtained from the tree that the PositionSolver object
   * was created with. The tolerance specifies the amount of relaxation required
   * from the min or max values. This can be used to prevent the solutions from
   * being too close to the extreme values which may be unstable.
   * The constraint is computed as:
   * (min + tolerance) <= q_i <= (max - tolerance) for each value in q.
   * @param tolerance Requried tolerance value
   */
  void AddJointLimitConstraint(const double tolerance);
  /*
   * Function to solve the problem after all the required initial values and
   * other constraints have been set.
   * Returns a MathematicalProgramResult type that gives an indication of
   * whether the solver could successfully solve the problem or not.
   */
  drake::solvers::MathematicalProgramResult Solve();
  /*
   * Function to check if the a given value of q satisfies the position
   * constraints.
   * This may be used after Solve() to make sure that the q value obtained
   * satisfies the required constraints.
   * @param tolerance Tolerance upto which the constraint needs to be satisfied.
   * If the constraints are not satisfied, it prints the status of each
   * constraint to the standard output stream.
   */
  bool CheckConstraint(Eigen::VectorXd q, double tolerance = 1.0e-10) const;

  /*
   * Function to get a shared pointer to the Mathematical program that runs the
   * solver.
   */
  std::shared_ptr<drake::solvers::MathematicalProgram> get_program();
  /*
   * Function to obtain the result of the Solve() function through a
   * MathematicalProgramResult type.
   */
  drake::solvers::MathematicalProgramResult get_program_result();
  /*
   * Get the solved q value.
   * This needs to be called after Solve() to get the final solution.
   */
  Eigen::VectorXd GetSolutionQ();

  /*
   * Setters and getters to change the filename for the log files and the
   * major/minor tolerance values used by the solver.
   */
  void set_filename(std::string filename);
  void set_major_tolerance(double major_tolerance);
  void set_minor_tolerance(double minor_tolerance);

  double get_major_tolerance();
  double get_minor_tolerance();

 private:
  const RigidBodyTree<double>& tree_;
  std::shared_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::VectorXDecisionVariable q_;
  drake::solvers::MathematicalProgramResult program_result_;

  double major_tolerance_ = 1.0e-13;
  double minor_tolerance_ = 1.0e-13;
};

/*
 * ContactSolver class that acts as a solver interface to solve for contact
 * constraints. Contacts are specified through a ContactInfo object that
 * specifies point-ground plane contacts. The class handles creation and use of
 * the ContactConstraint object.
 */
class ContactSolver {
 public:
  // Disabling copy construction and assignment.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactSolver)

  ContactSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info);
  ContactSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                const Eigen::VectorXd q_cost);
  ContactSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                const Eigen::VectorXd q_cost, const Eigen::MatrixXd Q);

  /*
   * Function to set the initial guess of q.
   * If an initial guess needs to be specified, this function must be called
   * before calling Solve()
   * @param q The initial guess for the generalized positions q
   */
  void SetInitialGuessQ(Eigen::VectorXd q);
  /*
   * Function to add the quadratic cost to the program.
   * The cost that is added is of the form:
   * Cost = (q - q_desired)'*Q*(q - q_desired);
   * If the Q matrix is not provided, it defaults to the identity, causing the
   * cost to be a simple dot product.
   * @param q_desired The desired q value in the cost.
   * @param Q The Q matrix in the cost (Size num_positions x num_positions).
   */
  void AddProgramCost(const Eigen::VectorXd q_desired, const Eigen::MatrixXd Q);
  /*
   * Function to fix the values of certain joints. The joints to be fixed are
   * passed as a map, together with the values. The indices are with respect to
   * the generalized positions q.
   * map(joint_index_i -> joint_value_i)
   * @param fixed_joints Map of joint indices that are to be fixed to the mapped
   * joint values values.
   */
  void AddFixedJointsConstraint(std::map<int, double> fixed_joints);
  /*
   * Function to add joint limit constraints.
   * The joint limits are obtained from the tree that the ContactSolver object
   * was created with. The tolerance specifies the amount of relaxation required
   * from the min or max values. This can be used to prevent the solutions from
   * being too close to the extreme values which may be unstable.
   * The constraint is computed as:
   * (min + tolerance) <= q_i <= (max - tolerance) for each value in q.
   * @param tolerance Requried tolerance value
   */
  void AddJointLimitConstraint(const double tolerance);
  /*
   * Function to solve the problem after all the required initial values and
   * other constraints have been set.
   * Returns a MathematicalProgramResult type that gives an indication of
   * whether the solver could successfully solve the problem or not.
   */
  drake::solvers::MathematicalProgramResult Solve();
  /*
   * Function to check if the a given value of q satisfies the contact
   * constraints.
   * This may be used after Solve() to make sure that the q value obtained
   * satisfies the required constraints.
   * @param tolerance Tolerance upto which the constraint needs to be satisfied.
   * If the constraints are not satisfied, it prints the status of each
   * constraint to the standard output stream.
   */
  bool CheckConstraint(Eigen::VectorXd q, double tolerance = 1.0e-10) const;

  /*
   * Function to get a shared pointer to the Mathematical program that runs the
   * solver.
   */
  std::shared_ptr<drake::solvers::MathematicalProgram> get_program();
  /*
   * Function to obtain the result of the Solve() function through a
   * MathematicalProgramResult type.
   */
  drake::solvers::MathematicalProgramResult get_program_result();
  /*
   * Get the solved q value.
   * This needs to be called after Solve() to get the final solution.
   */
  Eigen::VectorXd GetSolutionQ();

  /*
   * Setters and getters to change the filename for the log files and the
   * major/minor tolerance values used by the solver.
   */
  void set_filename(std::string filename);
  void set_major_tolerance(double major_tolerance);
  void set_minor_tolerance(double minor_tolerance);

  double get_major_tolerance();
  double get_minor_tolerance();

 private:
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  std::shared_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::VectorXDecisionVariable q_;
  drake::solvers::MathematicalProgramResult program_result_;
  double major_tolerance_ = 1.0e-13;
  double minor_tolerance_ = 1.0e-13;
};

/*
 * FixedPointSolver class that acts as a solver interface to solve for fixed
 * point constraints. Contacts are specified through a ContactInfo object that
 * specifies point-ground plane contacts. If contact information is provided
 * while creating the object, contact constraints are also satisfied and this
 * information is used while computing xdot. The class handles creation and use
 * of the FixedPointConstraint object.
 */
class FixedPointSolver {
 public:
  // Disabling copy construction and assignment.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedPointSolver)

  /*
   * Constructors for solving without contact constraints.
   */
  FixedPointSolver(const RigidBodyTree<double>& tree);
  FixedPointSolver(const RigidBodyTree<double>& tree,
                   const Eigen::VectorXd q_desired,
                   const Eigen::VectorXd u_desired);
  FixedPointSolver(const RigidBodyTree<double>& tree,
                   const Eigen::VectorXd q_desired,
                   const Eigen::VectorXd u_desired, const Eigen::MatrixXd Q,
                   const Eigen::MatrixXd U);
  /*
   * Constructor for solving with contact constraints. The ContactInfo object
   * stores point-ground plane contact information.
   */
  FixedPointSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info);
  FixedPointSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                   Eigen::VectorXd q_desired, const Eigen::VectorXd u_desired);
  FixedPointSolver(const RigidBodyTree<double>& tree, ContactInfo contact_info,
                   Eigen::VectorXd q_desired, const Eigen::VectorXd u_desired,
                   const Eigen::MatrixXd Q, const Eigen::MatrixXd U);

  /*
   * Function to set the initial guess of q.
   * If an initial guess needs to be specified, this function must be called
   * before calling Solve()
   * @param q The initial guess for the generalized positions q
   */
  void SetInitialGuessQ(Eigen::VectorXd q);
  /*
   * Function to set the initial guess of u.
   * If an initial guess needs to be specified, this function must be called
   * before calling Solve()
   * @param u The initial guess for the control inputs u.
   */
  void SetInitialGuessU(Eigen::VectorXd u);
  /*
   * Function to set the initial guess of lambda.
   * If an initial guess needs to be specified, this function must be called
   * before calling Solve()
   * @param u The initial guess for the constraint forces lambda.
   */
  void SetInitialGuessLambda(Eigen::VectorXd lambda);
  /*
   * Function to set the initial guesses for all the variables at once.
   * @param q The initial guess for q.
   * @param u The initial guess for u.
   * @param lambda The initial guess for lambda.
   */
  void SetInitialGuess(Eigen::VectorXd q, Eigen::VectorXd u,
                       Eigen::VectorXd lambda);
  /*
   * Function to add the quadratic cost to the program.
   * The cost that is added is of the form:
   * Cost = (q - q_desired)'*Q*(q - q_desired) +
   * (u - u_desired)'*U*(u - u_desired)
   * If the Q and U matrix is not provided, it defaults to the identity, causing
   * the cost to be a simple dot product.
   * @param q_desired The desired q value in the cost.
   * @param u_desired The desired u value in the cost.
   * @param Q The Q matrix in the cost (size num_positions x num_positions).
   * @param U The U matrix in the cost (size num_actuators x num_actuators).
   */
  void AddProgramCost(const Eigen::VectorXd q_desired,
                      const Eigen::VectorXd u_desired, const Eigen::MatrixXd Q,
                      const Eigen::MatrixXd U);
  /*
   * Function to add an L-2 norm cost on all the normal lambda values to ensure
   * uniform distribution of the normal forces.
   */
  void AddSpreadNormalForcesCost();
  /*
   * Function to add friction cone constraints at all the the contact points.
   * The constraints take the form |fn_i| <= mu * ft_i, Where
   * fn_i = normal force at the ith contact point.
   * ft_i = tangential force at the ith contact point.
   * mu = Static coefficient of friction.
   * @param mu Static Coefficient of friction mu.
   */
  void AddFrictionConeConstraint(const double mu);
  /*
   * Function to fix the values of certain joints. The joints to be fixed are
   * passed as a map, together with the values. The indices are with respect to
   * the generalized positions q.
   * map(joint_index_i -> joint_value_i)
   * @param fixed_joints Map of joint indices that are to be fixed to the mapped
   * joint values values.
   */
  void AddFixedJointsConstraint(std::map<int, double> fixed_joints);
  /*
   * Function to add joint limit constraints.
   * The joint limits are obtained from the tree that the FixedPointSolver
   * object
   * was created with. The tolerance specifies the amount of relaxation required
   * from the min or max values. This can be used to prevent the solutions from
   * being too close to the extreme values which may be unstable.
   * The constraint is computed as:
   * (min + tolerance) <= q_i <= (max - tolerance) for each value in q.
   * @param tolerance Requried tolerance value
   */
  void AddJointLimitConstraint(const double tolerance);
  /*
   * Function to solve the problem after all the required initial values and
   * other constraints have been set.
   * @param q Value of q used in the quadratic cost. The solution q would be as
   * close to this value as possible.
   * @param fixed_joints A vector of integers that correspond to indices of q
   * whose values are not allowed to change.
   * Returns a MathematicalProgramResult type that gives an indication of
   * whether the
   * solver could successfully solve the problem or not.
   */
  drake::solvers::MathematicalProgramResult Solve();
  /*
   * Function to check if the a given values of q, u and lambda satisfies the
   * contact constraints.
   * This may be used after Solve() to make sure that the q value obtained
   * satisfies the required constraints.
   * @param tolerance Tolerance upto which the constraint needs to be satisfied.
   * If the constraints are not satisfied, it prints the status of each
   * constraint to the standard output stream.
   */
  bool CheckConstraint(Eigen::VectorXd q, Eigen::VectorXd u,
                       Eigen::VectorXd lambda,
                       double tolerance = 1.0e-10) const;

  /*
   * Function to get a shared pointer to the Mathematical program that runs the
   * solver.
   */
  std::shared_ptr<drake::solvers::MathematicalProgram> get_program();
  /*
   * Function to obtain the result of the Solve() function through a
   * MathematicalProgramResult type.
   */
  drake::solvers::MathematicalProgramResult get_program_result();
  /*
   * Get the solved q value.
   * This needs to be called after Solve() to get the final solution.
   */
  Eigen::VectorXd GetSolutionQ();
  /*
   * Get the solved u value.
   * This needs to be called after Solve() to get the final solution.
   */
  Eigen::VectorXd GetSolutionU();
  /*
   * Get the solved lambda value.
   * This needs to be called after Solve() to get the final solution.
   */
  Eigen::VectorXd GetSolutionLambda();

  /*
   * Setters and getters to change the filename for the log files and the
   * major/minor tolerance values used by the solver.
   */
  void set_filename(std::string filename);
  void set_major_tolerance(double major_tolerance);
  void set_minor_tolerance(double minor_tolerance);

  double get_major_tolerance();
  double get_minor_tolerance();

 private:
  const RigidBodyTree<double>& tree_;
  ContactInfo contact_info_;
  std::shared_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::VectorXDecisionVariable q_;
  drake::solvers::VectorXDecisionVariable u_;
  drake::solvers::VectorXDecisionVariable lambda_;
  drake::solvers::MathematicalProgramResult program_result_;
  double major_tolerance_ = 1.0e-13;
  double minor_tolerance_ = 1.0e-13;
};

}  // namespace multibody
}  // namespace dairlib
