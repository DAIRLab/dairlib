#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/output_vector.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/controllers/controlUtil.h"
#include "gurobi_c++.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::Map;
using drake::systems::LeafSystem;
using drake::systems::Context;
namespace dairlib{
namespace systems {

class MpcBalanceController : public LeafSystem<double> {
 public:
 // initialize with the equilibrium point and matrix from rigid body tree
  MpcBalanceController(int num_positions, int num_velocities, int num_inputs,RigidBodyTree<double>& tree,VectorXd x_des,
  VectorXd u_des, VectorXd lambda_des);

// get input port. output_input_port_ and config_input_port_ record
// dont need to receive the gains,only need 
  const drake::systems::InputPort<double>& get_input_port_output()
      const {
    return this->get_input_port(output_input_port_);
  }

  void get_optimization_matrix(MatrixXd dynamic_matrix[14],int N,VectorXd x0) const;
  //void get_optimization_matrix_no_sliding(MatrixXd dynamic_matrix[6],MatrixXd optimizaion_matrix[6],int N,VectorXd x0) const;
  VectorXd admm_solve(MatrixXd dynamic_matrix[9],int N,VectorXd xd,int rho) const;
  //void admm_solve_no_sliding(MatrixXd dynamic_matrix[6]) ;
 
 private:
  void CalcControl(const Context<double>& context,
                   TimestampedVector<double>* output) const;

  int output_input_port_;

  // desired states and gain matrix
  VectorXd x_des_;
  VectorXd u_des_;
  VectorXd lambda_des_;

  int num_states_;
  int num_inputs_;
  int num_contacts_nor_;
  int num_contacts_tan_;

  MatrixXd A_;
  MatrixXd B_;
  MatrixXd D_;
  MatrixXd n_;
  MatrixXd dlim_;
  MatrixXd invMD_;
  MatrixXd invMn_;
  MatrixXd invMJtree_;
  VectorXd const_;

  // limit for state,input and force
  VectorXd qmax_;
  VectorXd qmin_;
  VectorXd umax_;
  VectorXd umin_;
  VectorXd cmax_;
  VectorXd cmin_;

  double dt_ = 0.02;
};

}
}