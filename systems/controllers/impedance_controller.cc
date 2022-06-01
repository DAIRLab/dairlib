#include "impedance_controller.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "multibody/geom_geom_collider.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"

#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"


//#include
//"external/drake/common/_virtual_includes/autodiff/drake/common/eigen_autodiff_types.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::VectorX;
using drake::SortedPair;
using drake::geometry::GeometryId;
//using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::math::RotationMatrix;
using drake::multibody::MultibodyPlant;
using drake::multibody::JacobianWrtVariable;
using drake::systems::Context;


using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using std::vector;

// test functions for the joint and cartesian impedance controllers

// motion planning test for the joint impedance controller
// bends all joints to right angles
vector<VectorXd> compute_target_joint_space_vector(double t){
    // set start and end points in joint space
    VectorXd start = VectorXd::Zero(7);
    VectorXd end = 1.57*VectorXd::Ones(7);
    start(3) = -0.0698;
    end(3) = -1.57;

    // path parameters
    double start_time = 5.0;
    double duration = 5.0;
    double end_time = start_time+duration;

    // return q_des and q_dot_des
    if (t < start_time) { // wait for controller to stabilize
        return {start, VectorXd::Zero(7)};
    }
    else if (t > end_time){
        return {end, VectorXd::Zero(7)};
    }    
    else {
        VectorXd v = (end-start) / duration;
        double a = (t-start_time) / duration;
        return {(1-a)*start + a*end, v};
    }
}

// motion planning test for the cartesian impedance controller
// traces a small horizontal circle
std::vector<Vector3d> compute_target_task_space_vector(double t){
    // tracks a cirle in task sapce
    double r = 0.125;
    double x_c = 0.6; // smaller x_c performs worse
    double y_c = 0;
    double z_c = 0.2;
    double w = 2;
    Vector3d start(x_c+r, y_c, z_c);
    double start_time = 1.0;

    // return x_des and x_dot_des
    if (t < start_time){ // wait for controller to stabilize
      return {start, VectorXd::Zero(3)};
    }
    else{
      Vector3d x_des(x_c + r*cos(w*(t-start_time)), y_c + r*sin(w*(t-start_time)), z_c);
      Vector3d x_dot_des(-r*w*sin(w*(t-start_time)), r*w*cos(w*(t-start_time)), 0);
      return {x_des, x_dot_des};
    }
}

// TODO: IK function (INCOMPLETE)
VectorXd inverse_kinematics(const MultibodyPlant<double>& plant, 
    const Vector3d& x){
    
    drake::multibody::InverseKinematics ik(plant);
    double eps = 1e-4;
    
    // define frames
    const auto& world_frame = plant.world_frame();
    const auto& EE_frame = plant.GetFrameByName("panda_link8");

    ik.AddPositionConstraint(EE_frame, Vector3d(0,0,0), world_frame, 
            x - eps*VectorXd::Ones(3),
            x + eps*VectorXd::Ones(3));
    // TODO: add orientation constraint
    // TODO: figure out how to warm start this
    //ik.get_mutable_prog()->SetInitialGuess(ik.q(), warm_start);
    const auto result = Solve(ik.prog());
    const auto q_sol = result.GetSolution(ik.q());

    return q_sol;
}

namespace dairlib {
namespace systems {
namespace controllers {

ImpedanceController::ImpedanceController(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const MatrixXd& K, 
    const MatrixXd& B,
    const std::vector<drake::geometry::GeometryId>& contact_geoms,
    int num_friction_directions)
    : plant_(plant),
      context_(context),
      K_(K),
      B_(B),
      contact_geoms_(contact_geoms),
      num_friction_directions_(num_friction_directions){
  
  // plant parameters
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();

  // set up input and output ports
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &ImpedanceController::CalcControl)
                             .get_index();

  // define end effector
  EE_offset_ << 0, 0, 0;
  EE_frame_ = &plant_.GetBodyByName("panda_link8").body_frame();
  world_frame_ = &plant_.world_frame();

}


// CARTESIAN IMPEDANCE CONTROLLER
void ImpedanceController::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {

  // get values
  const int n = 7;
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd u = robot_output->GetEfforts();
  
  //update the context_
  VectorXd C(plant_.num_velocities());
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);

  // calculate corriolis and gravity terms
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // compute jacobian
  MatrixXd J(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context_, JacobianWrtVariable::kV,
      *EE_frame_, EE_offset_,
      *world_frame_, *world_frame_, &J);

  // perform all truncations
  VectorXd q_franka = q.head(n);
  VectorXd v_franka = v.head(n);
  VectorXd C_franka = C.head(n);
  VectorXd tau_g_franka = tau_g.head(n);
  MatrixXd J_franka = J.block(0, 0, 6, n);
  
  // forward kinematics
  const drake::math::RigidTransform<double> H = 
    plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("panda_link8"));
  auto d = H.translation();
  auto R = H.rotation();

  // build task space state vectors
  VectorXd x = VectorXd::Zero(6);
  x.tail(3) << d;
  VectorXd x_dot = J_franka * v_franka;

  // TODO: get desired x and x_dot from input ports
  std::vector<Vector3d> target = compute_target_task_space_vector(timestamp);
  VectorXd xd = VectorXd::Zero(6);
  xd.tail(3) << target[0];
  VectorXd xd_dot = VectorXd::Zero(6);
  xd_dot.tail(3) << target[1];

  // TODO: using fixed Rd for the time being, this is subject to change
  Matrix3d Rd_eigen;
  Rd_eigen << 
    1,  0,  0,
    0, -1,  0,
    0,  0, -1;
  RotationMatrix<double> Rd(Rd_eigen);

  // compute rotational error, computational steps taken from
  // https://github.com/frankaemika/libfranka/blob/master/examples/cartesian_impedance_control.cpp
  Quaterniond orientation = R.ToQuaternion();
  Quaterniond orientation_d = Rd.ToQuaternion();

  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0){
    orientation.coeffs() << -orientation.coeffs();
  }
  Quaterniond error_quaternion(orientation.inverse() * orientation_d);
  Vector3d error_quaternion_no_w(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
  Vector3d rotational_error = R.matrix() * error_quaternion_no_w;
  
  // compute xtilde and xtilde_dot
  VectorXd xtilde = xd - x;
  xtilde.head(3) << rotational_error;
  // TODO: will I get xd_dot?
  VectorXd xtilde_dot = xd_dot - x_dot;
  //VectorXd xtilde_dot = -x_dot;

  // compute the input with feedforward contact term
  VectorXd tau = J_franka.transpose() * (K_*xtilde + B_*xtilde_dot) 
                  + C_franka - tau_g_franka;
  // TODO: get J_c = dphi(q)/dq, need to add ball and EE to the urdf
  // For now, just adding lambda_d in the positive x direction
  // after 5 seconds as proof of concept
  // if (timestamp > 5.0){
  //   VectorXd lambda_des = VectorXd::Zero(6);
  //   lambda_des(5) = 20; // set desired force in z direction
  //   tau += J_franka.transpose() * lambda_des;
  // }

  // feedforward force term
  // TODO: check that this is done properly
  // ex. confirm dimensions of everything
  std::vector<SortedPair<GeometryId>> contact_pairs;
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1])); // EE <-> Sphere
  // contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[2])); // Sphere <-> Ground
  
  VectorXd phi(contact_pairs.size());
  MatrixXd J_n(contact_pairs.size(), plant_.num_velocities());
  MatrixXd J_t(2 * contact_pairs.size() * num_friction_directions_,
               plant_.num_velocities());

  for (int i = 0; i < (int) contact_pairs.size(); i++) {
    multibody::GeomGeomCollider collider(
        plant_, contact_pairs[i]);  // deleted num_fricton_directions (check with
                                   // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions_);

    phi(i) = phi_i;

    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions_, 0, 2 * num_friction_directions_,
              plant_.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions_, plant_.num_velocities());
  }
  
  // compute nullspace projection
  MatrixXd J_franka_pinv = J_franka.completeOrthogonalDecomposition().pseudoInverse();
  MatrixXd N = MatrixXd::Identity(7, 7) - J_franka.transpose() * J_franka_pinv.transpose();

  VectorXd qd = VectorXd::Zero(7);
  // TODO: which qd do I use?
  qd << 0, 0, 0, -1.57, q_franka.tail(3); // task-specific
  //qd << 0, 0, 0, -1.57, 0, 1.57, 0;; // middle of range of motion
  
  // TODO: parameter tune these if necessary
  MatrixXd K_null = MatrixXd::Identity(7, 7);
  MatrixXd B_null = MatrixXd::Identity(7, 7);
  VectorXd tau_null = K_null*(qd-q_franka) - B_null*v_franka;

  control->SetDataVector(tau + N*tau_null);
  control->set_timestamp(timestamp);

  // debug prints every 10th of a second
  // if (trunc(timestamp*10) / 10.0 == timestamp){
  //   std::cout << timestamp << "\n---------------" << std::endl;
  //   std::cout << "phi\n:" << phi << std::endl;
  //   std::cout << J_n.rows() << " " << J_n.cols() << std::endl;
  //   std::cout << J_t.rows() << " " << J_t.cols() << std::endl;
  // }
}

/*
// JOINT IMPEDANCE CONTROLLER
void ImpedanceController::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {

  /// get values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd u = robot_output->GetEfforts();
  
  //update the context_
  VectorXd C(plant_.num_velocities());
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);

  // calculate corriolis and gravity terms
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // TODO: get desired x and x_dot from input ports
  // TODO: get qd and qd_dot from task space and IK instead
  vector<VectorXd> target = compute_target_joint_space_vector(timestamp);
  VectorXd qd = target[0];
  VectorXd qd_dot = target[1];

  // TODO: finish IK with warm start
  // Vector3d xd = compute_target_task_space_vector(timestamp);
  // VectorXd qd = inverse_kinematics(plant_, xd);
  // VectorXd qd_dot = VectorXd::Ones(7); // set joint velocities to 0 for now
  
  // gain matrices
  int num_joints = plant_.num_positions();
  MatrixXd Kp = MatrixXd::Zero(num_joints, num_joints);
  MatrixXd Kd = MatrixXd::Zero(num_joints, num_joints);
  // gains from
  // https://github.com/frankaemika/libfranka/blob/master/examples/joint_impedance_control.cpp
  std::vector<double> P_gains = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
  std::vector<double> D_gains = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
  double ratio = 0.25;
  for (int i = 0; i < num_joints; i++){
      Kp(i,i) = P_gains[i]*ratio;
      Kd(i,i) = D_gains[i]*ratio;
  }
  
  // TODO: add limit on tau?
  VectorXd tau = Kp*(qd - q) + Kd*(qd_dot - v) + C - tau_g;
  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
}
*/

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib