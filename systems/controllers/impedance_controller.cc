#include "impedance_controller.h"

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

bool isZeroVector(const VectorXd& a, double eps = 1e-6){
  assert(eps > 0);
  return (a.array().abs() <= eps*VectorXd::Ones(a.size()).array()).all();
}

namespace dairlib {
namespace systems {
namespace controllers {

ImpedanceController::ImpedanceController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::MultibodyPlant<double>& plant_f,
    drake::systems::Context<double>& context,
    drake::systems::Context<double>& context_f,
    const MatrixXd& K, 
    const MatrixXd& B,
    const MatrixXd& K_null,
    const MatrixXd& B_null,
    const VectorXd& qd,
    const std::vector<drake::geometry::GeometryId>& contact_geoms,
    int num_friction_directions,
    double moving_offset,
    double pushing_offset)
    : plant_(plant),
      plant_f_(plant_f),
      context_(context),
      context_f_(context_f),
      K_(K),
      B_(B),
      K_null_(K_null),
      B_null_(B_null),
      qd_(qd),
      contact_geoms_(contact_geoms),
      num_friction_directions_(num_friction_directions),
      moving_offset_(moving_offset),
      pushing_offset_(pushing_offset){
  
  // plant parameters
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();

  // set up input and output ports
  franka_state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();
  
  // xee: 3D, xball: 7D, xee_dot: 3D, xball_dot: 6D, lambda: 6D (Total: 25D)
  c3_state_input_port_ =
      this->DeclareVectorInputPort(
              "xee, xball, xee_dot, xball_dot, lambda",
              TimestampedVector<double>(28))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &ImpedanceController::CalcControl)
                             .get_index();

  // get c3_parameters
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");

  // define end effector and contact
  EE_offset_ << param_.EE_offset;
  EE_frame_ = &plant_.GetBodyByName("panda_link8").body_frame();
  world_frame_ = &plant_.world_frame();
  contact_pairs_.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1])); // EE <-> Sphere
  n_ = 7;
  enable_heuristic_ = param_.enable_heuristic;

  // control-related variables
  // TODO: using fixed Rd for the time being
  Matrix3d Rd_eigen;
  Rd_eigen << 
    1,  0,  0,
    0, -1,  0,
    0,  0, -1;
  RotationMatrix<double> Rd(Rd_eigen);
  orientation_d_ = Rd.ToQuaternion();
}


// CARTESIAN IMPEDANCE CONTROLLER
void ImpedanceController::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {
                                 
  auto start = std::chrono::high_resolution_clock::now();
  // parse values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_input_port_);
  double timestamp = robot_output->get_timestamp();
  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();
  VectorXd u = robot_output->GetEfforts();

  // get values from c3
  auto c3_output =
      (TimestampedVector<double>*) this->EvalVectorInput(context, c3_state_input_port_);
  VectorXd state = c3_output->get_data();
  VectorXd xd = VectorXd::Zero(6);
  VectorXd xd_dot = VectorXd::Zero(6);
  VectorXd lambda = VectorXd::Zero(5); // does not contain the slack variable
  Vector3d ball_xyz(state(7), state(8), state(9));
  Vector3d ball_xyz_d(state(25), state(26), state(27));

  xd.tail(3) << state.head(3);
  xd_dot.tail(3) << state(10), state(11), state(12);
  lambda << state(20), state(21), state(22), state(23), state(24);
  
  //update the context_
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);
  plant_f_.SetPositions(&context_f_, q);
  plant_f_.SetVelocities(&context_f_, v);
  //multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);

  // calculate manipulator equation terms and Jacobian
  MatrixXd M(plant_.num_velocities(), plant_.num_velocities());
  plant_.CalcMassMatrix(context_, &M);
  VectorXd C(plant_.num_velocities());
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  MatrixXd J(6, plant_.num_velocities());
  plant_.CalcJacobianSpatialVelocity(
      context_, JacobianWrtVariable::kV,
      *EE_frame_, EE_offset_,
      *world_frame_, *world_frame_, &J);

  // perform all truncations
  VectorXd q_franka = q.head(n_);
  VectorXd v_franka = v.head(n_);
  MatrixXd M_franka = M.block(0 ,0, n_, n_);
  VectorXd C_franka = C.head(n_);
  VectorXd tau_g_franka = tau_g.head(n_);
  MatrixXd J_franka = J.block(0, 0, 6, n_);
  
  // forward kinematics
  const drake::math::RigidTransform<double> H = 
    plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("panda_link8"));
  const RotationMatrix<double> R = H.rotation();
  Vector3d d = H.translation() + R*EE_offset_;

  // build task space state vectors
  VectorXd x = VectorXd::Zero(6);
  x.tail(3) << d;
  VectorXd x_dot = J_franka * v_franka;

  double settling_time = param_.stabilize_time1 + param_.move_time + param_.stabilize_time2;
  if (enable_heuristic_ && timestamp > settling_time){
    Vector3d xd_new = ApplyHeuristic(xd.tail(3), xd_dot.tail(3), lambda, d, x_dot.tail(3), 
                                 ball_xyz, ball_xyz_d, settling_time, timestamp);
    xd.tail(3) << xd_new;
  }

  // compute position control input
  VectorXd xtilde = xd - x;
  xtilde.head(3) << this->CalcRotationalError(R);
  VectorXd xtilde_dot = xd_dot - x_dot;
  VectorXd tau = J_franka.transpose() * (K_*xtilde + B_*xtilde_dot) + C_franka - tau_g_franka;

  // add feedforward force term if contact is desired
  MatrixXd Jc(contact_pairs_.size() + 2 * contact_pairs_.size() * num_friction_directions_, n_);

 if (lambda.norm() > param_.contact_threshold){
   //std::cout << "here" << std::endl;
   // compute contact jacobian
   VectorXd phi(contact_pairs_.size());
   MatrixXd J_n(contact_pairs_.size(), plant_.num_velocities());
   MatrixXd J_t(2 * contact_pairs_.size() * num_friction_directions_, plant_.num_velocities());
   this->CalcContactJacobians(contact_pairs_, phi, J_n, J_t);
   Jc << J_n.block(0, 0, J_n.rows(), n_),
         J_t.block(0, 0, J_t.rows(), n_);

   tau = tau - Jc.transpose() * lambda;
 }
  // compute nullspace projection
  MatrixXd M_inv = M_franka.inverse();
  MatrixXd J_ginv_tranpose = (J_franka * M_inv * J_franka.transpose()).inverse() * J_franka * M_inv;
  MatrixXd N = MatrixXd::Identity(7, 7) - J_franka.transpose() * J_ginv_tranpose;
  VectorXd qd = VectorXd::Zero(7);
  qd << qd_.head(4), q.tail(3);
  VectorXd tau_null = N * (K_null_*(qd-q_franka) - B_null_*v_franka);

  control->SetDataVector(tau + tau_null);
  control->set_timestamp(timestamp);
  
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  // debug prints every 10th of a second
  int print_enabled = 0; // print flag
  if (print_enabled && trunc(timestamp*10) / 10.0 == timestamp && timestamp >= settling_time){
    std::cout << timestamp << "\n---------------" << std::endl;
    std::cout << lambda.norm() << std::endl;
    std::cout << std::endl;
  }
}

Vector3d ImpedanceController::CalcRotationalError(const RotationMatrix<double>& R) const {
  // compute rotational error, computational steps taken from
  // https://github.com/frankaemika/libfranka/blob/master/examples/cartesian_impedance_control.cpp
  Quaterniond orientation = R.ToQuaternion();

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0){
    orientation.coeffs() << -orientation.coeffs();
  }
  Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  Vector3d error_quaternion_no_w(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
  return R.matrix() * error_quaternion_no_w;
}

void ImpedanceController::CalcContactJacobians(const std::vector<SortedPair<GeometryId>>& contact_pairs,
                            VectorXd& phi, MatrixXd& J_n, MatrixXd& J_t) const {
  for (int i = 0; i < (int) contact_pairs.size(); i++) {
    multibody::GeomGeomCollider collider(
        plant_f_, contact_pairs[i]);  // deleted num_fricton_directions (check with
                                   // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context_f_, num_friction_directions_);
    phi(i) = phi_i;

    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions_, 0, 2 * num_friction_directions_,
              plant_f_.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions_, plant_f_.num_velocities());
  }
}

Vector3d ImpedanceController::ApplyHeuristic(
    const VectorXd& xd, const VectorXd& xd_dot, const VectorXd& lambda,
    const VectorXd& x, const VectorXd& x_dot,
    const VectorXd& ball_xyz, const VectorXd& ball_xyz_d,
    double settling_time, double timestamp) const {
  
  Vector3d xd_new = xd;
  Vector3d ball_to_EE = (x-ball_xyz) / (x-ball_xyz).norm();

  double period = param_.period;
  double duty_cycle = param_.duty_cycle;
  double shifted_time = timestamp - settling_time;
  double ts = shifted_time - period * floor((shifted_time / period));

  if (lambda.norm() > 0.001 && ts < period * duty_cycle && x_dot(2) < 0){
    //std::cout << "here" << std::endl;
    double diff = (x-ball_xyz).norm() - param_.ball_radius - param_.finger_radius;
    if (diff > 0){
      xd_new = xd_new - diff*ball_to_EE;
    }
    xd_new = xd_new + pushing_offset_*ball_to_EE;
  }
  else {
    double diff = (x-ball_xyz).norm() - param_.ball_radius - param_.finger_radius;
    if (diff < 0){
      xd_new = xd_new - diff*ball_to_EE;
    }
    xd_new = xd_new + moving_offset_*ball_to_EE;
  }
  return xd_new;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib