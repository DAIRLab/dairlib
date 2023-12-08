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
    int num_friction_directions)
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
      num_friction_directions_(num_friction_directions){

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

  // xee: 7D, xball: 7D, xee_dot: 3D, xball_dot: 6D, lambda: 6D (Total: 29D + visuals)
  c3_state_input_port_ =
      this->DeclareVectorInputPort(
              "xee, xball, xee_dot, xball_dot, lambda, visualization",
              TimestampedVector<double>(38))
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
  EE_frame_ = &plant_.GetBodyByName("end_effector_tip").body_frame();
  world_frame_ = &plant_.world_frame();
  contact_pairs_.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1])); // EE <-> Sphere
  n_ = 7;
  enable_contact_ = param_.enable_contact;

  // define franka limits
  // TODO: parse these from urdf instead of hard coding
  torque_limits_ = VectorXd::Zero(n_);
  torque_limits_ << 87, 87, 87, 87, 12, 12, 12;

  // control-related variables
  // TODO: using fixed Rd for the time being
  Matrix3d Rd_eigen;
  Rd_eigen <<
    1,  0,  0,
    0, -1,  0,
    0,  0, -1;
  RotationMatrix<double> Rd(Rd_eigen);
  // RotationMatrix<double> rot_y = RotationMatrix<double>::MakeYRotation(20 * 3.14 / 180.0);
  // orientation_d_ = (Rd * rot_y).ToQuaternion();
  orientation_d_ = Rd.ToQuaternion();

  // TODO: prototyping integrator term
  // if this works, will need to make this more principled
  // pass in integrator gains from constructor, use states instead of mutables, etc
  integrator_ = VectorXd::Zero(6);
  I_ = MatrixXd::Zero(6,6);
  I_.block(0,0,3,3) << param_.rotational_integral_gain * MatrixXd::Identity(3,3);
  I_.block(3,3,3,3) << param_.translational_integral_gain * MatrixXd::Identity(3,3);

  prev_time_ = 0.0;
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


  // TODO: parse out info here
  VectorXd state = c3_output->get_data();

  VectorXd xd = VectorXd::Zero(6);
  VectorXd xd_dot = VectorXd::Zero(6);
  VectorXd lambda = VectorXd::Zero(5); // does not contain the slack variable
  Vector3d ball_xyz_d(state(25), state(26), state(27));
  Vector3d ball_xyz(state(28), state(29), state(30));


  xd.tail(3) << state.head(3);
  Quaterniond orientation_d(state(3), state(4), state(5), state(6));
  xd_dot.tail(3) << state.segment(14, 3);
  lambda << state.segment(24, 5);

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
    plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("end_effector_tip"));
  const RotationMatrix<double> R = H.rotation();
  Vector3d d = H.translation() + R*EE_offset_;
  //std::cout << "position\n" << d << std::endl;

  // build task space state vectors
  VectorXd x = VectorXd::Zero(6);
  x.tail(3) << d;
  VectorXd x_dot = J_franka * v_franka;

  // compute xd_dot.head(3) angular velocity
  // RotationMatrix<double> R_d(orientation_d);
  // Eigen::AngleAxis<double> w_d_aa = (R.inverse() * R_d).ToAngleAxis();
  // std::cout <<  w_d_aa.angle() << std::endl << std::endl;
  // w_d_aa.angle() = 0.1 * w_d_aa.angle() / 70.0;



  // Quaterniond w_desired = RotationMatrix<double>(w_d_aa).ToQuaternion();

  VectorXd xtilde = xd - x;
  xtilde.head(3) << this->CalcRotationalError(R, orientation_d);
  // std::cout << xtilde << std::endl << std::endl;
  VectorXd xtilde_dot = xd_dot - x_dot;

  // double ang_speed = x_dot.head(3).norm();
  // Eigen::Vector3d axis = x_dot.head(3) / ang_speed;
  // if (ang_speed > 0.001){
  //   RotationMatrix<double> w_current(Eigen::AngleAxis<double>(ang_speed, axis));
  //   xtilde_dot.head(3) = this->CalcRotationalError(w_current, w_desired);
  // }
  // else{
  //   RotationMatrix<double> w_current(Eigen::AngleAxis<double>(0, Vector3d(1, 0, 0)));
  //   xtilde_dot.head(3) = this->CalcRotationalError(w_current, w_desired);
  // }

  MatrixXd M_inv = M_franka.inverse();
  MatrixXd Lambda = (J_franka * M_inv * J_franka.transpose()).inverse(); // apparent end effector mass matrix

  /// integral term
  VectorXd tau_int = VectorXd::Zero(7);
  if (timestamp > 1) {
    double dt = timestamp - prev_time_;
    VectorXd integrator_temp = integrator_;
    integrator_ += xtilde * dt;
    tau_int = J_franka.transpose() * Lambda * I_*integrator_;
    if (SaturatedClamp(tau_int, param_.integrator_clamp)){
      std::cout << "clamped integrator torque" << std::endl;
      integrator_ = integrator_temp;
      ClampIntegratorTorque(tau_int, param_.integrator_clamp);
    }
  }
  VectorXd tau = J_franka.transpose() * Lambda * (K_*xtilde + B_*xtilde_dot) + tau_int + C_franka;

  // add feedforward force term if contact is desired
  MatrixXd Jc(contact_pairs_.size() + 2 * contact_pairs_.size() * num_friction_directions_, n_);
  if (enable_contact_ && lambda.norm() > param_.contact_threshold){
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
  MatrixXd J_ginv_tranpose = (J_franka * M_inv * J_franka.transpose()).inverse() * J_franka * M_inv;
  MatrixXd N = MatrixXd::Identity(7, 7) - J_franka.transpose() * J_ginv_tranpose;
  //VectorXd qd = VectorXd::Zero(7);
  //qd << qd_.head(4), q(4), q(5), q(6);
  tau += N * (K_null_*(qd_-q_franka) - B_null_*v_franka);

//  this->CheckJointLimits(q.head(7), timestamp);
  this->ClampJointTorques(tau, timestamp);

  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
  prev_time_ = timestamp;

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
}

Vector3d ImpedanceController::CalcRotationalError(const RotationMatrix<double>& R,
    const Quaterniond& orientation_d) const {
  // compute rotational error, computational steps taken from
  // https://github.com/frankaemika/libfranka/blob/master/examples/cartesian_impedance_control.cpp
  Quaterniond orientation = R.ToQuaternion();

  if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0){
    orientation.coeffs() << -orientation.coeffs();
  }
  Quaterniond error_quaternion(orientation.inverse() * orientation_d);
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

void ImpedanceController::ClampJointTorques(VectorXd& tau, double timestamp) const {
  VectorXd thresholds = 0.95 * torque_limits_;
  std::cout << std::setprecision(3) << std::fixed;

  for (int i = 0; i < n_; i++){
    int sign = 1;
    if (tau(i) < 0) sign = -1;

    if (abs(tau(i)) > thresholds(i)){
//      std::cout << "[time: " << timestamp << "] " << "Joint "
//                << i+1 << "'s desired torque of " << tau(i)
//                << " is close to or above its torque limit of "
//                << torque_limits_(i) <<". This torque input has been clamped to "
//                << sign * thresholds(i) << "." << std::endl;
      tau(i) = sign * thresholds(i);
    }
  }
}

void ImpedanceController::ClampIntegratorTorque(VectorXd& tau, const VectorXd& clamp) const {
  for (int i = 0; i < tau.size(); i++){
    int sign = 1;
    if (tau(i) < 0) sign = -1;

    if (abs(tau(i)) > clamp(i)){
      tau(i) = sign * clamp(i);
    }
  }
}

bool ImpedanceController::SaturatedClamp(const VectorXd& tau, const VectorXd& clamp) const {
  for (int i = 0; i < tau.size(); i++){
    if (abs(tau(i)) > clamp(i)){
      return true;
    }
  }


  return false;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib