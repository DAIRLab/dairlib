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
    double w = 1;
    Vector3d start(x_c+r, y_c, z_c);
    double start_time = 3.0;

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
              TimestampedVector<double>(25))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &ImpedanceController::CalcControl)
                             .get_index();

  // define end effector and contact
  EE_offset_ << 0, 0, 0.05;
  EE_frame_ = &plant_.GetBodyByName("panda_link8").body_frame();
  world_frame_ = &plant_.world_frame();
  contact_pairs_.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1])); // EE <-> Sphere
  n_ = 7;

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

  // TODO: uncomment to get info from port
  auto c3_output =
      (TimestampedVector<double>*) this->EvalVectorInput(context, c3_state_input_port_);
  VectorXd state = c3_output->get_data();
  VectorXd xd = VectorXd::Zero(6);
  VectorXd xd_dot = VectorXd::Zero(6);
  Vector3d ball_xyz(state(7), state(8), state(9));

  xd.tail(3) << state.head(3);
  xd_dot.tail(3) << state(10), state(11), state(12);
  VectorXd lambda = state.tail(5); // does not contain the slack variable

  // std::vector<Vector3d> target = compute_target_task_space_vector(timestamp);
  // VectorXd xd = VectorXd::Zero(6);
  // xd.tail(3) << target[0];
  // VectorXd xd_dot = VectorXd::Zero(6);
  // xd_dot.tail(3) << target[1];
  // VectorXd lambda = VectorXd::Zero(5);
  // if (timestamp > 10.0 && timestamp < 20.0){
  //   lambda << 100, 0, 0, 0, 0;
  // }

  bool in_contact = !isZeroVector(lambda,0.1);
  
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
  
  // modify desired state if no contact desired
  // if (lambda(0) < 0.000001){
  //   Vector3d ball_to_EE = (d-ball_xyz) / (d-ball_xyz).norm();
  //   Vector3d xd_new = xd.tail(3) + moving_offset_*ball_to_EE;
  //   xd.tail(3) << xd_new;
  //   //std::cout << "here" << std::endl;
  // }
//  else{

//
//  }

  if (lambda(0) > 0.001){
    //std::cout << "here" << std::endl;
    Vector3d ball_to_EE = (d-ball_xyz) / (d-ball_xyz).norm();
    Vector3d xd_new = xd.tail(3) + pushing_offset_*ball_to_EE;
    xd.tail(3) << xd_new;
    //std::cout << "here" << std::endl;
  }

//  int ts = round(timestamp);
//
//
//  if (ts % 3 == 0){
//
//    Vector3d ball_to_EE = (d-ball_xyz) / (d-ball_xyz).norm();
//    Vector3d xd_new = xd.tail(3) + moving_offset_*ball_to_EE;
//    xd.tail(3) << xd_new;
//
//  }



  // build task space state vectors
  VectorXd x = VectorXd::Zero(6);
  x.tail(3) << d;
  VectorXd x_dot = J_franka * v_franka;

  // compute position control input
  VectorXd xtilde = xd - x;
  xtilde.head(3) << this->CalcRotationalError(R);
  VectorXd xtilde_dot = xd_dot - x_dot;
  VectorXd tau = J_franka.transpose() * (K_*xtilde + B_*xtilde_dot) + C_franka - tau_g_franka;

  // add feedforward force term if contact is desired
  MatrixXd Jc(contact_pairs_.size() + 2 * contact_pairs_.size() * num_friction_directions_, n_);

  //std::cout << lambda << std::endl;

//  std::cout << "normal_des" << std::endl;
//
//  std::cout << lambda(0) << std::endl;

//  if (lambda(0) > 0.0001){
//    //std::cout << "here" << std::endl;
//    // compute contact jacobian
//    VectorXd phi(contact_pairs_.size());
//    MatrixXd J_n(contact_pairs_.size(), plant_.num_velocities());
//    MatrixXd J_t(2 * contact_pairs_.size() * num_friction_directions_, plant_.num_velocities());
//    this->CalcContactJacobians(contact_pairs_, phi, J_n, J_t);
//    Jc << J_n.block(0, 0, J_n.rows(), n_),
//          J_t.block(0, 0, J_t.rows(), n_);
//    //lambda(0) = lambda(0) + 10;
//    tau = tau - Jc.transpose() * lambda; // TODO: check if this should be +/-
//  }

  // compute nullspace projection
  MatrixXd M_inv = M_franka.inverse();
  MatrixXd J_ginv_tranpose = (J_franka * M_inv * J_franka.transpose()).inverse() * J_franka * M_inv;
  MatrixXd N = MatrixXd::Identity(7, 7) - J_franka.transpose() * J_ginv_tranpose;
  VectorXd tau_null = N * (K_null_*(qd_-q_franka) - B_null_*v_franka);

  control->SetDataVector(tau + tau_null);
  control->set_timestamp(timestamp);
  
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  // debug prints every 10th of a second
  int print_enabled = 0; // print flag
  if (print_enabled && trunc(timestamp*100) / 100.0 == timestamp && timestamp >= 9.0){
    std::cout << timestamp << "\n---------------" << std::endl;
    std::cout << "contact desired?\n" << in_contact << std::endl;
    std::cout << "relative_distance\n" << 100*(d-ball_xyz).norm() << std::endl;
    std::cout << "xd\n" << xd.tail(3) << std::endl;
    std::cout << "x\n" << d << std::endl;
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