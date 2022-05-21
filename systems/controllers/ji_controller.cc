#include "ji_controller.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"

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
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::vector;
using drake::multibody::JacobianWrtVariable;

// test functions for the joint and cartesian impedance controllers

// motion planning test for the joint impedance controller
// bends all joints to right angles
vector<VectorXd> compute_target_joint_space_vector(double t){
    VectorXd start = 0*VectorXd::Ones(7);
    start(3) = -0.0698;
    VectorXd end = 1.57*VectorXd::Ones(7);
    end(3) = -1.57;

    double start_time = 5.0;
    double duration = 5.0;
    double end_time = start_time+duration;

    if (t < start_time) {
        return {start, 0*VectorXd::Ones(7)};
    }
    else if (t > end_time){
        return {end, 0*VectorXd::Ones(7)};
    }    
    else {
        VectorXd v = (end-start) / duration;
        double a = (t-start_time) / duration;
        return {(1-a)*start + a*end, v};
    }
}

// motion planning test for the cartesian impedance controller
// traces a small horizontal circle
Vector3d compute_target_task_space_vector(double t){
    //return Vector3d(0.1, 0, 0.925);

    // tracks a cirle in task sapce
    double r = 0.125;
    double x_c = 0.5;
    double y_c = 0;
    double z_c = 0.4;
    double w = 1;
    Vector3d start(x_c+r, y_c, z_c);
    double start_time = 10.0;
    if (t < start_time){
      return start;
    }
    else{
      return Vector3d(x_c + r*cos(w*(t-start_time)), y_c + r*sin(w*(t-start_time)), z_c);
    }
}

// TODO: IK function (INCOMPLETE)
VectorXd inverse_kinematics(const drake::multibody::MultibodyPlant<double>& plant, 
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

JIController::JIController(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context,
    const MatrixXd& K, 
    const MatrixXd& B)
    : plant_(plant),
      context_(context),
      K_(K),
      B_(B){
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();

  // integral term for PID controller
  this->DeclareContinuousState(num_positions);

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &JIController::CalcControl)
                             .get_index();

  // define end effector
  EE_offset_ << 0, 0, 0;
  // TODO: check if these need to be updated everytime
  EE_frame_ = &plant_.GetBodyByName("panda_link8").body_frame();
  world_frame_ = &plant_.world_frame();
}

void JIController::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* control) const {


  /// get values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();
  int state_dim = plant_.num_positions() + plant_.num_velocities();
  VectorXd state(state_dim);
  state << robot_output->GetPositions(), robot_output->GetVelocities();
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

  // TASK SPACE CONTROLLER
  int flag = 0;
  if (timestamp >= 0) {
    // forward kinematics
    const drake::math::RigidTransform<double> H = 
      plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("panda_link8"));
    
    auto d = H.translation();
    auto R = H.rotation();

    Eigen::Matrix3d Rd_eigen; // TODO: check the type of the rotation matrix
    Rd_eigen << 
      1,  0,  0,
      0, -1,  0,
      0,  0, -1;
    RotationMatrix<double> Rd(Rd_eigen);
    
    VectorXd x = VectorXd::Ones(6);
    x.tail(3) << d; //  currently position (don't worry about orientation
                                    //  for the time being)
    
    // compute jacobian
    MatrixXd J(6, plant_.num_velocities());
    plant_.CalcJacobianSpatialVelocity(
        context_, JacobianWrtVariable::kV,
        *EE_frame_, EE_offset_,
        *world_frame_, *world_frame_, &J);
    VectorXd x_dot = J * v;
    
    // compute mass matrix
    //MatrixXd M = MatrixXd::Zero(plant_.num_velocities(), plant_.num_velocities()); 
    //plant_.CalcMassMatrix(context_, &M);

    // compute the control input, tau
    VectorXd tau = 0*VectorXd::Ones(7);
  
    // TODO: fix these awful constructors, make them more elegant
    VectorXd xd = 0*VectorXd::Ones(6);
    xd.tail(3) << compute_target_task_space_vector(timestamp);
    VectorXd xd_dot = 0*VectorXd::Ones(6); // TODO: set this to the actualy xd_dot
    VectorXd xtilde = xd - x;

    // compute rotational error
    Eigen::Quaterniond orientation = R.ToQuaternion();
    Eigen::Quaterniond orientation_d = Rd.ToQuaternion();

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    xtilde.head(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    xtilde.head(3) << R.matrix() * xtilde.head(3);

    VectorXd xtilde_dot = xd_dot - x_dot;

    // if (trunc(timestamp*10) / 10.0 == timestamp){
    //   std::cout << timestamp << "\n--------------\nR: \n" << R.matrix() << std::endl;
    //   std::cout << "error:\n" << rotation_error << std::endl;
    //    std::cout << "x_des:\n" << xd << std::endl;
    //    std::cout << "x_dot:\n" << x_dot << std::endl;
    // }

    // compute rotational components and add it to the error vectors


    // testing gains
    // parameter code taken directly from:
    // https://github.com/frankaemika/libfranka/blob/master/examples/cartesian_impedance_control.cpp
    
    // Compliance parameters
    double translational_stiffness = 125;
    double rotational_stiffness = 5;
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 1 * sqrt(rotational_stiffness) *
                                      Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 1 * sqrt(translational_stiffness) *
                                          Eigen::MatrixXd::Identity(3, 3);

    tau = J.transpose() * (stiffness*xtilde + damping*xtilde_dot) + C - tau_g;

    control->SetDataVector(tau);
    control->set_timestamp(timestamp);
  }

  // CODE FOR PID CONTROLLER
  else {
    // print position
    // const drake::math::RigidTransform<double> H = 
    // plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("panda_link8"));
    
    // auto d = H.translation();
    // //RotationMatrix<double> Rd(Rd_eigen);

    // // compute jacobian????
    // MatrixXd J(6, plant_.num_velocities());
    // plant_.CalcJacobianSpatialVelocity(
    //     context_, JacobianWrtVariable::kV,
    //     *EE_frame_, EE_offset_,
    //     *world_frame_, *world_frame_, &J);
    // VectorXd x_dot = J * v;

    // auto spatial_velocity = plant_.EvalBodySpatialVelocityInWorld(context_, plant_.GetBodyByName("panda_link8"));
    // VectorXd x_dot_drake = spatial_velocity.translational();
    
    // if (trunc(timestamp*10) / 10.0 == timestamp){
    //   std::cout << timestamp << "\n--------------\nd: \n" << d << std::endl;
    //   std::cout << "x_dot: \n" << x_dot << std::endl;
    //   std::cout << "x_dot_drake: \n" << x_dot_drake << std::endl;
    // }

    // compute the control input, tau
    VectorXd tau = 0*VectorXd::Ones(7);

    // arbitary target position
    vector<VectorXd> target = compute_target_joint_space_vector(timestamp);
    VectorXd qd = target[0];
    VectorXd qd_dot = target[1];

    // IK
    // Vector3d xd = compute_target_task_space_vector(timestamp);
    // VectorXd qd = inverse_kinematics(plant_, xd);
    // VectorXd qd_dot = VectorXd::Ones(7); // set joint velocities to 0 for now

    // integral term (hopefully)
    const VectorX<double>& integral =
        dynamic_cast<const BasicVector<double>&>(context.get_continuous_state_vector())
            .value();
    
  //   // scalar gains that work
  //   double Kp = 125;
  //   double Kd = 5;
  //   double Ki = 0; // no I term as per Brian's suggestion
    
    // generate gain matrices
    int num_joints = plant_.num_positions();
    MatrixXd Kp = MatrixXd::Zero(num_joints, num_joints);
    MatrixXd Kd = MatrixXd::Zero(num_joints, num_joints);
    MatrixXd Ki = MatrixXd::Zero(num_joints, num_joints);

    std::vector<double> P_gains = {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
    std::vector<double> D_gains = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
    std::vector<double> I_gains = {0, 0, 0, 0, 0, 0, 0};
    double ratio = 0.25;
    for (int i = 0; i < num_joints; i++){
        Kp(i,i) = P_gains[i]*ratio;
        Kd(i,i) = D_gains[i]*ratio;
        Ki(i,i) = I_gains[i]; // should just be 0
    }
    
    // not using integral term as per Brian's suggestion
    tau = Kp*(qd - q) + Kd*(qd_dot - v) + Ki * integral + C - tau_g;

    // TODO: add limited on tau?

    control->SetDataVector(tau);
    control->set_timestamp(timestamp);
  }
}


// FOR PID CONTROLLERS
void JIController::DoCalcTimeDerivatives(
    const Context<double>& context, drake::systems::ContinuousState<double>* derivatives) const {
  /// get values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  VectorXd q = robot_output->GetPositions();
  double timestamp = robot_output->get_timestamp();

  // The derivative of the continuous state is the instantaneous position error.
  drake::systems::VectorBase<double>& derivatives_vector = derivatives->get_mutable_vector();
  derivatives_vector.SetFromVector(compute_target_joint_space_vector(timestamp)[0] - q);
}


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib