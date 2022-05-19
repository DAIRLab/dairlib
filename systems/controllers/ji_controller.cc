#include "ji_controller.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"


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
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using std::vector;
using drake::multibody::JacobianWrtVariable;

// really really jank spline function
vector<VectorXd> compute_target_vector(double t){
    VectorXd start = 0*VectorXd::Ones(7);
    start(3) = -0.0698;
    VectorXd end = 1.57*VectorXd::Ones(7);
    end(3) = -1.57;

    double start_time = 5.0;
    double duration = 10.0;
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
  
/*
  // TASK SPACE CONTROLLER
  // forward kinematics
  // TODO: is this the right EE?
  const drake::math::RigidTransform<double> H = 
    plant_.EvalBodyPoseInWorld(context_, plant_.GetBodyByName("panda_link8"));
  
  auto p = H.translation();
  //std::cout << "p: " << p << std::endl;

  VectorXd x = VectorXd::Ones(6);
  x << p(1), p(2), p(3), 0, 0, 0; //  currently position (don't worr about orientation
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
  xd << -0.43, 0.60, 0.31, 0, 0, 0; // desired position is roughly straight up
  VectorXd xd_dot = 0*VectorXd::Ones(6);
  VectorXd xtilde = xd - x;
  VectorXd xtilde_dot = xd_dot - x_dot;

  // testing gains
  MatrixXd K = MatrixXd::Zero(6, 6);
  K(0,0) = 10; K(1,1) = 10; K(2,2) = 15;
  MatrixXd B = MatrixXd::Zero(6, 6);
  B(0,0) = 5; B(1,1) = 5; B(2,2) = 5;
  
  //double K = 10;
  //double B = 5; 

  tau = J.transpose() * (K*xtilde + B*xtilde_dot) + C - tau_g;

  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
*/

  // CODE FOR PID CONTROLLER

  // compute the control input, tau
  VectorXd tau = 0*VectorXd::Ones(7);

  // arbitary target position
  vector<VectorXd> target = compute_target_vector(timestamp);

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
  tau = Kp*(target[0] - q) + Kd*(target[1] - v) + Ki * integral + C - tau_g;

  // TODO: add limited on tau?

  control->SetDataVector(tau);
  control->set_timestamp(timestamp);

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
  derivatives_vector.SetFromVector(compute_target_vector(timestamp)[0] - q);
}


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib