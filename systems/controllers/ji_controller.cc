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

// really really jank spline function
vector<VectorXd> compute_target_vector(double t){
    VectorXd start = 0*VectorXd::Ones(7);
    start(3) = -0.0698;
    VectorXd end = 1.57*VectorXd::Ones(7);
    end(3) = -1.57;

    double start_time = 5.0;
    double duration = 20.0;
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
  // this->DeclareContinuousState(num_positions);

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(num_positions, num_velocities, num_inputs))
          .get_index();

  control_output_port_ = this->DeclareVectorOutputPort(
                                 "u, t", TimestampedVector<double>(num_inputs),
                                 &JIController::CalcControl)
                             .get_index();

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

  VectorXd C(plant_.num_velocities());

  //update the context_
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

/*
  // CODE FOR PID CONTROLLER
  // compute the control input, tau
  VectorXd tau = 0*VectorXd::Ones(7);

  // arbitary target position
  vector<VectorXd> target = compute_target_vector(timestamp);

  // integral term (hopefully)
   const VectorX<double>& integral =
       dynamic_cast<const BasicVector<double>&>(context.get_continuous_state_vector())
           .value();

  double Kp = 125;
  double Kd = 5;
  double Ki = 2;
  tau = Kp*(target[0] - q) + Kd*(target[1]-v) + Ki * integral + C - tau_g;
*/
  
  // compute the control input, tau
  VectorXd tau = 0*VectorXd::Ones(7);  

  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
}

/*
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
*/

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib