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

  //std::cout <<"states" << state.size() << std::endl;

  VectorXd C(plant_.num_velocities());

  //update the context_
  plant_.SetPositions(&context_, q);
  plant_.SetVelocities(&context_, v);
  plant_.CalcBiasTerm(context_, &C);
  VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_);

  // compute the control input, tau
  VectorXd tau = 0*VectorXd::Ones(7);

  // arbitary target position
  VectorXd q_target = 0*VectorXd::Ones(7);

  // integral term (hopefully)
   const VectorX<double>& integral =
       dynamic_cast<const BasicVector<double>&>(context.get_continuous_state_vector())
           .value();

  double Kp = 125;
  double Kd = 5;
  double Ki = 2;
  tau = Kp*(q_target - q) + Kd*(-1.0*v) + Ki * integral + C - tau_g;

//   std::cout << "tau:\n" << tau << std::endl << std::endl;
//   std::cout << "integral:\n" << integral << std::endl << std::endl;
//   std::cout << "C:\n" << C << std::endl << std::endl;
//   std::cout << "g:\n" << tau_g << std::endl << std::endl;


  control->SetDataVector(tau);
  control->set_timestamp(timestamp);
}

void JIController::DoCalcTimeDerivatives(
    const Context<double>& context, drake::systems::ContinuousState<double>* derivatives) const {
  /// get values
  auto robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  VectorXd q = robot_output->GetPositions();

  // The derivative of the continuous state is the instantaneous position error.
  drake::systems::VectorBase<double>& derivatives_vector = derivatives->get_mutable_vector();
  derivatives_vector.SetFromVector(-1*q);
}


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib