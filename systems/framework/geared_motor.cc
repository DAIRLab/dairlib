#include "systems/framework/geared_motor.h"

#include <drake/common/eigen_types.h>

namespace dairlib {
namespace systems {

using drake::multibody::JointActuator;
using drake::multibody::MultibodyPlant;
using drake::systems::kUseDefaultName;

GearedMotor::GearedMotor(const MultibodyPlant<double>& plant,
                         const std::unordered_map<std::string, double>& max_motor_speeds)
    : n_q(plant.num_positions()),
      n_v(plant.num_velocities()),
      n_u(plant.num_actuators()),
      B_(plant.MakeActuationMatrix()) {
  for (int i = 0; i < n_u; ++i) {
    const JointActuator<double>& joint_actuator =
        plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));
    actuator_gear_ratios.push_back(joint_actuator.default_gear_ratio());
    actuator_ranges.push_back(joint_actuator.effort_limit() / joint_actuator.default_gear_ratio());
    actuator_max_speeds.push_back(max_motor_speeds.at(joint_actuator.name()));
  }
  systems::BasicVector<double> input(plant.num_actuators());
  systems::BasicVector<double> output(plant.num_actuators());

  command_input_port_ =
      this->DeclareVectorInputPort("u_cmd", input).get_index();
  state_input_port_ =
      this->DeclareVectorInputPort("x", BasicVector<double>(n_q + n_v))
          .get_index();
  this->DeclareVectorOutputPort("u_motor", output,
                                &GearedMotor::CalcTorqueOutput);
}

void GearedMotor::CalcTorqueOutput(
    const drake::systems::Context<double>& context,
    systems::BasicVector<double>* output) const {
  const systems::BasicVector<double>& u =
      *this->template EvalVectorInput<BasicVector>(context,
                                                   command_input_port_);
  const systems::BasicVector<double>& x =
      *this->template EvalVectorInput<BasicVector>(context, state_input_port_);
  Eigen::VectorXd actuator_velocities = B_.transpose() * x.value().tail(n_v);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(n_u);
  for (int i = 0; i < n_u; ++i) {
    double ratio = actuator_gear_ratios[i];
    double tmax = actuator_ranges[i];
    double w = actuator_velocities[i] * ratio;
    double wmax = actuator_max_speeds[i];

    // Calculate torque limit based on motor speed
    double tlim = 2 * tmax * (1 - fabs(w) / wmax);
    tlim = fmax(fmin(tlim, tmax), 0);

    // Compute motor-side torque
    tau[i] = copysign(fmin(fabs(u[i] / ratio), tlim), u[i]) * ratio;
  }
  output->SetFromVector(tau);
}

}  // namespace systems
}  // namespace dairlib
