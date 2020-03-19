// A rewrite of Drake's kuka_torque_controller updated with a MultiBodyPlant.
#include <utility>

#include <vector>
#include "drake/systems/controllers/inverse_dynamics.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/multibody/plant/multibody_plant.h"

#include "examples/kuka_iiwa_arm/kuka_torque_controller.h"

namespace dairlib {
namespace systems {

 using drake::systems::kVectorValued;
 using drake::systems::Adder;
 using drake::systems::BasicVector;
 using drake::systems::Context;
 using drake::systems::DiagramBuilder;
 using drake::systems::LeafSystem;
 using drake::systems::PassThrough;
 using drake::systems::controllers::InverseDynamics;
 using drake::systems::controllers::PidController;
 using drake::multibody::MultibodyPlant;
 using drake::VectorX;

template <typename T>
class StateDependentDamper : public LeafSystem<T> {
 public:
  StateDependentDamper(const MultibodyPlant<T>& plant,
                       const VectorX<double>& stiffness,
                       const VectorX<double>& damping_ratio)
      : plant_(plant), stiffness_(stiffness), damping_ratio_(damping_ratio) {
    const int num_q = plant_.num_positions();
    const int num_v = plant_.num_velocities();
    const int num_x = num_q + num_v;

    DRAKE_DEMAND(stiffness.size() == num_v);
    DRAKE_DEMAND(damping_ratio.size() == num_v);

    this->DeclareInputPort(kVectorValued, num_x);
    this->DeclareVectorOutputPort(BasicVector<T>(num_v),
                                  &StateDependentDamper<T>::CalcTorque);
  }

 private:
  const MultibodyPlant<T>& plant_;
  const VectorX<double> stiffness_;
  const VectorX<double> damping_ratio_;

  /**
   * Computes joint level damping forces by computing the damping ratio for each
   * joint independently as if all other joints were fixed. Note that the
   * effective inertia of a joint, when all of the other joints are fixed, is
   * given by the corresponding diagonal entry of the mass matrix. The critical
   * damping gain for the i-th joint is given by 2*sqrt(M(i,i)*stiffness(i)).
   */
  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const {
    Eigen::VectorXd x = this->EvalVectorInput(context, 0)->get_value();
    Eigen::VectorXd q = x.head(plant_.num_positions());
    Eigen::VectorXd v = x.tail(plant_.num_velocities());

    // Compute critical damping gains and scale by damping ratio. Use Eigen
    // arrays (rather than matrices) for elementwise multiplication.
    Eigen::MatrixXd H(plant_.num_positions(), plant_.num_positions());
    std::unique_ptr<Context<double>> plant_context = plant_.CreateDefaultContext();

    plant_.CalcMassMatrixViaInverseDynamics(*plant_context.get(), &H);
    Eigen::ArrayXd temp = H.diagonal().array() * stiffness_.array();
    Eigen::ArrayXd damping_gains = 2 * temp.sqrt();
    damping_gains *= damping_ratio_.array();

    // Compute damping torque.
    torque->get_mutable_value() = -(damping_gains * v.array()).matrix();
  }
};


template <typename T>
void KukaTorqueController<T>::SetUp(const VectorX<double>& stiffness,
                                    const VectorX<double>& damping_ratio) {
  DiagramBuilder<T> builder;
  const MultibodyPlant<T>& plant = *robot_for_control_;
  DRAKE_DEMAND(plant.num_positions() == stiffness.size());
  DRAKE_DEMAND(plant.num_positions() == damping_ratio.size());

  const int dim = plant.num_positions();

  /*
  torque_in ----------------------------
                                       |
  (q, v)    ------>|Gravity Comp|------+---> torque_out
               |                      /|
               --->|Damping|---------- |
               |                       |
               --->| Virtual Spring |---
  (q*, v*)  ------>|PID with kd=ki=0|
  */

  // Redirects estimated state input into PID and gravity compensation.
  auto pass_through = builder.template AddSystem<PassThrough<T>>(2 * dim);


  // Add gravity compensator.
  auto gravity_comp = builder.template AddSystem<InverseDynamics<T>>(
      &plant, InverseDynamics<T>::kGravityCompensation);

  // Adds virtual springs.
  Eigen::VectorXd kd(dim);
  Eigen::VectorXd ki(dim);
  Eigen::VectorXd kp(dim);
  kd.setZero();
  ki.setZero();
  kp.setZero();
  auto spring = builder.template AddSystem<PidController<T>>(kp, kd, ki);

  // Adds virtual damper.
  auto damper = builder.template AddSystem<StateDependentDamper<T>>(
      plant, stiffness, damping_ratio);

  // Adds an adder to sum the gravity compensation, spring, damper, and
  // feedforward torque.
  auto adder = builder.template AddSystem<Adder<T>>(4, dim);

  // Connects the estimated state to the gravity compensator.
  builder.Connect(pass_through->get_output_port(),
                  gravity_comp->get_input_port_estimated_state());

  // Connects the estimated state to the spring.
  builder.Connect(pass_through->get_output_port(),
                  spring->get_input_port_estimated_state());

  // Connects the estimated state to the damper.
  builder.Connect(pass_through->get_output_port(),
                  damper->get_input_port(0));

  // Connects the gravity compensation, spring, and damper torques to the adder.
  builder.Connect(gravity_comp->get_output_port_force(), adder->get_input_port(1));
  builder.Connect(spring->get_output_port(0), adder->get_input_port(2));
  builder.Connect(damper->get_output_port(0), adder->get_input_port(3));

  // Exposes the estimated state port.
  input_port_index_estimated_state_ =
      builder.ExportInput(pass_through->get_input_port());

  // Exposes the desired state port.
  input_port_index_desired_state_ =
      builder.ExportInput(spring->get_input_port_desired_state());

  // Exposes the commanded torque port.
  input_port_index_commanded_torque_ =
      builder.ExportInput(adder->get_input_port(0));

  // Exposes controller output.
  output_port_index_control_ = builder.ExportOutput(adder->get_output_port());
  builder.BuildInto(this);
}

template <typename T>
KukaTorqueController<T>::KukaTorqueController(
    std::unique_ptr<drake::multibody::MultibodyPlant<T>> plant, const VectorX<double>& stiffness,
    const VectorX<double>& damping) {
  robot_for_control_ = std::move(plant);
  SetUp(stiffness, damping);
}


template class dairlib::systems::KukaTorqueController<double>;

} // namespace systems
} // namespace dairlib
