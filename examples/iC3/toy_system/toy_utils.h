#include "c3/systems/framework/c3_output.h"
#include <drake/common/yaml/yaml_io.h>
#include "examples/iC3/toy_system/toy_system_params.h"
#include "examples/iC3/iC3_options.h"

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include <iostream>

using drake::systems::BasicVector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib {

class C3Solution2Input : public drake::systems::LeafSystem<double> {
 public:
  explicit C3Solution2Input(const MultibodyPlant<double>& plant) : 
    plant_(plant),
    n_u_(plant.num_actuators()) {
    // Declare input port for C3 solutions.
    c3_solution_port_index_ =
        this->DeclareAbstractInputPort("c3_solution",
                                       drake::Value<c3::systems::C3Output::C3Solution>())
            .get_index();
    curr_obj_x_port_ =
        this->DeclareVectorInputPort("x", plant.num_positions() + plant.num_velocities())
            .get_index();
    // Declare output port for inputs.
    c3_input_port_index_ =
        this->DeclareVectorOutputPort("u", n_u_, &C3Solution2Input::GetC3Input)
            .get_index();
  }

  // Getter for the input port.
  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_index_);
  }

  const drake::systems::InputPort<double>& get_input_port_curr_x() const {
    return this->get_input_port(curr_obj_x_port_);
  }

  // Getter for the output port.
  const drake::systems::OutputPort<double>& get_output_port_c3_input() const {
    return this->get_output_port(c3_input_port_index_);
  }

 private:
  drake::systems::InputPortIndex c3_solution_port_index_;
  drake::systems::InputPortIndex curr_obj_x_port_;
  drake::systems::OutputPortIndex c3_input_port_index_;

  int n_u_;
  const MultibodyPlant<double>& plant_;

  // Compute the input from the C3 solution.
  void GetC3Input(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const {
    const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    const auto& sol = input->get_value<c3::systems::C3Output::C3Solution>();

    const BasicVector<double>* x_curr =
      (BasicVector<double>*)this->EvalVectorInput(context, curr_obj_x_port_);
    
    ToySystemParams toy_params =
        drake::yaml::LoadYamlFile<ToySystemParams>(
            "examples/iC3/toy_system/toy_system_params.yaml");
    iC3Options ic3_options =
        drake::yaml::LoadYamlFile<iC3Options>(
            "examples/iC3/toy_system/toy_ic3_options.yaml");

    double t = context.get_time();
    if (t < toy_params.time_to_wait) {
      if (t - std::round(t) == 0) {
        std::cout << "time: " << t << std::endl;
      }
      std::unique_ptr<drake::systems::Context<double>> plant_context = plant_.CreateDefaultContext();		
      auto& context_ref = *plant_context;  
      VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_ref);

      Eigen::VectorXd u_gravity = Eigen::VectorXd::Zero(5);
      u_gravity[2] = -(tau_g[2] + tau_g[10]); // Hard-coded cube + plate
      if (t < toy_params.time_to_wait) {
        u_gravity[4] = (0.13 * tau_g[10]); // Hard-coded cube + plate
      }
      output->SetFromVector(u_gravity);

    } else {
      output->get_mutable_value() = sol.u_sol_.col(0).cast<double>();
    }
  }
};

class C3Solution2InputHand : public drake::systems::LeafSystem<double> {
 public:
  explicit C3Solution2InputHand(const MultibodyPlant<double>& plant) : 
    plant_(plant),
    n_u_(plant.num_actuators()) {
    // Declare input port for C3 solutions.
    c3_solution_port_index_ =
        this->DeclareAbstractInputPort("c3_solution",
                                       drake::Value<c3::systems::C3Output::C3Solution>())
            .get_index();
    curr_obj_x_port_ =
        this->DeclareVectorInputPort("x", plant.num_positions() + plant.num_velocities())
            .get_index();
    // Declare output port for inputs.
    c3_input_port_index_ =
        this->DeclareVectorOutputPort("u", n_u_, &C3Solution2InputHand::GetC3Input)
            .get_index();
  }

  // Getter for the input port.
  const drake::systems::InputPort<double>& get_input_port_c3_solution() const {
    return this->get_input_port(c3_solution_port_index_);
  }

  const drake::systems::InputPort<double>& get_input_port_curr_x() const {
    return this->get_input_port(curr_obj_x_port_);
  }

  // Getter for the output port.
  const drake::systems::OutputPort<double>& get_output_port_c3_input() const {
    return this->get_output_port(c3_input_port_index_);
  }

 private:
  drake::systems::InputPortIndex c3_solution_port_index_;
  drake::systems::InputPortIndex curr_obj_x_port_;
  drake::systems::OutputPortIndex c3_input_port_index_;

  int n_u_;
  const MultibodyPlant<double>& plant_;

  // Compute the input from the C3 solution.
  void GetC3Input(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const {
    const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    const auto& sol = input->get_value<c3::systems::C3Output::C3Solution>();

    const BasicVector<double>* x_curr =
      (BasicVector<double>*)this->EvalVectorInput(context, curr_obj_x_port_);
    
    ToySystemParams toy_params =
        drake::yaml::LoadYamlFile<ToySystemParams>(
            "examples/iC3/toy_system/toy_system_hand_params.yaml");
    iC3Options ic3_options =
        drake::yaml::LoadYamlFile<iC3Options>(
            "examples/iC3/toy_system/toy_ic3_hand_options.yaml");

    double t = context.get_time();
    if (t < toy_params.time_to_wait) {
      if (t - std::round(t) == 0) {
        std::cout << "time: " << t << std::endl;
      }
      std::unique_ptr<drake::systems::Context<double>> plant_context = plant_.CreateDefaultContext();		
      auto& context_ref = *plant_context;  
      VectorXd tau_g = plant_.CalcGravityGeneralizedForces(context_ref);

      Eigen::VectorXd u_gravity = Eigen::VectorXd::Zero(9);
      u_gravity[2] = -tau_g[2]; // Hard-coded cube + plate
      u_gravity[5] = -tau_g[5]; // Hard-coded cube + plate
      u_gravity[8] = -tau_g[8]; // Hard-coded cube + plate

      output->SetFromVector(u_gravity);

    } else {
      output->get_mutable_value() = sol.u_sol_.col(0).cast<double>();
    }
  }
};

  // Converts a vector to a timestamped vector.
class Vector2TimestampedVector : public drake::systems::LeafSystem<double> {
public:
  explicit Vector2TimestampedVector(int n) {
    vector_port_index_ = this->DeclareVectorInputPort("state", n).get_index();
    timestamped_vector_port_index_ =
        this->DeclareVectorOutputPort("timestamped_state",
                                      systems::TimestampedVector<double>(n),
                                      &Vector2TimestampedVector::Convert)
            .get_index();
  }

  // Getter for the input port.
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(vector_port_index_);
  }

  // Getter for the output port.
  const drake::systems::OutputPort<double>& get_output_port_timestamped_state()
      const {
    return this->get_output_port(timestamped_vector_port_index_);
  }

 private:
  drake::systems::InputPortIndex vector_port_index_;
  drake::systems::OutputPortIndex timestamped_vector_port_index_;

  // Convert input vector to timestamped vector.
  void Convert(const drake::systems::Context<double>& context,
               systems::TimestampedVector<double>* output) const {
    const BasicVector<double>* state =
      (BasicVector<double>*)this->EvalVectorInput(context, vector_port_index_);

    output->SetDataVector(
        this->EvalVectorInput(context, vector_port_index_)->get_value());
    output->set_timestamp(context.get_time());  // Set timestamp.
  }
};

class PlateKinematics : public drake::systems::LeafSystem<double> {
public:
  explicit PlateKinematics(const MultibodyPlant<double>& plant) {
    x_plate_port_ =
      this->DeclareVectorInputPort(
              "x_plate", BasicVector<double>(5 + 5 + 5 + 3 + 1)) // Hard-coded (q, v, u, imu, time)
          .get_index();

    x_object_port_ =
      this->DeclareVectorInputPort(
              "x_object", BasicVector<double>(7 + 6 + 1)) // Hard-coded
          .get_index();

    state_output_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              BasicVector<double>(
                  5 + 7 + 6 + 5), // Hard-coded
              &PlateKinematics::CalcState)
          .get_index();
  }

  // Getter for the input port.
  const drake::systems::InputPort<double>& get_input_port_x_plate() const {
    return this->get_input_port(x_plate_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_x_object() const {
    return this->get_input_port(x_object_port_);
  }
  // Getter for the output port.
  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return this->get_output_port(state_output_port_);
  }

 private:
  drake::systems::InputPortIndex x_plate_port_;
  drake::systems::InputPortIndex x_object_port_;
  drake::systems::OutputPortIndex state_output_port_;

  // Convert input vector to timestamped vector.
  void CalcState(const drake::systems::Context<double>& context,
                BasicVector<double>* output) const {

    const BasicVector<double>* plate_state =
      (BasicVector<double>*)this->EvalVectorInput(context, x_plate_port_);
    const BasicVector<double>* object_state =
      (BasicVector<double>*)this->EvalVectorInput(context, x_object_port_);
    
    VectorXd plate_q = plate_state->get_value().head(5);
    VectorXd plate_v = plate_state->get_value().segment(5, 5);
    VectorXd object_q = object_state->get_value().head(7);
    VectorXd object_v = object_state->get_value().segment(7, 6);

    auto output_vec = output->get_mutable_value();
    output_vec.segment(0, 5) = plate_q;
    output_vec.segment(5, 7) = object_q;
    output_vec.segment(12, 5) = plate_v;
    output_vec.segment(17, 6) = object_v;

  }
};

class HandKinematics : public drake::systems::LeafSystem<double> {
public:
  explicit HandKinematics(const MultibodyPlant<double>& plant) {
    x_hand_port_ =
      this->DeclareVectorInputPort(
              "x_hand", BasicVector<double>(9 + 9 + 9 + 3 + 1)) // Hard-coded (q, v, u, imu, time)
          .get_index();

    x_object_port_ =
      this->DeclareVectorInputPort(
              "x_object", BasicVector<double>(7 + 6 + 1)) // Hard-coded (q, v, time)
          .get_index();

    state_output_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              BasicVector<double>(
                  9 + 7 + 9 + 6), // Hard-coded
              &HandKinematics::CalcState)
          .get_index();
  }

  // Getter for the input port.
  const drake::systems::InputPort<double>& get_input_port_x_hand() const {
    return this->get_input_port(x_hand_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_x_object() const {
    return this->get_input_port(x_object_port_);
  }
  // Getter for the output port.
  const drake::systems::OutputPort<double>& get_output_port_state() const {
    return this->get_output_port(state_output_port_);
  }

 private:
  drake::systems::InputPortIndex x_hand_port_;
  drake::systems::InputPortIndex x_object_port_;
  drake::systems::OutputPortIndex state_output_port_;

  // Convert input vector to timestamped vector.
  void CalcState(const drake::systems::Context<double>& context,
                BasicVector<double>* output) const {

    const BasicVector<double>* hand_state =
      (BasicVector<double>*)this->EvalVectorInput(context, x_hand_port_);
    const BasicVector<double>* object_state =
      (BasicVector<double>*)this->EvalVectorInput(context, x_object_port_);
    
    VectorXd hand_q = hand_state->get_value().head(9);
    VectorXd hand_v = hand_state->get_value().segment(9, 9);
    VectorXd object_q = object_state->get_value().head(7);
    VectorXd object_v = object_state->get_value().segment(7, 6);

    auto output_vec = output->get_mutable_value();
    output_vec.segment(0, 9) = hand_q;
    output_vec.segment(9, 7) = object_q;
    output_vec.segment(16, 9) = hand_v;
    output_vec.segment(25, 6) = object_v;

  }
};

} // namespace dairlib
