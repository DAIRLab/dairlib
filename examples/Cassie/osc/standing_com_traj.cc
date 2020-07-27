#include "examples/Cassie/osc/standing_com_traj.h"

#include <math.h>

using std::cout;
using std::endl;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;
using drake::multibody::JacobianWrtVariable;

namespace dairlib {
namespace cassie {
namespace osc {

StandingComTraj::StandingComTraj(
    const MultibodyPlant<double>& plant,
    const std::vector<std::pair<const Vector3d, const Frame<double>&>>&
        feet_contact_points,
    double height)
    : plant_(plant),
      world_(plant_.world_frame()),
      feet_contact_points_(feet_contact_points),
      height_(height) {
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort(OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("com_traj", traj_inst,
                                  &StandingComTraj::CalcDesiredTraj);
  // Create context
  context_ = plant_.CreateDefaultContext();
}

void StandingComTraj::CalcDesiredTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd x = robot_output->GetState();

  plant_.SetPositionsAndVelocities(context_.get(), x);

  // Get center of left/right feet contact points positions
  Vector3d contact_pos_sum = Vector3d::Zero();
  Vector3d contact_vel_sum = Vector3d::Zero();
  Vector3d position;
  MatrixXd J(3, plant_.num_velocities());
  for (const auto& point_and_frame : feet_contact_points_) {
    plant_.CalcPointsPositions(*context_, point_and_frame.second,
                               point_and_frame.first, world_, &position);
    contact_pos_sum += position;

    plant_.CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kV,
        point_and_frame.second, point_and_frame.first,
        world_, world_, &J);
    contact_vel_sum += J * x.tail(plant_.num_velocities());
  }
  Vector3d feet_center_pos = contact_pos_sum / 4;
  Vector3d desired_com_pos(feet_center_pos(0), feet_center_pos(1),
                           feet_center_pos(2) + height_);
  Vector3d desired_com_vel = contact_vel_sum / 4;

  double dt = 1;
  const std::vector<double> breaks = {context.get_time(),
                                      context.get_time() + dt};
  std::vector<MatrixXd> samples(breaks.size(), MatrixXd::Zero(3, 1));
  samples[0] = desired_com_pos;
  samples[1] = desired_com_pos + dt * desired_com_vel;

  // Assign traj
  PiecewisePolynomial<double>* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
