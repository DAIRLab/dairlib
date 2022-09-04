#include "dairlib/lcmt_saved_traj.hpp"
#include "lcm_trajectory_receiver.h"
#include "lcm/lcm_trajectory.h"

namespace dairlib::systems::controllers {

using Eigen::VectorXd;
using Eigen::MatrixXd;

using std::string;
using dairlib::lcmt_saved_traj;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::Context;
using drake::trajectories::Trajectory;


LcmTrajectoryReceiver::LcmTrajectoryReceiver(
    std::string traj_name, TrajectoryType traj_type)
    : traj_type_(traj_type), traj_name_(traj_name) {

  this->DeclareAbstractInputPort("lcmt_saved_traj", drake::Value<lcmt_saved_traj>{});

  PiecewisePolynomial<double> empty_traj(VectorXd::Zero(0));
  Trajectory<double>& traj_inst = empty_traj;
  this->DeclareAbstractOutputPort(
      traj_name, traj_inst, &LcmTrajectoryReceiver::MakeTrajFromLcm);

}

void LcmTrajectoryReceiver::MakeTrajFromLcm(const Context<double> &context,
                                            Trajectory<double> *traj) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<lcmt_saved_traj>();

  LcmTrajectory lcm_traj(input_msg);
  LcmTrajectory::Trajectory this_traj = lcm_traj.GetTrajectory(traj_name_);
  auto* casted_traj = dynamic_cast<PiecewisePolynomial<double>*>(traj);
  int n = this_traj.datapoints.rows() / 2;
  switch (traj_type_) {
    case kZOH:
      *casted_traj = PiecewisePolynomial<double>::ZeroOrderHold(
          this_traj.time_vector, this_traj.datapoints);
      break;
    case kFOH:
      *casted_traj = PiecewisePolynomial<double>::FirstOrderHold(
          this_traj.time_vector, this_traj.datapoints);
      break;
    case kCubicHermite:
      DRAKE_ASSERT(this_traj.datapoints.rows() % 2 == 0);
      *casted_traj = PiecewisePolynomial<double>::CubicHermite(
          this_traj.time_vector, this_traj.datapoints.topRows(n),
          this_traj.datapoints.bottomRows(n));
      break;
    case kCubicShapePreserving:
      *casted_traj = PiecewisePolynomial<double>::CubicShapePreserving(
          this_traj.time_vector, this_traj.datapoints);
      break;
    case kCubicConstAccel:
      throw std::logic_error("We don't have a sensible way to implement"
                             " this yet");
      break;
  }

}

}