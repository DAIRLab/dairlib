#include "fingertips_target_traj_demultiplexer.h"

namespace dairlib::systems {
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

FingertipsTargetTrajDemultiplexer::FingertipsTargetTrajDemultiplexer(
    drake::systems::Context<double>* context)
    : context_(context) {
  this->set_name("fingertips_target_demultiplexer");
  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd::Zero(9));
  input_traj_port_ = this->DeclareAbstractInputPort(
                             "fingertips_target_traj",
                             drake::Value<Trajectory<double>>(empty_pp_traj))
                         .get_index();

  PiecewisePolynomial<double> pp(Eigen::Vector3d::Zero());
  Trajectory<double>& traj_inst = pp;

  fingertip_0_target_traj_port_ =
      this->DeclareAbstractOutputPort(
              "fingertip_0_target_traj", traj_inst,
              &FingertipsTargetTrajDemultiplexer::CalcFingertip0TargetTraj)
          .get_index();

  fingertip_120_target_traj_port_ =
      this->DeclareAbstractOutputPort(
              "fingertip_120_target_traj", traj_inst,
              &FingertipsTargetTrajDemultiplexer::CalcFingertip120TargetTraj)
          .get_index();

  fingertip_240_target_traj_port_ =
      this->DeclareAbstractOutputPort(
              "fingertip_240_target_traj", traj_inst,
              &FingertipsTargetTrajDemultiplexer::CalcFingertip240TargetTraj)
          .get_index();
}

void FingertipsTargetTrajDemultiplexer::CopyToOutput(
    const drake::systems::Context<double>& context, const int start_index,
    const int size,
    drake::trajectories::Trajectory<double>* target_traj) const {
  const Trajectory<double>& input_traj =
      this->EvalAbstractInput(context, input_traj_port_)
          ->get_value<Trajectory<double>>();
  auto casted_input_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
      const PiecewisePolynomial<double>*>(&input_traj);
  auto casted_target_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          target_traj);
  *casted_target_traj = casted_input_traj.Block(start_index, 0, size, 1);
}

}  // namespace dairlib::systems
