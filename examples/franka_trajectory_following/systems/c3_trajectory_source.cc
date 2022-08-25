#include "c3_trajectory_source.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

C3TrajectorySource::C3TrajectorySource(const PiecewisePolynomial<double>& trajectory)
  : trajectory_(trajectory) {
  this->DeclareVectorInputPort("x, u, t", 
                                OutputVector<double>(14, 13, 7));
  this->DeclareVectorOutputPort("traj converter output",
                              TimestampedVector<double>(38),
                              &C3TrajectorySource::CalcOutput);
  derivative_ = trajectory_.derivative(1);
}

void C3TrajectorySource::CalcOutput(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const {

  auto input = (OutputVector<double>*)this->EvalVectorInput(context, 0);
  double timestamp = input->get_timestamp();

  bool set_zero = timestamp > trajectory_.end_time() || timestamp < trajectory_.start_time();
  VectorXd x = trajectory_.value(timestamp);

  VectorXd xdot = VectorXd::Zero(3);
  if (!set_zero){
    xdot << derivative_.value(timestamp);
  }

  VectorXd data = VectorXd::Zero(38);
  data.head(3) << x;
  data(4) = 1;
  data.segment(14, 3) << xdot;

  // std::cout << "desired trajectory position\n" << x << std::endl;
  // std::cout << "desired trajectory velocity\n" << xdot << std::endl;
  
  output->SetDataVector(data);
  output->set_timestamp(timestamp); // not used
}

}  // namespace systems
}  // namespace dairlib