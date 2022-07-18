#include "c3_trajectory_converter.h"

namespace dairlib {
namespace systems {

using Eigen::VectorXd;

C3TrajectoryConverter::C3TrajectoryConverter(){
  this->DeclareVectorInputPort("traj converter input", BasicVector<double>(6));
  this->DeclareVectorOutputPort("traj converter output",
                              TimestampedVector<double>(34),
                              &C3TrajectoryConverter::Convert);
}

void C3TrajectoryConverter::Convert(const drake::systems::Context<double>& context,
                    TimestampedVector<double>* output) const {

  auto port_input = (BasicVector<double>*)this->EvalVectorInput(context, 0);
  auto input = port_input->value();

  VectorXd data = VectorXd::Zero(34);
  data.head(3) << input.head(3);
  data.block(10, 0, 3, 1) << input.tail(3);

  std::cout << "data\n" << input.head(3) << std::endl;

  output->SetDataVector(data);
  output->set_timestamp(0.0); // not used
}

}  // namespace systems
}  // namespace dairlib