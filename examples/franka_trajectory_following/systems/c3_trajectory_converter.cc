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
  data.segment(10, 3) << input.tail(3);

  std::cout << "converter time\n" << context.get_time() << std::endl;

  output->SetDataVector(data);
  output->set_timestamp(context.get_time()); // not used
}

}  // namespace systems
}  // namespace dairlib