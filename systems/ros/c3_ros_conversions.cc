#include "c3_ros_conversions.h"

namespace dairlib {
namespace systems {


TimestampedVectorToROS::TimestampedVectorToROS(int num_elements)
  : num_elements_(num_elements) {
  
  this->DeclareVectorInputPort("u, t",
                               TimestampedVector<double>(num_elements_));
  this->DeclareAbstractOutputPort("ROS Float64MultiArray",
                                  &TimestampedVectorToROS::ConvertToROS);
};

void TimestampedVectorToROS::ConvertToROS(const drake::systems::Context<double>& context,
                std_msgs::Float64MultiArray* output) const {
  
  const TimestampedVector<double>* input =
    (TimestampedVector<double>*)this->EvalVectorInput(context, 0);
  
  // note: this function currently appends the timestamp to the end of the output
  output->data.clear();
  for (int i = 0; i < num_elements_; i++){
    if (std::isnan(input->GetAtIndex(i))){
      output->data.push_back(0);
    }
    else{
      output->data.push_back(input->GetAtIndex(i));
    }
  }
  output->data.push_back(input->get_timestamp()); // TODO: figure out if 1e6 needed
}


}  // namespace systems
}  // namespace dairlib
