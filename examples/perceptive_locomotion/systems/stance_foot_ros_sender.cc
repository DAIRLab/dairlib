#include "stance_foot_ros_sender.h"

#include <utility>

namespace dairlib::perceptive_locomotion {

StanceFootRosSender::StanceFootRosSender(
    std::unordered_map<int,std::string> fsm_to_stance_frame_id_map) :
    fsm_to_stance_frame_id_map_(std::move(fsm_to_stance_frame_id_map)) {
  this->DeclareVectorInputPort("fsm", 1);
  this->DeclareAbstractOutputPort(
      "stance_foot_frame_id", &StanceFootRosSender::CopyFrame);
}

void StanceFootRosSender::CopyFrame(
    const drake::systems::Context<double> &context,
    std_msgs::String *msg) const {
  int fsm = static_cast<int>(this->EvalVectorInput(context, 0)->get_value()(0));
  msg->data = fsm_to_stance_frame_id_map_.at(fsm);
}

}