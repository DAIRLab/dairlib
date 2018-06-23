#include "cassie_udp_systems.h"

namespace dairlib {


CassieUdpOutputPublisher::CassieUdpOutputPublisher(std::shared_ptr<CassieUdpSpoofer> spoofer)
{
    _spoofer = spoofer;
    this->DeclareAbstractInputPort();
}

void CassieUdpOutputPublisher::set_publish_period(double period)
{
    LeafSystem<double>::DeclarePeriodicPublish(period);
}

void CassieUdpOutputPublisher::DoPublish(const Context<double>& context,
               const std::vector<const systems::PublishEvent<double>*>&) const
{
    const drake::systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    const auto& input_msg = input->GetValue<dairlib::lcmt_robot_output>();
    cassie_dispatch_robot_out_t robot_out = CassieLcmInToRobotOut(input_msg);
    _spoofer->Publish(robot_out);
}




CassieUdpInputSubscriber::CassieUdpInputReceiver(std::shared_ptr<CassieUdpSpoofer> spoofer)
{
  spoofer.SetSubscriptionHandler([this](cassie_dispatch_robot_in_t robot_in){this->ProcessInput(robot_in);});
  this->DeclareAbstractOutputPort(&CassieUdpInputSubscriber::Output);
}

void CassieUdpInputSubscriber::ProcessInput(cassie_dispatch_robot_in_t new_input)
{
    lcmt_robot_input lcm_robot_in = CassieRobotInToLcmOut(new_input);
    this->last_received_input.store(lcm_robot_in, std::memory_order_relaxed);
}

void CassieUdpInputSubscriber::Output(const Context<double>& context, lcmt_robot_input * output) const
{
  *output = this->last_received_input.fetch(std::memory_order_relaxed);
}


}