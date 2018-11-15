#include "cassie_udp_systems.h"
#include "dairlib/lcmt_robot_output.hpp"

namespace dairlib {

using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::systems::PublishEvent;
CassieUdpOutputPublisher::CassieUdpOutputPublisher(
    std::shared_ptr<CassieUdpSpoofer> spoofer) {
  _spoofer = spoofer;

  this->DeclareAbstractInputPort("lcmt_robot_output",
    drake::systems::Value<dairlib::lcmt_robot_output>{});
}

void CassieUdpOutputPublisher::set_publish_period(double period)
{
    LeafSystem<double>::DeclarePeriodicPublish(period);
}

void CassieUdpOutputPublisher::DoPublish(const Context<double>& context,
               const std::vector<const PublishEvent<double>*>&) const
{
    const drake::systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    const auto& input_msg = input->GetValue<dairlib::lcmt_robot_output>();
    cassie_dispatch_robot_out_t robot_out = CassieLcmInToRobotOut(input_msg);
    _spoofer->Publish(robot_out);
    std::cout << "Robot State Sent!" << std::endl;std::cout.flush();
}




CassieUdpInputSubscriber::CassieUdpInputSubscriber(std::shared_ptr<CassieUdpSpoofer> spoofer)
{
  spoofer->SetSubscriptionHandler([this](cassie_dispatch_robot_in_t robot_in) {this->ProcessInput(robot_in);});
  //_serializer = std::make_unique<Serializer<LcmMessage>>();
  DeclareAbstractOutputPort(lcmt_robot_input(),
        &CassieUdpInputSubscriber::Output);
  last_received = std::make_unique<lcmt_robot_input>();
  //this->DeclareAbstractState(last_received);
}

void CassieUdpInputSubscriber::ProcessInput(cassie_dispatch_robot_in_t new_input)
{
    std::cout << "Robot Input Receive Start!" << std::endl;std::cout.flush();
    //std::cout << "input subscriber" << std::endl; std::cout.flush();
    lcmt_robot_input new_robot_in = CassieRobotInToLcmOut(new_input);
    //std::cout << "conversion finished" << std::endl; std::cout.flush();
    this->mux.lock();
    this->last_received = std::make_unique<lcmt_robot_input>(new_robot_in);
    this->mux.unlock();
    std::cout << "Robot Input Received!" << std::endl;std::cout.flush();
    
}

void CassieUdpInputSubscriber::Output(const Context<double>& context, lcmt_robot_input * output) const
{
  this->mux.lock();
  *output = *(this->last_received.get());
  this->mux.unlock();
}

/*std::unique_ptr<systems::AbstractValue>
LcmSubscriberSystem::AllocateSerializerOutputValue() const {
  return _serializer->CreateDefaultValue();
}

void LcmSubscriberSystem::CalcSerializerOutputValue(const Context<double>& context, AbstractValue* output_value) const {
  DRAKE_DEMAND(serializer_.get() != nullptr);
  output_value->SetFrom(context.get_abstract_state().get_value(kStateIndexMessage));
}*/



}