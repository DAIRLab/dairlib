#include "drake_ros_conversions.h"

namespace dairlib {
namespace systems {

using drake::systems::LeafSystem;
using Eigen::VectorXd;

/*--------------------------------------------------------------------------*/
// methods implementation for TimestampedVectorToROS.
TimestampedVectorToROS::TimestampedVectorToROS(
    const std::string& topic_name, int num_elements, int queue_size) : 
    topic_name_(topic_name_), num_elements_(num_elements_), 
    queue_size_(queue_size) {
    
    input_port_ = this->DeclareVectorInputPort("input",
                                 TimestampedVector<double>(num_elements_));
    this->DeclareVectorOutputPort("output",
                                TimestampedVector<double>(num_elements_)),
                                &TimestampedVectorToROS::PublishMessage);
    publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(topic_name_, queue_size_);    
}


void TimestampedVectorToROS::PublishMessage(
    const drake::systems::Context<double>& context, TimestampedVector<double>* output) const{
    
    // pass input directly to output
    auto input = (TimestampedVector<double>*)this->EvalVectorInput(context, input_port_);
    VectorXd vector_data = input->get_data();
    output->SetDataVector(vector_data);
    output->set_timestamp(input->get_timestamp());

    // generate and publish message
    std_msgs::Float64MultiArray msg;
    msg.data.clear();
    for (int i = 0; i < num_elements_; i++){
        msg.data.push_back(vector_data[i]);
    }
    
    publisher_.publish(msg);
}

/*--------------------------------------------------------------------------*/
// methods implementation for ROSToOutputVector.
ROSToOutputVector::ROSToOutputVector(const std::string& topic_name, 
    int num_positions, int num_velocities, 
    int num_efforts, int queue_size) :
    topic_name_(topic_name), num_positions_(num_positions),
    num_velocities_(num_velocities), num_efforts_(num_efforts),
    queue_size_(queue_size) {

    this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(num_positions_, num_velocities_, num_efforts_),
      &ROSToOutputVector::CopyOutput);
    
    subscriber_ = nh_.subscribe(topic_name_, queue_size_, 
                                this->ConversionCallback);
}

void ROSToOutputVector::CopyOutput(const drake::systems::Context<double>& context,
    OutputVector<double>* output) const{
    
    output->SetPositions(output_.GetPositions());
    output->SetVelocities(output_.GetVelocities());
    output->SetEfforts(output_.GetEfforts());
    output->SetIMUAccelerations(output_.GetIMUAccelerations());
    output->set_timestamp(output_.get_timestamp());
}

// TODO: if compiler is not happy with this since it is not const function
// can maybe try making a lambda function for it in constructor
void ROSToOutputVector::ConversionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    // NOTE: position and velocity should already be appended with 0s 
    // for the ball states
    
    VectorXd positions = VectorXd::Zero(num_positions_);
    for (int i = 0; i < num_positions_; i++){
        positions(i) = msg->data[i];
    }
    VectorXd velocities = VectorXd::Zero(num_velocities_);
    for (int i = 0; i < num_velcoities_; i++){
        velocities(i) = msg->data[i+num_positions_];
    }
    VectorXd efforts = VectorXd::Zero(num_efforts_);
    for (int i = 0; i < num_efforts_; i++){
        efforts(i) = msg->data[i+num_positions_+num_velocities_];
    }
    double timestamp = msg->data[msg->data.size()-1];
    
    output_.SetPositions(positions);
    output_.SetVelocities(velocities);
    output_.SetEfforts(efforts);
    output_.SetIMUAccelerations(VectorXd::Zero(3));
    output_.set_timestamp(timestamp);
}

/*--------------------------------------------------------------------------*/
// methods implementation for ROSToTimestampedVector.

ROSToTimestampedVector::ROSToTimestampedVector(const std::string& topic_name, 
    int num_elements, int queue_size) :
    topic_name_(topic_name), num_elements_(num_elements),
    queue_size_(queue_size) {

    this->DeclareVectorOutputPort(
      "u, t",
      TimestampedVector<double>(num_elements_),
      &ROSToTimestampedVector::CopyOutput);
    
    subscriber_ = nh_.subscribe(topic_name_, queue_size_, 
                                this->ConversionCallback);
}


void ROSTimestampedVector::CopyOutput(const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const{
    
    output->SetDataVector(output_->get_data());
    output->set_timestamp(output->get_timestamp());
}

// TODO: if compiler is not happy with this since it is not const function
// can maybe try making a lambda function for it in constructor
void ROSToTimestampedVector::ConversionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){  
    VectorXd data = VectorXd::Zero(num_elements_);
    for (int i = 0; i < num_elements_; i++){
        data(i) = msg->data[i];
    }
    double timestamp = msg->data[msg->data.size()-1];
    
    output_.SetDataVector(data);
    output_.set_timestamp(timestamp);
}


}  // namespace systems
}  // namespace dairlib