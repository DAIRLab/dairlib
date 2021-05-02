#include "systems/ros/ros2lcm_odometry.h"

namespace dairlib{
namespace systems{

RosOdometry2LcmSender::RosOdometry2LcmSender() {
    //Declare input for messages from ros_subscriber_system
    rosmsg_input_port_ = this->DeclareAbstractInputPort(
        "rosmsg_input_port", drake::Value<nav_msgs::Odometry>{}).get_index();
    lcm_output_port_ = this->DeclareAbstractOutputPort(
        "lcmt_odometry", &RosOdometry2LcmSender::CalcLcmFromRos).get_index();
}

using InPort = drake::systems::InputPort<double>;
const InPort& RosOdometry2LcmSender::get_rosmsg_input_port() const {
    return LeafSystem<double>::get_input_port(rosmsg_input_port_);
}

using OutPort = drake::systems::OutputPort<double>;
const OutPort& RosOdometry2LcmSender::get_lcmmsg_output_port() const {
    return LeafSystem<double>::get_output_port(lcm_output_port_);
}

void RosOdometry2LcmSender::CalcLcmFromRos(const Context<double> &context,
                                           lcmt_ros_odometry* output) const {

    const auto* const ros_odom_msg = this->EvalInputValue<nav_msgs::Odometry>(context, rosmsg_input_port_);

    lcmt_ros_odometry& lcm_odom_message = *output;
    lcm_odom_message.header.seq = ros_odom_msg->header.seq;
    std::cout << "seq: " << ros_odom_msg->header.seq << std::endl;
    lcm_odom_message.header.timestamp_sec = ros_odom_msg->header.stamp.sec;
    std::cout << "rosmsg time: " << ros_odom_msg->header.stamp.sec << std::endl;
    std::cout << "lcm stored time: " << lcm_odom_message.header.seq << std::endl;
    lcm_odom_message.header.timestamp_nsec = ros_odom_msg->header.stamp.nsec;
    lcm_odom_message.header.frame_id = ros_odom_msg->header.frame_id;

    lcm_odom_message.child_frame_id = ros_odom_msg->child_frame_id;

    lcm_odom_message.pose_point[0] = ros_odom_msg->pose.pose.position.x;
    lcm_odom_message.pose_point[1] = ros_odom_msg->pose.pose.position.y;
    lcm_odom_message.pose_point[2] = ros_odom_msg->pose.pose.position.z;

    lcm_odom_message.pose_quat[0] = ros_odom_msg->pose.pose.orientation.w;
    lcm_odom_message.pose_quat[1] = ros_odom_msg->pose.pose.orientation.x;
    lcm_odom_message.pose_quat[2] = ros_odom_msg->pose.pose.orientation.y;
    lcm_odom_message.pose_quat[3] = ros_odom_msg->pose.pose.orientation.z;

    lcm_odom_message.twist_linear[0] = ros_odom_msg->twist.twist.linear.x;
    lcm_odom_message.twist_linear[1] = ros_odom_msg->twist.twist.linear.y;
    lcm_odom_message.twist_linear[2] = ros_odom_msg->twist.twist.linear.z;

    lcm_odom_message.twist_angular[0] = ros_odom_msg->twist.twist.angular.x;
    lcm_odom_message.twist_angular[1] = ros_odom_msg->twist.twist.angular.y;
    lcm_odom_message.twist_angular[2] = ros_odom_msg->twist.twist.angular.z;
}

}
}
