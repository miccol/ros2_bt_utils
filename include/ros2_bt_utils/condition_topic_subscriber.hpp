//
// Copyright (c) 2022 by Michele Colledanchise. All Rights Reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef ROS2_BT_UTILS_CONDITION_TOPIC_SUBSCRIBER_H
#define ROS2_BT_UTILS_CONDITION_TOPIC_SUBSCRIBER_H

#include <behaviortree_cpp_v3/condition_node.h>
#include <ros2_bt_utils/ros2_utils.h>

#include <atomic>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

namespace ros2_bt_utils
{

template<typename ROSTopicT>
class ConditionTopicSubscriber : public BT::ConditionNode
{
protected:
  using ROSTopicTPtr = typename ROSTopicT::SharedPtr;

  using ROSSTopicSubscription = typename rclcpp::Subscription<ROSTopicT>::SharedPtr;

public:
  ConditionTopicSubscriber(
    const std::string & name, const BT::NodeConfiguration & config,
    const std::string & topic_name)
  : BT::ConditionNode(name, config), topic_name_{topic_name}
  {
    ros_node_ = ros2_bt_utils::ROSNode();
    rclcpp::spin_some(ros_node_);
    server_attached_ = attachServer();
  }

  bool attachServer()
  {
    if (!ros_node_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Cannot attach Server. Condition %s Will fail",
        name().c_str());
      return false;
    }
    subscription_ = ros_node_->create_subscription<ROSTopicT>(
      topic_name_.c_str(), 10,
      std::bind(&ConditionTopicSubscriber::topic_callback, this, _1));
    return true;
  }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(ros_node_);

    if (!server_attached_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Subscriber non attached. Condition %s Failing",
        name().c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (has_message_) {
      return elaborateMessageAndReturn(last_message_);
    } else {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Message not published yet on the Topic %s. Condition %s returning the value of "
        "boundaryConditionStatus()",
        topic_name_.c_str(), name().c_str());
      return boundaryConditionStatus();
    }
  }

protected:
  virtual BT::NodeStatus elaborateMessageAndReturn(const ROSTopicT msg) = 0;
  virtual BT::NodeStatus boundaryConditionStatus() = 0;
  /* The BT node may be ticked before the message is written.
       In that case, the node returns the assumed BT::NodeStatus.
       If you need to wait for the actrual message to be written,
       this should not be done inside a condition node! */

private:
  const std::string topic_name_;
  bool server_attached_{false};
  rclcpp::Node::SharedPtr ros_node_;
  ROSTopicT last_message_;
  ROSSTopicSubscription subscription_;
  std::atomic<bool> has_message_{false};

  void topic_callback(const ROSTopicTPtr msg)
  {
    has_message_ = true;       // a better way to do that?
    last_message_ = *msg;
  }
};

}  // namespace ros2_bt_utils

#endif  // ROS2_BT_UTILS_CONDITION_TOPIC_SUBSCRIBER_H
