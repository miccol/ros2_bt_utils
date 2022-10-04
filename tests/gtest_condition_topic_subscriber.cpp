/*
 *   Copyright (c) 2022 Michele Colledanchise
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class ConditionTopicSubscriberTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node;
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("action_ros_action_client_test");
    spinner = std::thread([&]() {rclcpp::spin(node);});
    spinner.detach();
    rclcpp::PublisherOptions po;
    po.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    bool_publisher = node->create_publisher<std_msgs::msg::Bool>("/topic_bool", 1);
    string_publisher = node->create_publisher<std_msgs::msg::String>("/topic_string", 1);
    int_publisher = node->create_publisher<std_msgs::msg::Int8>("/topic_int8", 1);
  }
  void TearDown() override
  {
    // spinner.join();
    //  node.reset();
  }

  void publish_bool(const bool value)
  {
    std_msgs::msg::Bool msg;
    msg.data = value;
    bool_publisher->publish(msg);
  }
  void publish_string(const std::string value)
  {
    std_msgs::msg::String msg;
    msg.data = value;
    string_publisher->publish(msg);
  }
  void publish_int8(const int8_t value)
  {
    std_msgs::msg::Int8 msg;
    msg.data = value;
    int_publisher->publish(msg);
  }

private:
  std::thread spinner;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr int_publisher;
};

class ConditionReadBool : public ros2_bt_utils::ConditionTopicSubscriber<std_msgs::msg::Bool>
{
public:
  ConditionReadBool(const std::string & name, const BT::NodeConfiguration & config)
  : ConditionTopicSubscriber(name, config, "/topic_bool")
  {
  }

  BT::NodeStatus elaborateMessageAndReturn(std_msgs::msg::Bool msg) override
  {
    // as simple as it may get
    return msg.data ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus boundaryConditionStatus() override
  {
    // if no message has arrived. I assume it is false
    return BT::NodeStatus::FAILURE;
  }

  static BT::PortsList providedPorts() {return {};}
};

class ConditionReadInt8 : public ros2_bt_utils::ConditionTopicSubscriber<std_msgs::msg::Int8>
{
public:
  ConditionReadInt8(const std::string & name, const BT::NodeConfiguration & config)
  : ConditionTopicSubscriber(name, config, "/topic_int8")
  {
  }

  BT::NodeStatus elaborateMessageAndReturn(std_msgs::msg::Int8 msg) override
  {
    read_value_ = msg.data;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus boundaryConditionStatus() override
  {
    // if no message has arrived. I assume it is false
    read_value_ = -1;
    return BT::NodeStatus::FAILURE;
  }

  int8_t read_value() const {return read_value_;}

  static BT::PortsList providedPorts() {return {};}

private:
  mutable std::atomic<int8_t> read_value_;
};

class ConditionReadString : public ros2_bt_utils::ConditionTopicSubscriber<std_msgs::msg::String>
{
public:
  ConditionReadString(const std::string & name, const BT::NodeConfiguration & config)
  : ConditionTopicSubscriber(name, config, "/topic_string")
  {
  }

  BT::NodeStatus elaborateMessageAndReturn(std_msgs::msg::String msg) override
  {
    const std::lock_guard<std::mutex> lock(read_value_mutex_);

    read_value_ = msg.data;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus boundaryConditionStatus() override
  {
    // if no message has arrived. I assume it is false
    const std::lock_guard<std::mutex> lock(read_value_mutex_);

    read_value_ = "boundary value";
    return BT::NodeStatus::FAILURE;
  }

  std::string read_value() const
  {
    const std::lock_guard<std::mutex> lock(read_value_mutex_);
    return read_value_;
  }

  static BT::PortsList providedPorts() {return {};}

private:
  std::string read_value_;
  mutable std::mutex read_value_mutex_;   // cannot atomic string
};

TEST_F(ConditionTopicSubscriberTest, TestBool)
{
  static const char * xml_tree_condition_bool =
    R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Condition ID="ConditionReadBool"/>
    </BehaviorTree>
</root>
 )";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ConditionReadBool>("ConditionReadBool");
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(xml_tree_condition_bool, blackboard);
  auto condition = dynamic_cast<ConditionReadBool *>(tree.rootNode());
  ASSERT_TRUE(condition != nullptr);   // make sure is correct
  auto status = condition->tick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
  publish_bool(true);
  status = condition->tick();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
  publish_bool(false);
  status = condition->tick();
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ConditionTopicSubscriberTest, TestString)
{
  static const char * xml_tree_condition_string =
    R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Condition ID="ConditionReadString"/>
    </BehaviorTree>
</root>
 )";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ConditionReadString>("ConditionReadString");
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(xml_tree_condition_string, blackboard);
  auto condition = dynamic_cast<ConditionReadString *>(tree.rootNode());
  ASSERT_TRUE(condition != nullptr);   // make sure is correct
  condition->tick();
  ASSERT_EQ(condition->read_value(), "boundary value");
  publish_string("I");
  condition->tick();
  ASSERT_EQ(condition->read_value(), "I");
  publish_string("am");
  condition->tick();
  ASSERT_EQ(condition->read_value(), "am");
  publish_string("batman");
  condition->tick();
  ASSERT_EQ(condition->read_value(), "batman");
  publish_string("no I am not");
  condition->tick();
  ASSERT_EQ(condition->read_value(), "no I am not");
}

TEST_F(ConditionTopicSubscriberTest, TestInt8)
{
  static const char * xml_tree_condition_int =
    R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Condition ID="ConditionReadInt8"/>
    </BehaviorTree>
</root>
 )";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ConditionReadInt8>("ConditionReadInt8");
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(xml_tree_condition_int, blackboard);
  auto condition = dynamic_cast<ConditionReadInt8 *>(tree.rootNode());
  ASSERT_TRUE(condition != nullptr);   // make sure is correct
  condition->tick();
  ASSERT_EQ(condition->read_value(), -1);
  publish_int8(0);
  condition->tick();
  ASSERT_EQ(condition->read_value(), 0);
  publish_int8(1);
  condition->tick();
  ASSERT_EQ(condition->read_value(), 1);
  publish_int8(-1);
  condition->tick();
  ASSERT_EQ(condition->read_value(), -1);
  publish_int8(100);
  condition->tick();
  ASSERT_EQ(condition->read_value(), 100);
}

// int main(int argc, char **argv)
// {
//    testing::InitGoogleTest(&argc, argv);
//    rclcpp::init(argc, argv);
//    node = std::make_shared<rclcpp::Node>("condition_topic_subscriber_test");
//    std::thread spinner([]() { rclcpp::spin(node); });
//    int result = RUN_ALL_TESTS();
//    rclcpp::shutdown();
//    spinner.join();
//    node.reset();
//    return result;
// }
