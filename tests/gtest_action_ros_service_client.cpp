/*
 *   Copyright (c) 2022 Michele Colledanchise
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following actions:

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

#include <chrono>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/action_ros_service_client.hpp>
#include <string>
using namespace std::chrono_literals;

class ActionROSServiceClientTest : public ::testing::Test
{
   protected:
   std::shared_ptr<rclcpp::Node> node;
   void SetUp() override
   {
      node = std::make_shared<rclcpp::Node>("action_ros_service_client_test");
      spinner = std::thread([&]() { rclcpp::spin(node); });
      spinner.detach();
      service_add = node->create_service<example_interfaces::srv::AddTwoInts>(
          "service_add", std::bind(&ActionROSServiceClientTest::add, this, std::placeholders::_1,
                                   std::placeholders::_2));
      service_trigger = node->create_service<example_interfaces::srv::Trigger>(
          "service_trigger", std::bind(&ActionROSServiceClientTest::trigger, this,
                                       std::placeholders::_1, std::placeholders::_2));
   }
   void TearDown() override
   {
      // spinner.join();
      //  node.reset();
   }

   private:
   std::thread spinner;
   void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
   {
      response->sum = request->a + request->b;
   }

   void trigger(
       [[maybe_unused]] const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
       [[maybe_unused]] std::shared_ptr<example_interfaces::srv::Trigger::Response> response)
   {
      // does nothing
   }
   rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_add;
   rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_trigger;
};

class ActionTrigger : public ros2_bt_utils::ActionROSServiceClient<example_interfaces::srv::Trigger>
{
   public:
   ActionTrigger(const std::string &name, const BT::NodeConfiguration &config)
       : ActionROSServiceClient(name, config, "service_trigger")
   {
   }
   BT::NodeStatus elaborateResponseAndReturn(
       [[maybe_unused]] const example_interfaces::srv::Trigger::Response::SharedPtr result) override
   {
      ++counter_;
      // retunr success if odd
      return counter_ % 2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
   }
   example_interfaces::srv::Trigger::Request::SharedPtr computeRequest() override
   {
      auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
      return request;
   }

   static BT::PortsList providedPorts() { return {}; }
   int counter() const { return counter_; };

   private:
   int counter_{0};
};

class ActionAdd : public ros2_bt_utils::ActionROSServiceClient<example_interfaces::srv::AddTwoInts>
{
   public:
   ActionAdd(const std::string &name, const BT::NodeConfiguration &config)
       : ActionROSServiceClient(name, config, "service_add")
   {
   }
   BT::NodeStatus elaborateResponseAndReturn(
       const example_interfaces::srv::AddTwoInts::Response::SharedPtr result) override
   {
      result_ = result.get()->sum;

      // return success if odd
      return result_ % 2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
   }
   example_interfaces::srv::AddTwoInts::Request::SharedPtr computeRequest() override
   {
      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = a_;
      request->b = b_;
      return request;
   }

   static BT::PortsList providedPorts() { return {}; }
   int result() const { return result_; }
   void set_a_b(const int a, const int b)
   {
      a_ = a;
      b_ = b;
   }

   private:
   int result_{-1};
   int a_{0};
   int b_{0};
};

TEST_F(ActionROSServiceClientTest, TestTrigger)
{
   static const char *xml_tree_action_trigger = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Action ID="ActionTrigger"/>
    </BehaviorTree>
</root>
 )";

   BT::BehaviorTreeFactory factory;
   factory.registerNodeType<ActionTrigger>("ActionTrigger");
   auto blackboard = BT::Blackboard::create();
   auto tree = factory.createTreeFromText(xml_tree_action_trigger, blackboard);
   auto action = dynamic_cast<ActionTrigger *>(tree.rootNode());
   ASSERT_TRUE(action != nullptr);  // make sure is correct
   ASSERT_EQ(action->counter(), 0);
   ASSERT_EQ(action->status(), BT::NodeStatus::IDLE);

   auto status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->counter(), 1);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

   status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->counter(), 2);
   ASSERT_EQ(status, BT::NodeStatus::FAILURE);

   status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->counter(), 3);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ActionROSServiceClientTest, TestAdd)
{
   static const char *xml_tree_action_add = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Action ID="ActionAdd"/>
    </BehaviorTree>
</root>
 )";

   BT::BehaviorTreeFactory factory;
   factory.registerNodeType<ActionAdd>("ActionAdd");
   auto blackboard = BT::Blackboard::create();
   auto tree = factory.createTreeFromText(xml_tree_action_add, blackboard);
   auto action = dynamic_cast<ActionAdd *>(tree.rootNode());
   ASSERT_TRUE(action != nullptr);  // make sure is correct
   ASSERT_EQ(action->result(), -1);
   ASSERT_EQ(action->status(), BT::NodeStatus::IDLE);

   action->set_a_b(0, 0);
   auto status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->result(), 0);
   ASSERT_EQ(status, BT::NodeStatus::FAILURE);

   action->set_a_b(0, 1);
   status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      // RCLCPP_WARN(node->get_logger(), "ticking root %d", status);
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->result(), 1);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

   action->set_a_b(2, 1);
   status = tree.tickRoot();
   while (status == BT::NodeStatus::RUNNING)
   {
      // RCLCPP_WARN(node->get_logger(), "ticking root %d", status);
      status = tree.tickRoot();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
   }
   ASSERT_EQ(action->result(), 3);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
}

// int main(int argc, char **argv)
// {
//    testing::InitGoogleTest(&argc, argv);
//    rclcpp::init(argc, argv);
//    node = std::make_shared<rclcpp::Node>("action_topic_subscriber_test");
//    std::thread spinner([]() { rclcpp::spin(node); });
//    int result = RUN_ALL_TESTS();
//    rclcpp::shutdown();
//    spinner.join();
//    node.reset();
//    return result;
// }