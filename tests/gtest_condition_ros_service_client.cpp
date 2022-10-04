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

#include <example_interfaces/srv/add_two_ints.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_bt_utils/condition_ros_service_client.hpp>
#include <string>

class ConditionROSServiceClientTest : public ::testing::Test
{
   protected:
   std::shared_ptr<rclcpp::Node> node;
   void SetUp() override
   {
      node = std::make_shared<rclcpp::Node>("condition_ros_service_client_test");
      spinner = std::thread([&]() { rclcpp::spin(node); });
      spinner.detach();

      service_add = node->create_service<example_interfaces::srv::AddTwoInts>(
          "service_add", std::bind(&ConditionROSServiceClientTest::add, this, std::placeholders::_1,
                                   std::placeholders::_2));
      service_trigger = node->create_service<example_interfaces::srv::Trigger>(
          "service_trigger", std::bind(&ConditionROSServiceClientTest::trigger, this,
                                       std::placeholders::_1, std::placeholders::_2));
   }
   void TearDown() override
   {
      // spinner.join();
      // node.reset();
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
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "trigger");
   }
   rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_add;
   rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr service_trigger;
};

class ConditionTrigger
    : public ros2_bt_utils::ConditionROSServiceClient<example_interfaces::srv::Trigger>
{
   public:
   ConditionTrigger(const std::string &name, const BT::NodeConfiguration &config)
       : ConditionROSServiceClient(name, config, "service_trigger")
   {
   }
   BT::NodeStatus elaborateResponseAndReturn(
       [[maybe_unused]] const example_interfaces::srv::Trigger::Response::SharedPtr result) override
   {
      ++counter_;
      // returns success if odd
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

class ConditionAdd
    : public ros2_bt_utils::ConditionROSServiceClient<example_interfaces::srv::AddTwoInts>
{
   public:
   ConditionAdd(const std::string &name, const BT::NodeConfiguration &config)
       : ConditionROSServiceClient(name, config, "service_add")
   {
   }
   BT::NodeStatus elaborateResponseAndReturn(
       const example_interfaces::srv::AddTwoInts::Response::SharedPtr result) override
   {
      result_ = result.get()->sum;
      // retunr success if odd
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
   // std::shared_ptr<example_interfaces::srv::AddTwoInts_Request> request_;
};

TEST_F(ConditionROSServiceClientTest, TestTrigger)
{
   static const char *xml_tree_condition_trigger = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Condition ID="ConditionTrigger"/>
    </BehaviorTree>
</root>
 )";

   BT::BehaviorTreeFactory factory;
   factory.registerNodeType<ConditionTrigger>("ConditionTrigger");
   auto blackboard = BT::Blackboard::create();
   auto tree = factory.createTreeFromText(xml_tree_condition_trigger, blackboard);
   auto condition = dynamic_cast<ConditionTrigger *>(tree.rootNode());
   ASSERT_TRUE(condition != nullptr);  // make sure is correct
   ASSERT_EQ(condition->counter(), 0);
   ASSERT_EQ(condition->status(), BT::NodeStatus::IDLE);
   auto status = condition->executeTick();
   ASSERT_EQ(condition->counter(), 1);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
   status = condition->executeTick();
   ASSERT_EQ(condition->counter(), 2);
   ASSERT_EQ(status, BT::NodeStatus::FAILURE);
   status = condition->executeTick();
   ASSERT_EQ(condition->counter(), 3);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ConditionROSServiceClientTest, TestAdd)
{
   static const char *xml_tree_condition_add = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Condition ID="ConditionAdd"/>
    </BehaviorTree>
</root>
 )";

   BT::BehaviorTreeFactory factory;
   factory.registerNodeType<ConditionAdd>("ConditionAdd");
   auto blackboard = BT::Blackboard::create();
   auto tree = factory.createTreeFromText(xml_tree_condition_add, blackboard);
   auto condition = dynamic_cast<ConditionAdd *>(tree.rootNode());
   ASSERT_TRUE(condition != nullptr);  // make sure is correct
   ASSERT_EQ(condition->result(), -1);
   ASSERT_EQ(condition->status(), BT::NodeStatus::IDLE);
   condition->set_a_b(0, 0);
   auto status = condition->executeTick();
   ASSERT_EQ(condition->result(), 0);
   ASSERT_EQ(status, BT::NodeStatus::FAILURE);
   condition->set_a_b(0, 1);
   status = condition->executeTick();
   ASSERT_EQ(condition->result(), 1);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
   condition->set_a_b(2, 1);
   status = condition->executeTick();
   ASSERT_EQ(condition->result(), 3);
   ASSERT_EQ(status, BT::NodeStatus::SUCCESS);
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