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
#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros2_bt_utils/action_client.hpp>
#include <string>
using namespace std::chrono_literals;

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class ActionROSActionClientTest : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node;
  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("action_ros_action_client_test");
    spinner = std::thread([&]() {rclcpp::spin(node);});
    spinner.detach();
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      node, "fibonacci",
      std::bind(
        &ActionROSActionClientTest::handle_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&ActionROSActionClientTest::handle_cancel, this, std::placeholders::_1),
      std::bind(&ActionROSActionClientTest::handle_accepted, this, std::placeholders::_1));
  }

  void TearDown() override
  {
    //  spinner.join();
    //  action_server_.reset();
    // node.reset();
  }

private:
  std::thread spinner;
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    [[maybe_unused]] std::shared_ptr<const Fibonacci::Goal> goal)
  {
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionROSActionClientTest::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;   // example interface different that action tutorial
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
    }
  }
};
class ActionFibonacci : public ros2_bt_utils::ActionROSActionClient<Fibonacci>
{
public:
  ActionFibonacci(const std::string & name, const BT::NodeConfiguration & config)
  : ActionROSActionClient(name, config, "fibonacci")
  {
  }
  void elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback) override
  {
    feedback_ = feedback.get()->sequence;
  }
  BT::NodeStatus elaborateResultAndReturn(
    const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult wrpd_result)
  {
    result_ = wrpd_result.result->sequence;
    return result_.size() % 2 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  Fibonacci::Goal computeGoal() override
  {
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = order_;
    return goal_msg;
  }

  static BT::PortsList providedPorts() {return {};}
  std::vector<int> result() const {return result_;}
  std::vector<int> feedback() const {return feedback_;}
  void set_order(const int order) {order_ = order;}

private:
  int order_{0};
  std::vector<int> result_{};
  std::vector<int> feedback_{};
};

TEST_F(ActionROSActionClientTest, TestAddAction)
{
  static const char * xml_tree_action_add =
    R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
            <Action ID="ActionFibonacci"/>
    </BehaviorTree>
</root>
 )";

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ActionFibonacci>("ActionFibonacci");
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(xml_tree_action_add, blackboard);
  auto action = dynamic_cast<ActionFibonacci *>(tree.rootNode());
  ASSERT_TRUE(action != nullptr);   // make sure is correct

  action->set_order(0);
  auto status = tree.tickRoot();
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(action->result().size(), (long unsigned int)2);   // avoid annoing warning
  ASSERT_EQ(action->result()[0], 0);
  ASSERT_EQ(action->result()[1], 1);
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  action->set_order(1);
  status = tree.tickRoot();
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_EQ(action->result().size(), (long unsigned int)2);
  ASSERT_EQ(action->result()[0], 0);
  ASSERT_EQ(action->result()[1], 1);
  ASSERT_EQ(status, BT::NodeStatus::FAILURE);

  action->set_order(4);
  status = tree.tickRoot();
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    switch (action->feedback().size()) {  // nothing fancy in tests
      case 1:
        ASSERT_EQ(action->feedback()[0], 0);
        break;
      case 2:
        ASSERT_EQ(action->feedback()[0], 0);
        ASSERT_EQ(action->feedback()[1], 1);
        break;
      case 3:
        ASSERT_EQ(action->feedback()[0], 0);
        ASSERT_EQ(action->feedback()[1], 1);
        ASSERT_EQ(action->feedback()[2], 1);
        break;
      case 4:
        ASSERT_EQ(action->feedback()[0], 0);
        ASSERT_EQ(action->feedback()[1], 1);
        ASSERT_EQ(action->feedback()[2], 1);
        ASSERT_EQ(action->feedback()[3], 2);
        break;

      default:
        break;
    }
  }
  ASSERT_EQ(action->result().size(), (long unsigned int)5);
  ASSERT_EQ(action->result()[0], 0);
  ASSERT_EQ(action->result()[1], 1);
  ASSERT_EQ(action->result()[2], 1);
  ASSERT_EQ(action->result()[3], 2);
  ASSERT_EQ(action->result()[4], 3);
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
