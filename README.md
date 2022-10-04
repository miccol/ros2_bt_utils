

# Dependencies

- [ROS2 (Humble)](http://docs.ros.org/en/humble/)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)


# ROS2 BT Utils

Package containing the base classes (virtual and templates) to ease the development Behavior Trees leaf nodes in ROS2, using [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).
The classes contain the source code needed to set up the client side of ROS2's [Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html), [Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html), and subscribe to a [Topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html):

- `ActionROSActionClient(const std::string &name, const BT::NodeConfiguration &config,
                const std::string &ros_action_name)` (The base class of a BT action with name `name` and NodeConfiguration `config` that implements the client side of the ROS Action named `ros_action_name`)

- `ActionROSServiceClient(const std::string &name,
                       const BT::NodeConfiguration &config,
                       const std::string &service_name)` (The base class of a BT action with name `name` and NodeConfiguration `config` that implements the client side of the ROS Service named `service_name`)
- `ConditionTopicSubscriber(const std::string &name,
                       const BT::NodeConfiguration &config,
                       const std::string &topic_name)` (The base class of a BT condition with name `name` and NodeConfiguration `config` that reads the most updated message of from the topic `topic_name`)
- `ConditionROSServiceClient(const std::string &name,
                       const BT::NodeConfiguration &config,
                       const std::string &service_name)` (The base class of a BT condition with name `name` and NodeConfiguration `config` that implements the client side of the ROS Service named `service_name`)

## ActionROSActionClient
You just need to implement the following function:

```c++
YOURACTION::Goal computeGoal()
```

that computes the ROS Action Goal to be sent to the server. 

```c++
BT::NodeStatus elaborateResultAndReturn(const ClientGoalHandle<YOURACTION>::WrappedResult result);
```
That elaborates the results and computes the [BT::NodeStatus](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/b8fd0b2443f1171365b693387b9e4e3155384c3b/include/behaviortree_cpp_v3/basic_types.h#L35) to be sent to the BT parent node. The return status should be `SUCCESS` or `FAILURE`.

```c++
void elaborateFeedback(const TOURACTIONGOALFeedbackConstPtr feedback);
```
that is called at each  ROS Action `feedback` received.


Moreover, the BT Node returns `FAILURE` if the [Result Code](https://docs.ros2.org/latest/api/rclcpp_action/namespacerclcpp__action.html#ae469597b77e40287e19539b806a54619) of the ROS Action is `CANCELED` or `ABORTED`. It returns `RUNNING` otherwise (It actually implementa a `BT::CoroActionNode`, see [here](https://www.behaviortree.dev/tutorial_09_coroutines/) the details.).


Below a simple example of an BT Action Node class that reads a `geometry_msgs::msg::Pose` from the BT blackboard and sends it as a goal to the action named `navigate_to_pose` from [Nav2](https://navigation.ros.org/). It also writes the Estimated Time of Arrival (ETA):

header file:
```c++
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ros2_bt_utils/action_client.hpp>

using NavigateToPoseFeedback = typename rclcpp_action::ClientGoalHandle<
    nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr;

class ActionGoto : public ros2_bt_utils::ActionROSActionClient <nav2_msgs::action::NavigateToPose>
{
   public:
   ActionGoto(const std::string &name, const BT::NodeConfiguration &config);

   static BT::PortsList providedPorts()
   {
      return {BT::InputPort<geometry_msgs::msg::Pose>("pose")};
   }

   private:
   nav2_msgs::action::NavigateToPose::Goal computeGoal() override;
   void elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback) override;
   BT::NodeStatus elaborateResultAndReturn(
      const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result) override;
   rclcpp::Node::SharedPtr ros_node_; 
};
```
implementation file

```c++
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ros2_bt_utils/action_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using NavigateToPoseFeedback = typename rclcpp_action::ClientGoalHandle<
    nav2_msgs::action::NavigateToPose>::Feedback::ConstSharedPtr;

ActionGoto::ActionGoto(const std::string &name, const BT::NodeConfiguration &config)
    : ActionROSActionClient(name, config, "navigate_to_pose")
{
   RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

nav2_msgs::action::NavigateToPose::Goal ActionGoto::computeGoal()
{
   geometry_msgs::msg::Pose goal_pose;
   getInput("pose", goal_pose);  // get goa value from BT blackboard
   nav2_msgs::action::NavigateToPose::Goal goal;
   goal.pose.header.frame_id = "map";
   goal.pose.pose = goal_pose;
   return goal;
}

void ActionGoto::elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback)
{
   RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "ETA: %d",
               feedback.get()->estimated_time_remaining.sec);
}

BT::NodeStatus ActionGoto::elaborateResultAndReturn(
    [[maybe_unused]] const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult result)
{
      RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Goal Reached. BT Node returning Success");
   // NavigateToPose.action is std_msgs/Empty
   return BT::NodeStatus::SUCCESS;
}
```

## ActionROSServiceClient
You just need to implement the following functions:


```c++
YOURSERVICE::Request::SharedPtr computeRequest()
```

That computes the service request, and 

``` c++
BT::NodeStatus elaborateResponseAndReturn(const YOURSERVICE::Response response)
```
That takes the service response and returns the final return status of the BT node as [BT::NodeStatus](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/b8fd0b2443f1171365b693387b9e4e3155384c3b/include/behaviortree_cpp_v3/basic_types.h#L35)(i.e. `BT::SUCCESS` or `BT::FAILURE`).

## ConditionTopicSubscriber

You just need to implement the following function:

```c++
BT::NodeStatus elaborateMessageAndReturn(const YOURTOPICMESSAGE msg)
```

that takes a topic message and returns the return status of the BT node as [BT::NodeStatus](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/b8fd0b2443f1171365b693387b9e4e3155384c3b/include/behaviortree_cpp_v3/basic_types.h#L35)(i.e. `BT::SUCCESS` or `BT::FAILURE`). **However** it may the the case that the Condition Node gets ticked before the message is published. This should not. For this reason you need to implement the function:

```c++
BT::NodeStatus boundaryConditionStatus()
```
 whose return status will be used when the message `msg` is not published.


Below a simple example of a condition node class that reads the battery level from the topic `/battery` and compares it with  `reference_value` from the blackboard;

header 
```c++
#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

class ConditionBatteryLevelAbove
    : public ros2_bt_utils::ConditionTopicSubscriber<sensor_msgs::msg::BatteryState>
{
   public:
   ConditionBatteryLevelAbove(const std::string &name, const BT::NodeConfiguration &config);

   static BT::PortsList providedPorts() { return {BT::InputPort<float>("reference_value")}; }

   private:
   BT::NodeStatus elaborateMessageAndReturn(sensor_msgs::msg::BatteryState msg) override;
   BT::NodeStatus boundaryConditionStatus() override;
   
   double reference_value, current_value;
};
```


implementation file
```c++
#include <ros2_bt_utils/condition_topic_subscriber.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
ConditionBatteryLevelAbove::ConditionBatteryLevelAbove(const std::string &name,
                                                       const BT::NodeConfiguration &config)
    : ConditionTopicSubscriber(name, config, "/battery_state")
{
   RCLCPP_INFO(ros2_bt_utils::ROSNode()->get_logger(), "Creating %s", name.c_str());
}

BT::NodeStatus ConditionBatteryLevelAbove::elaborateMessageAndReturn(
    sensor_msgs::msg::BatteryState msg)
{
   current_value = msg.percentage;
   getInput("reference_value", reference_value);

   return current_value > reference_value ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus ConditionBatteryLevelAbove::boundaryConditionStatus()
{
   // if no message has arrived, assumes that the battery level is high enough
   RCLCPP_WARN(ros2_bt_utils::ROSNode()->get_logger(), "Assuming Battery level high ");
   return BT::NodeStatus::SUCCESS;
}
```


## ConditionROSServiceClient

You just need to implement the following functions:


```c++
  YOURSERVICE::Request::SharedPtr computeRequest()
```

That computes the service request, and 

``` c++
  BT::NodeStatus elaborateResponseAndReturn(const YOURSERVICE::Response response)
```
That takes the service response and returns the return status of the BT node as [BT::NodeStatus](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/b8fd0b2443f1171365b693387b9e4e3155384c3b/include/behaviortree_cpp_v3/basic_types.h#L35)(i.e. `SUCCESS` or `FAILURE`).
