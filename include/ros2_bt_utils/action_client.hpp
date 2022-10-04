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

#ifndef ROS2_BT_UTILS_ROSACTION_CLIENT_H
#define ROS2_BT_UTILS_ROSACTION_CLIENT_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros2_bt_utils/ros2_utils.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_bt_utils
{
   template <typename ROSActionT>
   /// @brief The base class of a BT action with name "name" and NodeConfiguration config that
   /// implements the client side of a ROS Action. Then the BT Node receives tick and halt, it sends
   /// the corresponding command to the ROS Action server. The derived class must implement the
   /// methods computeGoal and elaborateFeedback
   /// @tparam ROSActionT Template of the ROS Action
   class ActionROSActionClient : public BT::StatefulActionNode
   {
  protected:
      using ROSActionGoalWrappedResult =
          typename rclcpp_action::ClientGoalHandle<ROSActionT>::WrappedResult;

      using ROSActionClientPtr = typename rclcpp_action::Client<ROSActionT>::SharedPtr;
      using ROSActionGoal = typename ROSActionT::Goal;
      using ROSActionGoalFeedback = typename rclcpp_action::ClientGoalHandle<ROSActionT>::Feedback;
      using ROSActionGoalHandlePtr =
          typename rclcpp_action::ClientGoalHandle<ROSActionT>::SharedPtr;
      using ROSActionGoalFeedbackConstPtr =
          typename rclcpp_action::ClientGoalHandle<ROSActionT>::Feedback::ConstSharedPtr;

  public:
      ActionROSActionClient(const std::string &name, const BT::NodeConfiguration &config,
                            const std::string &ros_action_name)
          : BT::StatefulActionNode(name, config), ros_action_name_{ros_action_name}
      {
         ros_node_ = ros2_bt_utils::ROSNode();
         server_attached_ = attachServer();
      }

  protected:
        /// method invoked by the BT ath the first tick.
      BT::NodeStatus onStart() override
      {
         action_result_.code =
             rclcpp_action::ResultCode::UNKNOWN;  // reinitialize for future ticks.
         action_result_ = typename rclcpp_action::ClientGoalHandle<
             ROSActionT>::WrappedResult();  // reinitialize for future ticks.
         auto goal_options = typename rclcpp_action::Client<ROSActionT>::SendGoalOptions();
         goal_options.result_callback =
             std::bind(&ActionROSActionClient<ROSActionT>::resultCallback, this, _1);
         goal_options.goal_response_callback =
             std::bind(&ActionROSActionClient<ROSActionT>::responseCallback, this, _1);
         goal_options.feedback_callback =
             std::bind(&ActionROSActionClient<ROSActionT>::feedbackCallback, this, _1, _2);
         future_goal_handle_ = action_client_->async_send_goal(computeGoal(), goal_options);

         if (goal_rejected_)
         {
            RCLCPP_WARN(ros_node_->get_logger(),
                        "Goal was rejected by server but the BT is still ticking the action");
            return BT::NodeStatus::FAILURE;
         }
         return actionResultToStatus();
      }

      /// method invoked by the tick() while in the RUNNING state.
      BT::NodeStatus onRunning() override { return actionResultToStatus(); }

      void onHalted() override
      {
         RCLCPP_INFO(ros_node_->get_logger(), "Canceling goal");
         action_client_->async_cancel_goal(future_goal_handle_.get());
         setStatus(BT::NodeStatus::IDLE);
      }

  private:
      /// @brief checks if the result is available. It retunrs FAILURE if the ROS Action is CANCELED
      /// or ABORTED. If result is available it calls the virtual method elaborateResultAndReturn
      /// (user-defined)
      /// @return true if ok. false otherwise.
      BT::NodeStatus actionResultToStatus()
      {
         rclcpp::spin_some(ros_node_);
         std::lock_guard<std::mutex> guard(action_result_mutex_);
         if (action_result_.code == rclcpp_action::ResultCode::SUCCEEDED)

         {
            return elaborateResultAndReturn(action_result_);
         }
         else if (action_result_.code == rclcpp_action::ResultCode::ABORTED ||
                  action_result_.code == rclcpp_action::ResultCode::CANCELED)
         {
            RCLCPP_ERROR(ros_node_->get_logger(),
                         "Action canceled or aborted in the ROS Action Server %s",
                         ros_action_name_.c_str());
            return BT::NodeStatus::FAILURE;
         }

         return BT::NodeStatus::RUNNING;
      }

      /// @brief Attach the node to the ros action server
      /// @return true if ok. false otherwise.
      bool attachServer()
      {
         if (!ros_node_)
         {
            return false;
         }
         action_client_ =
             rclcpp_action::create_client<ROSActionT>(ros_node_, ros_action_name_.c_str());
         if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
         {
            auto node = ros2_bt_utils::ROSNode();
            RCLCPP_ERROR(ros_node_->get_logger(), "Timed out connecting to ROS Action Server %s",
                         ros_action_name_.c_str());
            return false;
         }
         return true;
      }

      void resultCallback(const ROSActionGoalWrappedResult &result)
      {
         std::lock_guard<std::mutex> guard(action_result_mutex_);
         action_result_ = result;
      }

      void responseCallback(ROSActionGoalHandlePtr future)
      {
         auto goal_handle = future.get();
         if (!goal_handle)
         {
            goal_rejected_ = true;
            RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
         }
         else
         {
            goal_rejected_ = false;
         }
      }

      void feedbackCallback(ROSActionGoalHandlePtr, ROSActionGoalFeedbackConstPtr feedback)
      {
         elaborateFeedback(feedback);
      }

  protected:
      virtual ROSActionGoal computeGoal() = 0;
      virtual void elaborateFeedback(const ROSActionGoalFeedbackConstPtr feedback) = 0;
      virtual BT::NodeStatus elaborateResultAndReturn(const ROSActionGoalWrappedResult result) = 0;

  private:
      bool server_attached_{false};
      std::string ros_action_name_;
      ROSActionClientPtr action_client_;
      rclcpp::Node::SharedPtr ros_node_;
      bool goal_rejected_{false};
      ROSActionGoalWrappedResult action_result_;
      std::mutex action_result_mutex_;  // I prefer mutex in this case so I can lock and check
                                        // the action_result_.code
      std::shared_future<ROSActionGoalHandlePtr> future_goal_handle_;
   };

}  // namespace ros2_bt_utils

#endif  // ROS2_BT_UTILS_ROSACTION_CLIENT_H