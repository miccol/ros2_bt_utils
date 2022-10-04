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

#ifndef ROS2_BT_UTILS_ACTION_ROS_SERVICE_CLIENT_H
#define ROS2_BT_UTILS_ACTION_ROS_SERVICE_CLIENT_H

#include <behaviortree_cpp_v3/action_node.h>
#include <ros2_bt_utils/ros2_utils.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace ros2_bt_utils
{

   template <typename ROSServiceT>
   class ActionROSServiceClient : public BT::CoroActionNode
   {
  protected:
      using ServiceRequest = typename ROSServiceT::Request;
      using ServiceRequestPtr = typename ROSServiceT::Request::SharedPtr;
      using ServiceResponse = typename ROSServiceT::Response::SharedPtr;
      using ServiceClient = typename rclcpp::Client<ROSServiceT>::SharedPtr;

  public:
      ActionROSServiceClient(const std::string &name, const BT::NodeConfiguration &config,
                             const std::string &service_name)
          : BT::CoroActionNode(name, config), service_name_{service_name}
      {
         server_attached_ = attachServer();
      }

      BT::NodeStatus tick() override
      {
         //  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Ticked.");
         if (!server_attached_)
         {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Server non attached. Action Failing");
            return BT::NodeStatus::FAILURE;
         }

         if (!ros_node_)
         {
            return BT::NodeStatus::FAILURE;
         }
         std::chrono::milliseconds pause = std::chrono::milliseconds(50);

         auto result = service_client_->async_send_request(computeRequest());
         auto future_status = result.wait_for(pause);
         // Wait for the result.
         while (future_status != std::future_status::ready && rclcpp::ok())
         {
            RCLCPP_INFO(ros_node_->get_logger(), "Waiting for service %s", service_name_.c_str());
            future_status = result.wait_for(pause);
            spin_some(ros_node_);  // need to use spin_some to return the BT status of running
            setStatusRunningAndYield();
         }

         if (rclcpp::spin_until_future_complete(ros_node_, result) ==
             rclcpp::FutureReturnCode::SUCCESS)
         {
            return elaborateResponseAndReturn(result.get());
         }
         else
         {
            RCLCPP_ERROR(ros_node_->get_logger(), "Something went wrong in service %s",
                         service_name_.c_str());
         }
         return BT::NodeStatus::FAILURE;
      }

  private:
      bool attachServer()
      {
         ros_node_ = ros2_bt_utils::ROSNode();
         if (!ros_node_)
         {
            return false;
         }
         service_client_ = ros_node_->create_client<ROSServiceT>(service_name_.c_str());

         while (!service_client_->wait_for_service())
         {
            if (!rclcpp::ok())
            {
               RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                            "Interrupted while waiting for the service. Exiting.");
               return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
         }
         return true;
      }

  protected:
      virtual BT::NodeStatus elaborateResponseAndReturn(const ServiceResponse response) = 0;
      virtual ServiceRequestPtr computeRequest() = 0;

  private:
      std::string service_name_;
      bool server_attached_{false};
      ServiceClient service_client_;
      rclcpp::Node::SharedPtr ros_node_;  // static?
      ServiceRequest service_request_;
   };

}  // namespace ros2_bt_utils

#endif  // ROS2_BT_UTILS_ACTION_ROS_SERVICE_CLIENT_H
