// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
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

#include <gmock/gmock.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "test_msgs/action/fibonacci.hpp"

using test_msgs::action::Fibonacci;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
using realtime_tools::RealtimeServerGoalHandle;

const size_t ATTEMPTS = 10;
const std::chrono::milliseconds DELAY(250);

struct ActionServerCallbacks
{
  bool have_handle_ = false;
  std::shared_ptr<ServerGoalHandle> handle_;
  std::mutex mtx_;

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  void accepted_callback(std::shared_ptr<ServerGoalHandle> handle)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    handle_ = handle;
    have_handle_ = true;
  }

  rclcpp_action::CancelResponse cancel_callback(std::shared_ptr<ServerGoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  bool wait_for_handle(rclcpp::Node::SharedPtr node)
  {
    for (size_t i = 0; i < ATTEMPTS; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(DELAY);
      std::unique_lock<std::mutex> lock(mtx_);
      if (have_handle_) {
        break;
      }
    }
    std::unique_lock<std::mutex> lock(mtx_);
    return have_handle_;
  }
};

struct ActionClientCallbacks
{
  bool have_feedback_ = false;
  bool have_result_ = false;
  std::mutex mtx_;

  void feedback_callback(ClientGoalHandle::SharedPtr, const Fibonacci::Feedback::ConstSharedPtr &)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    have_feedback_ = true;
  }

  void result_callback(const ClientGoalHandle::WrappedResult &)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    have_result_ = true;
  }

  bool wait_for_feedback(rclcpp::Node::SharedPtr node)
  {
    for (size_t i = 0; i < ATTEMPTS; ++i) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(DELAY);
      std::unique_lock<std::mutex> lock(mtx_);
      if (have_feedback_) {
        break;
      }
    }
    std::unique_lock<std::mutex> lock(mtx_);
    return have_feedback_;
  }
};

std::shared_ptr<ClientGoalHandle> send_goal(
  rclcpp::Node::SharedPtr node, std::shared_ptr<rclcpp_action::Client<Fibonacci>> ac,
  const std::string & /*server_name*/, ActionClientCallbacks & client_callbacks)
{
  for (size_t i = 0; i < ATTEMPTS && !ac->action_server_is_ready(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  if (ac->action_server_is_ready()) {
    Fibonacci::Goal goal;
    goal.order = 10;
    // Create "SendGoalOptions" that has callbacks set
    rclcpp_action::Client<Fibonacci>::SendGoalOptions send_goal_options;
    send_goal_options.feedback_callback = std::bind(
      &ActionClientCallbacks::feedback_callback, &client_callbacks, std::placeholders::_1,
      std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ActionClientCallbacks::result_callback, &client_callbacks, std::placeholders::_1);
    // wait to get a client handle
    auto future_handle = ac->async_send_goal(goal, send_goal_options);
    rclcpp::spin_until_future_complete(node, future_handle);
    return future_handle.get();
  }
  return nullptr;
}

std::shared_ptr<ClientGoalHandle::WrappedResult> wait_for_result(
  rclcpp::Node::SharedPtr node, std::shared_ptr<ClientGoalHandle> client_goal_handle,
  std::shared_ptr<rclcpp_action::Client<Fibonacci>> client)
{
  // Get a result future
  auto result_future = client->async_get_result(client_goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);
  return std::make_shared<ClientGoalHandle::WrappedResult>(result_future.get());
}

rclcpp_action::Server<Fibonacci>::SharedPtr make_server(
  rclcpp::Node::SharedPtr node, const std::string & server_name, ActionServerCallbacks & callbacks)
{
  return rclcpp_action::create_server<Fibonacci>(
    node, server_name,
    std::bind(
      &ActionServerCallbacks::goal_callback, &callbacks, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&ActionServerCallbacks::cancel_callback, &callbacks, std::placeholders::_1),
    std::bind(&ActionServerCallbacks::accepted_callback, &callbacks, std::placeholders::_1));
}

TEST(RealtimeServerGoalHandle, set_aborted)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("set_aborted");
  ActionServerCallbacks callbacks;
  ActionClientCallbacks client_callbacks;
  const std::string server_name("~/client/set_aborted");
  auto as = make_server(node, server_name, callbacks);

  auto client = rclcpp_action::create_client<Fibonacci>(node, server_name);
  auto client_handle = send_goal(node, client, server_name, client_callbacks);
  ASSERT_NE(nullptr, client_handle.get());

  ASSERT_TRUE(callbacks.wait_for_handle(node));
  RealtimeServerGoalHandle<Fibonacci> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());
  rt_handle.execute();
  rt_handle.runNonRealtime();

  {
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {1, 1, 2, 3, 5};
    rt_handle.setAborted(result);
    rt_handle.runNonRealtime();
  }

  auto wrapped_result = wait_for_result(node, client_handle, client);
  ASSERT_NE(nullptr, wrapped_result.get());
  EXPECT_EQ(wrapped_result->code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_NE(nullptr, wrapped_result->result.get());
  EXPECT_EQ(5, wrapped_result->result->sequence.size());
  rclcpp::shutdown();
}

TEST(RealtimeServerGoalHandle, set_canceled)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("set_canceled");
  ActionServerCallbacks callbacks;
  ActionClientCallbacks client_callbacks;
  const std::string server_name("~/client/set_canceled");
  auto as = make_server(node, server_name, callbacks);

  auto client = rclcpp_action::create_client<Fibonacci>(node, server_name);
  auto client_handle = send_goal(node, client, server_name, client_callbacks);
  ASSERT_NE(nullptr, client_handle.get());

  ASSERT_TRUE(callbacks.wait_for_handle(node));
  RealtimeServerGoalHandle<Fibonacci> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());
  rt_handle.execute();
  rt_handle.runNonRealtime();

  // Cancel and wait for server to learn about that
  client->async_cancel_goal(client_handle);
  for (size_t i = 0; i < ATTEMPTS; ++i) {
    if (callbacks.handle_->is_canceling()) {
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }

  ASSERT_TRUE(callbacks.handle_->is_canceling());

  {
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {1, 1, 2, 3, 5};
    rt_handle.setCanceled(result);
    rt_handle.runNonRealtime();
  }

  auto wrapped_result = wait_for_result(node, client_handle, client);
  ASSERT_NE(nullptr, wrapped_result.get());
  EXPECT_EQ(wrapped_result->code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_NE(nullptr, wrapped_result->result.get());
  EXPECT_EQ(5, wrapped_result->result->sequence.size());
  rclcpp::shutdown();
}

TEST(RealtimeServerGoalHandle, set_succeeded)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("set_succeeded");
  ActionServerCallbacks callbacks;
  ActionClientCallbacks client_callbacks;
  const std::string server_name("~/client/set_succeeded");
  auto as = make_server(node, server_name, callbacks);

  auto client = rclcpp_action::create_client<Fibonacci>(node, server_name);
  auto client_handle = send_goal(node, client, server_name, client_callbacks);
  ASSERT_NE(nullptr, client_handle.get());

  ASSERT_TRUE(callbacks.wait_for_handle(node));
  RealtimeServerGoalHandle<Fibonacci> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());
  rt_handle.execute();
  rt_handle.runNonRealtime();

  {
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {1, 1, 2, 3, 5};
    rt_handle.setSucceeded(result);
    rt_handle.runNonRealtime();
  }

  auto wrapped_result = wait_for_result(node, client_handle, client);
  ASSERT_NE(nullptr, wrapped_result.get());
  EXPECT_EQ(wrapped_result->code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_NE(nullptr, wrapped_result->result.get());
  EXPECT_EQ(5, wrapped_result->result->sequence.size());
  rclcpp::shutdown();
}

TEST(RealtimeServerGoalHandle, send_feedback)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("send_feedback");
  ActionServerCallbacks callbacks;
  ActionClientCallbacks client_callbacks;
  const std::string server_name("~/client/send_feedback");
  auto as = make_server(node, server_name, callbacks);

  auto client = rclcpp_action::create_client<Fibonacci>(node, server_name);
  auto client_handle = send_goal(node, client, server_name, client_callbacks);
  ASSERT_NE(nullptr, client_handle.get());

  ASSERT_TRUE(callbacks.wait_for_handle(node));
  RealtimeServerGoalHandle<Fibonacci> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());
  rt_handle.execute();
  rt_handle.runNonRealtime();

  {
    auto fb = std::make_shared<Fibonacci::Feedback>();
    rt_handle.setFeedback(fb);
    rt_handle.runNonRealtime();
  }

  EXPECT_TRUE(client_callbacks.wait_for_feedback(node));
  rclcpp::shutdown();
}
