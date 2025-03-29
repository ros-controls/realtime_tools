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
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "test_msgs/msg/strings.hpp"

using StringMsg = test_msgs::msg::Strings;
using realtime_tools::RealtimePublisher;

TEST(RealtimePublisher, construct_destruct) { RealtimePublisher<StringMsg> rt_pub; }

struct StringCallback
{
  StringMsg msg_;
  std::mutex mtx_;

  void callback(const StringMsg::SharedPtr msg)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    msg_ = *msg;
  }
};

TEST(RealtimePublisher, rt_publish)
{
  rclcpp::init(0, nullptr);
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  const char * expected_msg = "Hello World";
  auto node = std::make_shared<rclcpp::Node>("construct_move_destruct");
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();
  auto pub = node->create_publisher<StringMsg>("~/rt_publish", qos);
  RealtimePublisher<StringMsg> rt_pub(pub);
  // publish a latched message
  bool lock_is_held = rt_pub.trylock();
  for (size_t i = 0; i < ATTEMPTS && !lock_is_held; ++i) {
    lock_is_held = rt_pub.trylock();
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(lock_is_held);
  rt_pub.msg_.string_value = expected_msg;
  rt_pub.unlockAndPublish();

  // make sure subscriber gets it
  StringCallback str_callback;

  auto sub = node->create_subscription<StringMsg>(
    "~/rt_publish", qos,
    std::bind(&StringCallback::callback, &str_callback, std::placeholders::_1));
  for (size_t i = 0; i < ATTEMPTS && str_callback.msg_.string_value.empty(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  EXPECT_STREQ(expected_msg, str_callback.msg_.string_value.c_str());
  rclcpp::shutdown();
}

TEST(RealtimePublisher, rt_try_publish)
{
  rclcpp::init(0, nullptr);
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  const char * expected_msg = "Hello World";
  auto node = std::make_shared<rclcpp::Node>("construct_move_destruct");
  rclcpp::QoS qos(10);
  qos.reliable().transient_local();
  auto pub = node->create_publisher<StringMsg>("~/rt_publish", qos);
  RealtimePublisher<StringMsg> rt_pub(pub);

  // try publish a latched message
  bool publish_success = false;
  for (std::size_t i = 0; i < ATTEMPTS; ++i) {
    StringMsg msg;
    msg.string_value = expected_msg;

    if (rt_pub.tryPublish(msg)) {
      publish_success = true;
      break;
    }
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(publish_success);

  // make sure subscriber gets it
  StringCallback str_callback;

  auto sub = node->create_subscription<StringMsg>(
    "~/rt_publish", qos,
    std::bind(&StringCallback::callback, &str_callback, std::placeholders::_1));
  for (size_t i = 0; i < ATTEMPTS && str_callback.msg_.string_value.empty(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(DELAY);
  }
  EXPECT_STREQ(expected_msg, str_callback.msg_.string_value.c_str());
  rclcpp::shutdown();
}
