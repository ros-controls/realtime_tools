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
#include <future>
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

TEST(RealtimePublisher, rt_can_try_publish)
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
  ASSERT_TRUE(rt_pub.can_publish());

  // try publish a latched message
  bool publish_success = false;
  for (std::size_t i = 0; i < ATTEMPTS; ++i) {
    StringMsg msg;
    msg.string_value = expected_msg;

    if (rt_pub.can_publish()) {
      ASSERT_TRUE(rt_pub.try_publish(msg));
      publish_success = true;
    }
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(publish_success);

  // make sure subscriber gets it
  StringCallback str_callback;

  auto sub = node->create_subscription<StringMsg>(
    "~/rt_publish", qos,
    std::bind(&StringCallback::callback, &str_callback, std::placeholders::_1));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  for (size_t i = 0; i < ATTEMPTS && str_callback.msg_.string_value.empty(); ++i) {
    exec.spin_some();
    std::this_thread::sleep_for(DELAY);
  }
  EXPECT_STREQ(expected_msg, str_callback.msg_.string_value.c_str());
  rclcpp::shutdown();
}

class RealtimePublisherTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("rt_test_node");

    exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);

    spin_thread_ = std::thread([this]() { exec_->spin(); });
  }

  void TearDown() override {
    if (exec_) {
      exec_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_ = nullptr;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_ = nullptr;
  std::thread spin_thread_;
  const std::string TOPIC_NAME_ = "/rt_publish";
  const std::chrono::duration<int64_t> WAIT_ON_PUBLISH_TIMEOUT_ = std::chrono::seconds(1);
};

TEST_F(RealtimePublisherTest, node_based_init_works)
{
  rclcpp::QoS qos = rclcpp::QoS(10);
  qos.reliable().transient_local();
  RealtimePublisher<StringMsg> rt_pub(node_, TOPIC_NAME_, qos);
  ASSERT_TRUE(rt_pub.can_publish());
}

TEST_F(RealtimePublisherTest, publish_latched_message)
{
  const char * expected_msg = "Hello World";
  StringMsg msg;
  msg.string_value = expected_msg;
  rclcpp::QoS qos = rclcpp::QoS(10);
  qos.reliable().transient_local();
  RealtimePublisher<StringMsg> rt_pub(node_, TOPIC_NAME_, qos);
  
  ASSERT_TRUE(rt_pub.can_publish());
  ASSERT_TRUE(rt_pub.try_publish(msg));

  // create another node and subscriber to verify message received
  StringCallback str_callback;
  std::promise<StringMsg> promise;
  std::future<StringMsg> future = promise.get_future();
  auto node = std::make_shared<rclcpp::Node>("rt_msg_sub_node");
  auto sub = node->create_subscription<StringMsg>(
    TOPIC_NAME_, qos,
    [&promise](const StringMsg::SharedPtr incoming_msg) {
      promise.set_value(*incoming_msg);
    }
  );

  exec_->add_node(node);

  auto start = std::chrono::steady_clock::now();
  while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - start > WAIT_ON_PUBLISH_TIMEOUT_) {
      FAIL() << "Timed out while waiting for latched message";
    }
  }
  // Validate received message
  auto received = future.get();
  EXPECT_STREQ(expected_msg, received.string_value.c_str());
}
