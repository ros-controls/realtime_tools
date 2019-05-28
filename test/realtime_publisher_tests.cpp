/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gmock/gmock.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/String.h>
#include <chrono>
#include <mutex>
#include <thread>

using realtime_tools::RealtimePublisher;

TEST(RealtimePublisher, construct_destruct)
{
  ros::NodeHandle nh;
  RealtimePublisher<std_msgs::String> rt_pub(nh, "construct_destruct", 10);
}

TEST(RealtimePublisher, construct_init_destruct)
{
  RealtimePublisher<std_msgs::String> rt_pub;
  ros::NodeHandle nh;
  rt_pub.init(nh, "construct_init_destruct", 10);
}

struct StringCallback
{
  std_msgs::String msg_;
  std::mutex mtx_;

  void callback(const std_msgs::String & msg)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    msg_ = msg;
  }
};

TEST(RealtimePublisher, rt_publish)
{
  const size_t ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(250);

  const char * expected_msg = "Hello World";
  ros::NodeHandle nh;
  const bool latching = true;
  RealtimePublisher<std_msgs::String> rt_pub(nh, "rt_publish", 10, latching);
  // publish a latched message
  bool lock_is_held = rt_pub.trylock();
  for (size_t i = 0; i < ATTEMPTS && !lock_is_held; ++i)
  {
    lock_is_held = rt_pub.trylock();
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_TRUE(lock_is_held);
  rt_pub.msg_.data = expected_msg;
  rt_pub.unlockAndPublish();

  // make sure subscriber gets it
  StringCallback str_callback;

  ros::Subscriber sub = nh.subscribe("rt_publish", 10, &StringCallback::callback, &str_callback);
  for (size_t i = 0; i < ATTEMPTS && str_callback.msg_.data.empty(); ++i)
  {
    ros::spinOnce();
    std::this_thread::sleep_for(DELAY);
  }
  EXPECT_STREQ(expected_msg, str_callback.msg_.data.c_str());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "realtime_publisher_tests");
  return RUN_ALL_TESTS();
}
