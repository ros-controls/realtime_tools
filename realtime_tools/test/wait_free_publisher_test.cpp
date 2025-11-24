// Copyright (c) 2025, Brian Jin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Brian Jin

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <test_msgs/msg/empty.hpp>

#include <realtime_tools/wait_free_realtime_publisher.hpp>

class MockPublisher : public realtime_tools::PublisherInterface<test_msgs::msg::Empty>
{
public:
  MOCK_METHOD(void, publish, (const test_msgs::msg::Empty & msg), (override));
};

TEST(WaitFreeRealtimePublisherTests, PushAndPublish)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  test_msgs::msg::Empty msg;

  // Synchronization primitives to wait for publish call
  std::mutex mtx;
  std::condition_variable cv;
  bool published = false;

  EXPECT_CALL(*mock_publisher_ptr, publish(testing::Eq(msg)))
    .Times(1)
    .WillOnce(testing::Invoke([&]() {
      std::unique_lock<std::mutex> lock(mtx);
      published = true;
      cv.notify_one();
    }));

  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);

  ASSERT_TRUE(rt_pub.push(msg));

  // block until called
  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&published]() { return published; });
  }

  rt_pub.stop();
}

TEST(WaitFreeRealtimePublisherTests, Constructor)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);

  // Should be running after construction
  EXPECT_TRUE(rt_pub.running());
}

TEST(WaitFreeRealtimePublisherTests, Destructor)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);
  // Should destruct without blocking
}

TEST(WaitFreeRealtimePublisherTests, Start)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);

  // Call once
  rt_pub.start();
  EXPECT_TRUE(rt_pub.running());

  // Subsequent calls should have no effect
  rt_pub.start();
  EXPECT_TRUE(rt_pub.running());
}

TEST(WaitFreeRealtimePublisherTests, Stop)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);

  // Empty call should be ok
  rt_pub.stop();
  EXPECT_FALSE(rt_pub.running());

  // Call once
  rt_pub.start();

  // Regular stop should also be ok
  rt_pub.stop();
  EXPECT_FALSE(rt_pub.running());

  // Subsequent calls should have no effect
  rt_pub.stop();
  EXPECT_FALSE(rt_pub.running());
}
