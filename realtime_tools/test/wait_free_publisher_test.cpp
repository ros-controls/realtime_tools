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
#include <iostream>
#include <memory>
#include <mutex>
#include <test_msgs/msg/empty.hpp>

#include <realtime_tools/wait_free_realtime_publisher.hpp>

class MockPublisher : public realtime_tools::PublisherInterface<test_msgs::msg::Empty>
{
public:
  MOCK_METHOD(void, publish, (const test_msgs::msg::Empty & msg), (override));
};

class PublishTest : public ::testing::TestWithParam<int>
{
};

TEST_P(PublishTest, PushAndPublish)
{
  auto mock_publisher_ptr = std::make_shared<MockPublisher>();
  test_msgs::msg::Empty msg;

  const int expected_publish_calls = GetParam();

  // Synchronization primitives to wait for publish call
  std::mutex mtx;
  std::condition_variable test_cv;
  std::condition_variable publish_cv;
  int publish_count{0};

  EXPECT_CALL(*mock_publisher_ptr, publish(testing::Eq(msg)))
    .Times(testing::Exactly(expected_publish_calls))
    .WillRepeatedly(testing::Invoke([&]() {
      std::unique_lock<std::mutex> lock(mtx);
      publish_count++;
      if (publish_count >= expected_publish_calls) {
        test_cv.notify_one();
      }
      publish_cv.notify_one();
    }));

  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher_ptr);

  for (int i = 0; i < expected_publish_calls; ++i) {
    ASSERT_TRUE(rt_pub.push(msg));
    std::unique_lock<std::mutex> lock(mtx);
    publish_cv.wait(lock);
  }

  // block until called
  {
    std::unique_lock<std::mutex> lock(mtx);
    test_cv.wait(lock, [&publish_count, expected_publish_calls]() {
      return publish_count >= expected_publish_calls;
    });
  }

  std::cout << "All expected publish calls received: " << publish_count << std::endl;

  rt_pub.stop();
}

INSTANTIATE_TEST_SUITE_P(WaitFreeRealtimePublisherTests, PublishTest, ::testing::Values(1, 5, 10));

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
