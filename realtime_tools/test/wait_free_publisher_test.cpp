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
#include <memory>
#include <test_msgs/msg/empty.hpp>

#include <realtime_tools/wait_free_realtime_publisher.hpp>

class MockPublisher : public realtime_tools::PublisherInterface<test_msgs::msg::Empty>
{
public:
  MOCK_METHOD(void, publish, (const test_msgs::msg::Empty & msg), (override));
};

TEST(WaitFreeRealtimePublisherTests, push_and_publish)
{
  auto mock_publisher = std::make_shared<MockPublisher>();

  realtime_tools::WaitFreeRealtimePublisher<test_msgs::msg::Empty> rt_pub(mock_publisher);

  test_msgs::msg::Empty msg;

  ASSERT_TRUE(rt_pub.push(msg));

  EXPECT_CALL(*mock_publisher, publish(testing::Eq(msg))).Times(1);

  // Give some time for the publishing thread to process the message
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rt_pub.stop();
}
