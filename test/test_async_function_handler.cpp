// Copyright 2024 PAL Robotics S.L.
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

#include "test_async_function_handler.hpp"
#include "gmock/gmock.h"
#include "rclcpp/rclcpp.hpp"

namespace realtime_tools
{
TestAsyncFunctionHandler::TestAsyncFunctionHandler()
: state_(rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "test_async")),
  counter_(0),
  return_state_(return_type::OK)
{
}

void TestAsyncFunctionHandler::initialize()
{
  handler_.init(
    std::bind(
      &TestAsyncFunctionHandler::update, this, std::placeholders::_1, std::placeholders::_2),
    [this]() { return state_.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE; });
}

std::pair<bool, return_type> TestAsyncFunctionHandler::trigger()
{
  return handler_.trigger_async_callback(rclcpp::Time(0, 0), rclcpp::Duration(0, 0));
}

return_type TestAsyncFunctionHandler::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // to simulate some work being done
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  counter_++;
  return return_state_;
}
const rclcpp_lifecycle::State & TestAsyncFunctionHandler::get_state() const { return state_; }

int TestAsyncFunctionHandler::get_counter() const { return counter_; }
void TestAsyncFunctionHandler::activate()
{
  state_ =
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, state_.label());
}

void TestAsyncFunctionHandler::deactivate()
{
  state_ =
    rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, state_.label());
}
}  // namespace realtime_tools
class AsyncFunctionHandlerTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST_F(AsyncFunctionHandlerTest, check_initialization)
{
  realtime_tools::TestAsyncFunctionHandler async_class;

  ASSERT_FALSE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());

  // It should not be possible to initialize setting wrong functions
  ASSERT_THROW(async_class.get_handler().init(nullptr, nullptr), std::runtime_error);

  async_class.initialize();
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());

  // Once initialized, it should not be possible to initialize again
  async_class.get_handler().start_thread();
  auto trigger_status = async_class.trigger();
  ASSERT_TRUE(trigger_status.first);
  ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  ASSERT_THROW(async_class.initialize(), std::runtime_error);
}

TEST_F(AsyncFunctionHandlerTest, check_triggering)
{
  realtime_tools::TestAsyncFunctionHandler async_class;

  ASSERT_FALSE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  // It shouldn't be possible to trigger without initialization
  ASSERT_THROW(async_class.trigger(), std::runtime_error);

  async_class.initialize();
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  // It shouldn't be possible to trigger without starting the thread
  ASSERT_THROW(async_class.trigger(), std::runtime_error);
  async_class.get_handler().start_thread();

  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto trigger_status = async_class.trigger();
  ASSERT_TRUE(trigger_status.first);
  ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  ASSERT_TRUE(async_class.get_handler().get_thread().joinable());
  ASSERT_TRUE(async_class.get_handler().is_trigger_cycle_in_progress());
  async_class.get_handler().wait_for_trigger_cycle_to_finish();
  async_class.get_handler().get_last_execution_time();
  ASSERT_FALSE(async_class.get_handler().is_trigger_cycle_in_progress());
  ASSERT_EQ(async_class.get_counter(), 1);

  // Trigger one more cycle
  trigger_status = async_class.trigger();
  ASSERT_TRUE(trigger_status.first);
  ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  async_class.get_handler().stop_thread();
  ASSERT_LE(async_class.get_counter(), 2);

  // now the async update should be preempted
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_TRUE(async_class.get_handler().is_stopped());
  async_class.get_handler().wait_for_trigger_cycle_to_finish();
}

TEST_F(AsyncFunctionHandlerTest, trigger_for_several_cycles)
{
  realtime_tools::TestAsyncFunctionHandler async_class;

  async_class.initialize();
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  async_class.get_handler().start_thread();

  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  int missed_triggers = 0;
  const int total_cycles = 1e5;
  for (int i = 1; i < total_cycles; i++) {
    const auto trigger_status = async_class.trigger();
    if (trigger_status.first) {
      ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
      ASSERT_TRUE(async_class.get_handler().is_initialized());
      ASSERT_TRUE(async_class.get_handler().is_running());
      ASSERT_FALSE(async_class.get_handler().is_stopped());
      async_class.get_handler().wait_for_trigger_cycle_to_finish();
      ASSERT_FALSE(async_class.get_handler().is_trigger_cycle_in_progress());
      ASSERT_EQ(async_class.get_counter(), i - missed_triggers);
    } else {
      missed_triggers++;
    }
  }
  // Make sure that the failed triggers are less than 0.1%
  ASSERT_LT(missed_triggers, static_cast<int>(0.001 * total_cycles))
    << "The missed triggers cannot be more than 0.1%!";
  async_class.get_handler().stop_thread();

  // now the async update should be preempted
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_TRUE(async_class.get_handler().is_stopped());
}

TEST_F(AsyncFunctionHandlerTest, test_with_deactivate_and_activate_cycles)
{
  realtime_tools::TestAsyncFunctionHandler async_class;

  // Start with a deactivated state
  async_class.initialize();
  async_class.deactivate();
  ASSERT_THROW(async_class.trigger(), std::runtime_error)
    << "Should throw before starting a thread";
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  async_class.get_handler().start_thread();
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());
  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Now activate it and launch again
  async_class.activate();
  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const int total_cycles = 100;
  for (int i = 1; i < total_cycles; i++) {
    const auto trigger_status = async_class.trigger();
    ASSERT_TRUE(trigger_status.first);
    ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
    ASSERT_TRUE(async_class.get_handler().is_trigger_cycle_in_progress());
    ASSERT_TRUE(async_class.get_handler().is_initialized());
    ASSERT_TRUE(async_class.get_handler().is_running());
    ASSERT_FALSE(async_class.get_handler().is_stopped());
    async_class.get_handler().wait_for_trigger_cycle_to_finish();
    ASSERT_FALSE(async_class.get_handler().is_trigger_cycle_in_progress());
    ASSERT_EQ(async_class.get_counter(), i);
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }

  // Now let's do one more trigger cycle and then change the state to deactivate, then it should end
  // the thread once the cycle is finished
  auto trigger_status = async_class.trigger();
  ASSERT_TRUE(trigger_status.first);
  ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
  async_class.deactivate();
  async_class.get_handler().wait_for_trigger_cycle_to_finish();
  for (int i = 0; i < 50; i++) {
    const auto trigger_status = async_class.trigger();
    ASSERT_FALSE(trigger_status.first);
    ASSERT_EQ(async_class.get_counter(), total_cycles)
      << "The trigger should fail for any state different than ACTIVE";
  }
  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(async_class.get_handler().is_initialized());
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());

  // Now let's test the case of activating it and then deactivating it when the thread is waiting
  // for a trigger to start new update cycle execution
  async_class.activate();

  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  trigger_status = async_class.trigger();
  ASSERT_TRUE(trigger_status.first);
  ASSERT_EQ(realtime_tools::return_type::OK, trigger_status.second);
  async_class.get_handler().wait_for_trigger_cycle_to_finish();
  async_class.deactivate();
  EXPECT_EQ(async_class.get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  // Now they continue to wait until a new cycle is triggered or the preempt is called
  ASSERT_TRUE(async_class.get_handler().is_running());
  ASSERT_FALSE(async_class.get_handler().is_stopped());

  // now the async update should be preempted
  async_class.get_handler().stop_thread();
  ASSERT_FALSE(async_class.get_handler().is_running());
  ASSERT_TRUE(async_class.get_handler().is_stopped());
}
