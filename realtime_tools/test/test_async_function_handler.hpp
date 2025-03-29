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

#ifndef TEST_ASYNC_FUNCTION_HANDLER_HPP_
#define TEST_ASYNC_FUNCTION_HANDLER_HPP_

#include <utility>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/async_function_handler.hpp"

namespace realtime_tools
{
enum class return_type : std::uint8_t {
  OK = 0,
  FAILURE = 1,
  DEACTIVATE = 2,
};

class TestAsyncFunctionHandler
{
public:
  TestAsyncFunctionHandler();

  void initialize();

  realtime_tools::AsyncFunctionHandler<return_type> & get_handler() { return handler_; }

  std::pair<bool, return_type> trigger();

  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);

  const rclcpp_lifecycle::State & get_state() const;

  int get_counter() const;

  void activate();

  void deactivate();

  void reset_counter(int counter = 0);

  void set_return_state(return_type return_state);

private:
  rclcpp_lifecycle::State state_;
  int counter_;
  return_type return_state_;
  realtime_tools::AsyncFunctionHandler<return_type> handler_;
};
}  // namespace realtime_tools
#endif  // TEST_ASYNC_FUNCTION_HANDLER_HPP_
