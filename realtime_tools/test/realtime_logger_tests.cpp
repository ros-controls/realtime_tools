// Copyright 2026 ros2_control Development Team
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

#include <gmock/gmock.h>

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "rcutils/logging.h"
#include "realtime_tools/realtime_logger.hpp"

using ::testing::ElementsAre;

namespace
{
struct Captured
{
  int severity;
  std::string name;
  std::string message;
  std::string function;
  size_t line;
};

std::mutex g_mutex;
std::vector<Captured> g_captured;

void capture_handler(
  const rcutils_log_location_t * location, int severity, const char * name,
  rcutils_time_point_value_t, const char * format, va_list * args)
{
  char buffer[1024];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  std::lock_guard<std::mutex> lock(g_mutex);
  g_captured.push_back(
    {severity, name, buffer, location ? location->function_name : "",
     location ? location->line_number : 0});
}

std::vector<std::string> messages()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  std::vector<std::string> out;
  for (const auto & c : g_captured) {
    out.push_back(c.message);
  }
  return out;
}

// Fake clock so throttling is deterministic; the macros only need `now()`.
struct FakeClock
{
  int64_t ns = 0;
  rclcpp::Time now() const { return rclcpp::Time(ns, RCL_STEADY_TIME); }
};

class RealtimeLoggerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    previous_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(capture_handler);
    ASSERT_EQ(rcutils_logging_set_logger_level(kName, RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
    std::lock_guard<std::mutex> lock(g_mutex);
    g_captured.clear();
  }
  void TearDown() override { rcutils_logging_set_output_handler(previous_handler_); }

  static constexpr const char * kName = "rt_logger_test";
  rcutils_logging_output_handler_t previous_handler_;
};
}  // namespace

TEST_F(RealtimeLoggerTest, forwards_messages_with_severity_name_and_location)
{
  int line = 0;
  {
    realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName));
    RT_LOG_INFO(logger, "value %d", 42);
    line = __LINE__ - 1;
    RT_LOG_ERROR(logger, "plain");
  }  // destruction flushes everything pending
  std::lock_guard<std::mutex> lock(g_mutex);
  ASSERT_EQ(g_captured.size(), 2u);
  EXPECT_EQ(g_captured[0].severity, RCUTILS_LOG_SEVERITY_INFO);
  EXPECT_EQ(g_captured[0].name, kName);
  EXPECT_EQ(g_captured[0].message, "value 42");
  EXPECT_EQ(g_captured[0].line, static_cast<size_t>(line));
  EXPECT_EQ(g_captured[0].function, "TestBody");
  EXPECT_EQ(g_captured[1].severity, RCUTILS_LOG_SEVERITY_ERROR);
}

TEST_F(RealtimeLoggerTest, background_thread_drains_without_destruction)
{
  realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName), std::chrono::milliseconds(1));
  RT_LOG_WARN(logger, "async");
  for (int i = 0; i < 1000 && messages().empty(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_THAT(messages(), ElementsAre("async"));
}

TEST_F(RealtimeLoggerTest, respects_logger_level)
{
  ASSERT_EQ(rcutils_logging_set_logger_level(kName, RCUTILS_LOG_SEVERITY_WARN), RCUTILS_RET_OK);
  {
    realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName));
    RT_LOG_DEBUG(logger, "debug");
    RT_LOG_INFO(logger, "info");
    RT_LOG_WARN(logger, "warn");
    RT_LOG_FATAL(logger, "fatal");
  }
  EXPECT_THAT(messages(), ElementsAre("warn", "fatal"));
}

TEST_F(RealtimeLoggerTest, once_skipfirst_expression_function)
{
  {
    realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName));
    for (int i = 0; i < 3; ++i) {
      RT_LOG_INFO_ONCE(logger, "once %d", i);
      RT_LOG_INFO_SKIPFIRST(logger, "skip %d", i);
      RT_LOG_INFO_EXPRESSION(logger, i == 1, "expr %d", i);
      RT_LOG_INFO_FUNCTION(logger, [i]() { return i == 2; }, "func %d", i);
    }
  }
  EXPECT_THAT(
    messages(), ElementsAre("once 0", "skip 1", "expr 1", "skip 2", "func 2"));
}

TEST_F(RealtimeLoggerTest, throttle_and_skipfirst_throttle)
{
  FakeClock clock;
  {
    realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName));
    // 0, 50, 100, 150, 200, 250 ms with a 100 ms throttle period
    for (int i = 0; i < 6; ++i) {
      clock.ns = i * 50'000'000LL;
      RT_LOG_INFO_THROTTLE(logger, clock, 100, "throttle %d", i);
      RT_LOG_INFO_SKIPFIRST_THROTTLE(logger, clock, 100, "skipfirst %d", i);
    }
  }
  EXPECT_THAT(
    messages(), ElementsAre(
                  "throttle 0", "throttle 2", "skipfirst 2", "throttle 4", "skipfirst 4"));
}

TEST_F(RealtimeLoggerTest, drops_and_counts_when_queue_is_full)
{
  // A very long drain period keeps the background thread from emptying the queue during the loop.
  realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName), std::chrono::hours(1));
  const std::size_t extra = 5;
  for (std::size_t i = 0; i < realtime_tools::RealtimeLogger::kQueueCapacity + extra; ++i) {
    RT_LOG_INFO(logger, "msg %zu", i);
  }
  EXPECT_EQ(logger.get_dropped_count(), extra);
}

TEST_F(RealtimeLoggerTest, truncates_long_messages)
{
  const std::string long_text(realtime_tools::RealtimeLogger::kMaxMessageSize * 2, 'x');
  {
    realtime_tools::RealtimeLogger logger(rclcpp::get_logger(kName));
    RT_LOG_INFO(logger, "%s", long_text.c_str());
  }
  const auto out = messages();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].size(), realtime_tools::RealtimeLogger::kMaxMessageSize - 1);
}
