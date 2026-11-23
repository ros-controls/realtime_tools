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

#ifndef REALTIME_TOOLS__REALTIME_LOGGER_HPP_
#define REALTIME_TOOLS__REALTIME_LOGGER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/logger.hpp"
#include "rcutils/logging.h"
#include "realtime_tools/lock_free_pooled_queue.hpp"

#if defined(__GNUC__) || defined(__clang__)
#define REALTIME_TOOLS_PRINTF_FORMAT_(fmt_index, args_index) \
  __attribute__((format(printf, fmt_index, args_index)))
#else
#define REALTIME_TOOLS_PRINTF_FORMAT_(fmt_index, args_index)
#endif

namespace realtime_tools
{
/**
 * @brief Logger that can be called from a realtime thread and forwards to an rclcpp::Logger.
 *
 * The realtime side formats the message with vsnprintf into a preallocated slot of a wait-free
 * SPSC queue. A background thread drains the queue and hands each message to rcutils_log() with
 * the original logger name, severity and call-site location, so console output, log files and
 * /rosout behave exactly like with the RCLCPP_* macros.
 *
 * Use it through the RT_LOG_<SEVERITY>[_<VARIANT>] macros below rather than calling log()
 * directly; they add the severity check and the per-call-site conditions.
 *
 * @note Single producer: log from one realtime thread per RealtimeLogger instance.
 * @note When the queue is full the message is dropped (never blocks); see get_dropped_count().
 *       Messages longer than kMaxMessageSize - 1 characters are truncated.
 * @note glibc vsnprintf does not allocate for ordinary format strings, but may for extreme
 *       float precisions/widths. Keep format strings simple in the realtime loop.
 */
class RealtimeLogger
{
public:
  /// Number of messages that can be pending before new ones are dropped.
  static constexpr std::size_t kQueueCapacity = 128;
  /// Size of the per-message buffer, including the terminating null character.
  static constexpr std::size_t kMaxMessageSize = 512;

  /**
   * @param logger The rclcpp logger the messages are forwarded to.
   * @param drain_period How often the background thread empties the queue.
   */
  explicit RealtimeLogger(
    rclcpp::Logger logger,
    std::chrono::nanoseconds drain_period = std::chrono::milliseconds(10))
  : logger_(std::move(logger)),
    drain_period_(drain_period),
    queue_(std::make_unique<Queue>())
  {
    // rcutils initialises logging lazily on the first macro call, which allocates. Do it here so
    // is_enabled_for() is allocation-free in the realtime loop.
    RCUTILS_LOGGING_AUTOINIT;
    thread_ = std::thread([this]() { drain_loop(); });
  }

  /// Stops the background thread after forwarding every pending message.
  ~RealtimeLogger()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    cv_.notify_one();
    thread_.join();
  }

  RealtimeLogger(const RealtimeLogger &) = delete;
  RealtimeLogger & operator=(const RealtimeLogger &) = delete;

  /// @return true if the wrapped logger currently accepts @p severity. Realtime-safe.
  bool is_enabled_for(int severity) const
  {
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity);
  }

  /**
   * @brief Format a message and queue it for the background thread. Realtime-safe, wait-free.
   *
   * @param location Call-site location; must outlive the logger (the macros use a static).
   * @param severity One of the RCUTILS_LOG_SEVERITY_* values.
   * @param format printf-style format string.
   * @return false if the queue was full and the message was dropped.
   */
  bool log(const rcutils_log_location_t * location, int severity, const char * format, ...)
    REALTIME_TOOLS_PRINTF_FORMAT_(4, 5)
  {
    va_list args;
    va_start(args, format);
    const bool queued = queue_->produce([&](Entry & entry) noexcept {
      entry.location = location;
      entry.severity = severity;
      std::vsnprintf(entry.message, kMaxMessageSize, format, args);
    });
    va_end(args);
    if (!queued) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
    }
    return queued;
  }

  /// @return Number of messages dropped so far because the queue was full.
  std::size_t get_dropped_count() const { return dropped_.load(std::memory_order_relaxed); }

  const rclcpp::Logger & get_logger() const { return logger_; }

private:
  struct Entry
  {
    const rcutils_log_location_t * location = nullptr;
    int severity = RCUTILS_LOG_SEVERITY_UNSET;
    char message[kMaxMessageSize] = {};
  };
  using Queue = LockFreePooledSPSCQueue<Entry, kQueueCapacity>;

  void drain_loop()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!cv_.wait_for(lock, drain_period_, [this]() { return stop_; })) {
      drain();
    }
    drain();  // flush whatever was logged before destruction
  }

  void drain()
  {
    while (queue_->consume([this](const Entry & entry) {
      rcutils_log(entry.location, entry.severity, logger_.get_name(), "%s", entry.message);
    })) {
    }
  }

  rclcpp::Logger logger_;
  std::chrono::nanoseconds drain_period_;
  // Heap allocated: the pool holds kQueueCapacity * kMaxMessageSize bytes inline.
  std::unique_ptr<Queue> queue_;
  std::atomic<std::size_t> dropped_{0};
  // Only the background thread and the destructor touch these, never the realtime thread.
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stop_ = false;
  std::thread thread_;
};
}  // namespace realtime_tools

// ---------------------------------------------------------------------------------------------
// Macros mirroring RCLCPP_<SEVERITY>[_<VARIANT>]. The first argument is a RealtimeLogger instead of
// an rclcpp::Logger. Conditions keep per-call-site static state, like rcutils, and are evaluated
// only when the severity is enabled. There are no _STREAM variants: std::stringstream allocates.
// ---------------------------------------------------------------------------------------------

#define RT_LOG_EMIT_(rt_logger, severity, ...)                                           \
  {                                                                                      \
    static const rcutils_log_location_t rt_log_location_ = {__func__, __FILE__, __LINE__}; \
    (rt_logger).log(&rt_log_location_, severity, __VA_ARGS__);                           \
  }

#define RT_LOG_(rt_logger, severity, ...)            \
  do {                                               \
    if ((rt_logger).is_enabled_for(severity)) {      \
      RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__) \
    }                                                \
  } while (0)

#define RT_LOG_ONCE_(rt_logger, severity, ...)     \
  do {                                             \
    if (!(rt_logger).is_enabled_for(severity)) {   \
      break;                                       \
    }                                              \
    static bool rt_log_done_ = false;              \
    if (rt_log_done_) {                            \
      break;                                       \
    }                                              \
    rt_log_done_ = true;                           \
    RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__) \
  } while (0)

#define RT_LOG_SKIPFIRST_(rt_logger, severity, ...) \
  do {                                              \
    if (!(rt_logger).is_enabled_for(severity)) {    \
      break;                                        \
    }                                               \
    static bool rt_log_first_ = true;               \
    if (rt_log_first_) {                            \
      rt_log_first_ = false;                        \
      break;                                        \
    }                                               \
    RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__)  \
  } while (0)

#define RT_LOG_EXPRESSION_(rt_logger, severity, expression, ...)              \
  do {                                                                        \
    if ((rt_logger).is_enabled_for(severity) && (expression)) {               \
      RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__)                          \
    }                                                                         \
  } while (0)

#define RT_LOG_FUNCTION_(rt_logger, severity, function, ...)                  \
  do {                                                                        \
    if ((rt_logger).is_enabled_for(severity) && (function)()) {               \
      RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__)                          \
    }                                                                         \
  } while (0)

// The throttle state check is shared by _THROTTLE and _SKIPFIRST_THROTTLE. `clock` is anything
// with now().nanoseconds(), e.g. rclcpp::Clock; it must be realtime-safe to read.
#define RT_LOG_THROTTLE_CHECK_(clock, duration_ms)                                     \
  const int64_t rt_log_now_ = (clock).now().nanoseconds();                             \
  static bool rt_log_started_ = false;                                                 \
  static int64_t rt_log_last_ = 0;                                                     \
  if (rt_log_started_ && rt_log_now_ - rt_log_last_ < (duration_ms) * 1000000LL) {     \
    break;                                                                             \
  }                                                                                    \
  rt_log_started_ = true;                                                              \
  rt_log_last_ = rt_log_now_;

#define RT_LOG_THROTTLE_(rt_logger, severity, clock, duration_ms, ...) \
  do {                                                                 \
    if (!(rt_logger).is_enabled_for(severity)) {                       \
      break;                                                           \
    }                                                                  \
    RT_LOG_THROTTLE_CHECK_(clock, duration_ms)                         \
    RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__)                     \
  } while (0)

// Like rcutils: the first message that passes the throttle is skipped, later ones are throttled.
#define RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, severity, clock, duration_ms, ...) \
  do {                                                                           \
    if (!(rt_logger).is_enabled_for(severity)) {                                 \
      break;                                                                     \
    }                                                                            \
    RT_LOG_THROTTLE_CHECK_(clock, duration_ms)                                   \
    static bool rt_log_first_ = true;                                            \
    if (rt_log_first_) {                                                         \
      rt_log_first_ = false;                                                     \
      break;                                                                     \
    }                                                                            \
    RT_LOG_EMIT_(rt_logger, severity, __VA_ARGS__)                               \
  } while (0)

#define RT_LOG_DEBUG(rt_logger, ...) RT_LOG_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, __VA_ARGS__)
#define RT_LOG_DEBUG_ONCE(rt_logger, ...) \
  RT_LOG_ONCE_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, __VA_ARGS__)
#define RT_LOG_DEBUG_SKIPFIRST(rt_logger, ...) \
  RT_LOG_SKIPFIRST_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, __VA_ARGS__)
#define RT_LOG_DEBUG_EXPRESSION(rt_logger, expression, ...) \
  RT_LOG_EXPRESSION_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, expression, __VA_ARGS__)
#define RT_LOG_DEBUG_FUNCTION(rt_logger, function, ...) \
  RT_LOG_FUNCTION_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, function, __VA_ARGS__)
#define RT_LOG_DEBUG_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, clock, duration_ms, __VA_ARGS__)
#define RT_LOG_DEBUG_SKIPFIRST_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_DEBUG, clock, duration_ms, __VA_ARGS__)

#define RT_LOG_INFO(rt_logger, ...) RT_LOG_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, __VA_ARGS__)
#define RT_LOG_INFO_ONCE(rt_logger, ...) \
  RT_LOG_ONCE_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, __VA_ARGS__)
#define RT_LOG_INFO_SKIPFIRST(rt_logger, ...) \
  RT_LOG_SKIPFIRST_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, __VA_ARGS__)
#define RT_LOG_INFO_EXPRESSION(rt_logger, expression, ...) \
  RT_LOG_EXPRESSION_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, expression, __VA_ARGS__)
#define RT_LOG_INFO_FUNCTION(rt_logger, function, ...) \
  RT_LOG_FUNCTION_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, function, __VA_ARGS__)
#define RT_LOG_INFO_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, clock, duration_ms, __VA_ARGS__)
#define RT_LOG_INFO_SKIPFIRST_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_INFO, clock, duration_ms, __VA_ARGS__)

#define RT_LOG_WARN(rt_logger, ...) RT_LOG_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, __VA_ARGS__)
#define RT_LOG_WARN_ONCE(rt_logger, ...) \
  RT_LOG_ONCE_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, __VA_ARGS__)
#define RT_LOG_WARN_SKIPFIRST(rt_logger, ...) \
  RT_LOG_SKIPFIRST_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, __VA_ARGS__)
#define RT_LOG_WARN_EXPRESSION(rt_logger, expression, ...) \
  RT_LOG_EXPRESSION_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, expression, __VA_ARGS__)
#define RT_LOG_WARN_FUNCTION(rt_logger, function, ...) \
  RT_LOG_FUNCTION_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, function, __VA_ARGS__)
#define RT_LOG_WARN_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, clock, duration_ms, __VA_ARGS__)
#define RT_LOG_WARN_SKIPFIRST_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_WARN, clock, duration_ms, __VA_ARGS__)

#define RT_LOG_ERROR(rt_logger, ...) RT_LOG_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, __VA_ARGS__)
#define RT_LOG_ERROR_ONCE(rt_logger, ...) \
  RT_LOG_ONCE_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, __VA_ARGS__)
#define RT_LOG_ERROR_SKIPFIRST(rt_logger, ...) \
  RT_LOG_SKIPFIRST_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, __VA_ARGS__)
#define RT_LOG_ERROR_EXPRESSION(rt_logger, expression, ...) \
  RT_LOG_EXPRESSION_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, expression, __VA_ARGS__)
#define RT_LOG_ERROR_FUNCTION(rt_logger, function, ...) \
  RT_LOG_FUNCTION_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, function, __VA_ARGS__)
#define RT_LOG_ERROR_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, clock, duration_ms, __VA_ARGS__)
#define RT_LOG_ERROR_SKIPFIRST_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_ERROR, clock, duration_ms, __VA_ARGS__)

#define RT_LOG_FATAL(rt_logger, ...) RT_LOG_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, __VA_ARGS__)
#define RT_LOG_FATAL_ONCE(rt_logger, ...) \
  RT_LOG_ONCE_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, __VA_ARGS__)
#define RT_LOG_FATAL_SKIPFIRST(rt_logger, ...) \
  RT_LOG_SKIPFIRST_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, __VA_ARGS__)
#define RT_LOG_FATAL_EXPRESSION(rt_logger, expression, ...) \
  RT_LOG_EXPRESSION_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, expression, __VA_ARGS__)
#define RT_LOG_FATAL_FUNCTION(rt_logger, function, ...) \
  RT_LOG_FUNCTION_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, function, __VA_ARGS__)
#define RT_LOG_FATAL_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, clock, duration_ms, __VA_ARGS__)
#define RT_LOG_FATAL_SKIPFIRST_THROTTLE(rt_logger, clock, duration_ms, ...) \
  RT_LOG_SKIPFIRST_THROTTLE_(rt_logger, RCUTILS_LOG_SEVERITY_FATAL, clock, duration_ms, __VA_ARGS__)

#endif  // REALTIME_TOOLS__REALTIME_LOGGER_HPP_
