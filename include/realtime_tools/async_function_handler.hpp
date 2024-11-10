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

/// \author Sai Kishor Kothakota

#ifndef REALTIME_TOOLS__ASYNC_FUNCTION_HANDLER_HPP_
#define REALTIME_TOOLS__ASYNC_FUNCTION_HANDLER_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace realtime_tools
{
/**
 * @brief Class to handle asynchronous function calls.
 * AsyncFunctionHandler is a class that allows the user to have a asynchronous call to the parsed
 * method and be able to set some thread specific parameters
 */
template <typename T>
class AsyncFunctionHandler
{
public:
  AsyncFunctionHandler() = default;

  ~AsyncFunctionHandler() { stop_thread(); }

  /// Initialize the AsyncFunctionHandler with the callback and thread_priority
  /**
   * @param callback Function that will be called asynchronously
   * If the AsyncFunctionHandler is already initialized and is running, it will throw a runtime
   * error.
   * If the parsed functions are not valid, it will throw a runtime error.
   */
  void init(
    std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> callback,
    int thread_priority = 50)
  {
    if (callback == nullptr) {
      throw std::runtime_error(
        "AsyncFunctionHandler: parsed function to call asynchronously is not valid!");
    }
    if (thread_.joinable()) {
      throw std::runtime_error(
        "AsyncFunctionHandler: Cannot reinitialize while the thread is "
        "running. Please stop the async callback first!");
    }
    async_function_ = callback;
    thread_priority_ = thread_priority;
  }

  /// Initialize the AsyncFunctionHandler with the callback, trigger_predicate and thread_priority
  /**
   * @param callback Function that will be called asynchronously.
   * @param trigger_predicate Predicate function that will be called to check if the async callback
   * method should be triggered or not.
   * @param thread_priority Priority of the async worker thread.
   *
   * \note The parsed trigger_predicate should be free from any concurrency issues. It is expected
   * to be both thread-safe and reentrant.
   *
   * If the AsyncFunctionHandler is already initialized and is running, it will throw a runtime
   * error.
   * If the parsed functions are not valid, it will throw a runtime error.
   */
  void init(
    std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> callback,
    std::function<bool()> trigger_predicate, int thread_priority = 50)
  {
    if (trigger_predicate == nullptr) {
      throw std::runtime_error("AsyncFunctionHandler: parsed trigger predicate is not valid!");
    }
    init(callback, thread_priority);
    trigger_predicate_ = trigger_predicate;
  }

  /// Triggers the async callback method cycle
  /**
   * @param time Current time
   * @param period Current period
   * @return A pair with the first element being a boolean indicating if the async callback method was
   * triggered and the second element being the last return value of the async function.
   * If the AsyncFunctionHandler is not initialized properly, it will throw a runtime error.
   * If the callback method is waiting for the trigger, it will notify the async thread to start
   * the callback.
   * If the async callback method is still running, it will return the last return value from the
   * last trigger cycle.
   *
   * \note If an exception is caught in the async callback thread, it will be rethrown in the current
   * thread, so in order to have the trigger_async_callback method working again, the exception should
   * be caught and the `reset_variables` method should be invoked.
   *
   * \note In the case of controllers, The controller manager is responsible
   * for triggering and maintaining the controller's update rate, as it should be only acting as a
   * scheduler. Same applies to the resource manager when handling the hardware components.
   */
  std::pair<bool, T> trigger_async_callback(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    if (!is_initialized()) {
      throw std::runtime_error("AsyncFunctionHandler: need to be initialized first!");
    }
    if (async_exception_ptr_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("AsyncFunctionHandler"),
        "AsyncFunctionHandler: Exception caught in the async callback thread!");
      std::rethrow_exception(async_exception_ptr_);
    }
    if (!is_running()) {
      throw std::runtime_error(
        "AsyncFunctionHandler: need to start the async callback thread first before triggering!");
    }
    std::unique_lock<std::mutex> lock(async_mtx_, std::try_to_lock);
    bool trigger_status = false;
    if (lock.owns_lock() && !trigger_in_progress_ && trigger_predicate_()) {
      {
        std::unique_lock<std::mutex> scoped_lock(std::move(lock));
        trigger_in_progress_ = true;
        current_callback_time_ = time;
        current_callback_period_ = period;
      }
      async_callback_condition_.notify_one();
      trigger_status = true;
    }
    const T return_value = async_callback_return_;
    return std::make_pair(trigger_status, return_value);
  }

  /// Get the last return value of the async callback method
  /**
   * @return The last return value of the async callback method
   */
  T get_last_return_value() const { return async_callback_return_; }

  /// Get the current callback time
  /**
   * @return The current callback time
   */
  const rclcpp::Time & get_current_callback_time() const { return current_callback_time_; }

  /// Get the current callback period
  /**
   * @return The current callback period
   */
  const rclcpp::Duration & get_current_callback_period() const { return current_callback_period_; }

  /// Resets the internal variables of the AsyncFunctionHandler
  /**
   * A method to reset the internal variables of the AsyncFunctionHandler.
   * It will reset the async callback return value, exception pointer, and the trigger status.
   *
   * \note This method should be invoked after catching an exception in the async callback thread,
   * to be able to trigger the async callback method again.
   */
  void reset_variables()
  {
    std::unique_lock<std::mutex> lock(async_mtx_);
    stop_async_callback_ = false;
    trigger_in_progress_ = false;
    current_callback_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
    current_callback_period_ = rclcpp::Duration(0, 0);
    last_execution_time_ = std::chrono::nanoseconds(0);
    async_callback_return_ = T();
    async_exception_ptr_ = nullptr;
  }

  /// Waits until the current async callback method trigger cycle is finished
  /**
   * If the async method is running, it will wait for the current async method call to finish.
   */
  void wait_for_trigger_cycle_to_finish()
  {
    if (is_running()) {
      std::unique_lock<std::mutex> lock(async_mtx_);
      cycle_end_condition_.wait(lock, [this] { return !trigger_in_progress_; });
    }
  }

  /// Check if the AsyncFunctionHandler is initialized
  /**
   * @return True if the AsyncFunctionHandler is initialized, false otherwise
   */
  bool is_initialized() const { return async_function_ && trigger_predicate_; }

  /// Join the async callback thread
  /**
   * If the async method is running, it will join the async thread.
   * If the async method is not running, it will return immediately.
   */
  void join_async_callback_thread()
  {
    if (is_running()) {
      thread_.join();
    }
  }

  /// Check if the async worker thread is running
  /**
   * @return True if the async worker thread is running, false otherwise
   */
  bool is_running() const { return thread_.joinable(); }

  /// Check if the async callback is triggered to stop the cycle
  /**
   * @return True if the async callback is requested to be stopped, false otherwise
   */
  bool is_stopped() const { return stop_async_callback_; }

  /// Get the async worker thread
  /**
   * @return The async callback thread
   */
  std::thread & get_thread() { return thread_; }

  /// Get the const version of async worker thread
  /**
   * @return The async callback thread
   */
  const std::thread & get_thread() const { return thread_; }

  /// Check if the async callback method is in progress
  /**
   * @return True if the async callback method is in progress, false otherwise
   */
  bool is_trigger_cycle_in_progress() const { return trigger_in_progress_; }

  /// Stops the callback thread
  /**
   * If the async method is running, it will notify the async thread to stop and then joins the
   * async thread.
   */
  void stop_thread()
  {
    if (is_running()) {
      {
        std::unique_lock<std::mutex> lock(async_mtx_);
        stop_async_callback_ = true;
      }
      async_callback_condition_.notify_one();
      thread_.join();
    }
  }

  /// Get the last execution time of the async callback method
  /**
   * @return The last execution time of the async callback method in nanoseconds
   */
  std::chrono::nanoseconds get_last_execution_time() const
  {
    return last_execution_time_.load(std::memory_order_relaxed);
  }

  /// Initializes and starts the callback thread
  /**
   * If the worker thread is not running, it will start the async callback thread.
   * If the worker thread is already configured and running, does nothing and returns
   * immediately.
   */
  void start_thread()
  {
    if (!is_initialized()) {
      throw std::runtime_error("AsyncFunctionHandler: need to be initialized first!");
    }
    if (!thread_.joinable()) {
      reset_variables();
      thread_ = std::thread([this]() -> void {
        if (!realtime_tools::configure_sched_fifo(thread_priority_)) {
          RCLCPP_WARN(
            rclcpp::get_logger("AsyncFunctionHandler"),
            "Could not enable FIFO RT scheduling policy. Consider setting up your user to do FIFO "
            "RT "
            "scheduling. See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details.");
        }

        while (!stop_async_callback_.load(std::memory_order_relaxed)) {
          {
            std::unique_lock<std::mutex> lock(async_mtx_);
            async_callback_condition_.wait(
              lock, [this] { return trigger_in_progress_ || stop_async_callback_; });
            if (!stop_async_callback_) {
              const auto start_time = std::chrono::steady_clock::now();
              try {
                async_callback_return_ =
                  async_function_(current_callback_time_, current_callback_period_);
              } catch (...) {
                async_exception_ptr_ = std::current_exception();
              }
              const auto end_time = std::chrono::steady_clock::now();
              last_execution_time_ =
                std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
            }
            trigger_in_progress_ = false;
          }
          cycle_end_condition_.notify_all();
        }
      });
    }
  }

private:
  rclcpp::Time current_callback_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  rclcpp::Duration current_callback_period_{0, 0};

  std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> async_function_;
  std::function<bool()> trigger_predicate_ = []() { return true; };

  // Async related variables
  std::thread thread_;
  int thread_priority_ = std::numeric_limits<int>::quiet_NaN();
  std::atomic_bool stop_async_callback_{false};
  std::atomic_bool trigger_in_progress_{false};
  std::atomic<T> async_callback_return_;
  std::condition_variable async_callback_condition_;
  std::condition_variable cycle_end_condition_;
  std::mutex async_mtx_;
  std::atomic<std::chrono::nanoseconds> last_execution_time_;
  std::exception_ptr async_exception_ptr_;
};
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__ASYNC_FUNCTION_HANDLER_HPP_
