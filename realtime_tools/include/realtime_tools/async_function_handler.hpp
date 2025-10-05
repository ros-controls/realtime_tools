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
#include <cmath>
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

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_helpers.hpp"

namespace realtime_tools
{
/**
 * @brief Enum class to define the scheduling policy for the async worker thread.
 * SYNCHRONIZED: The async worker thread will be synchronized with the main thread, as the main
 * thread will be triggering the async callback method.
 * DETACHED: The async worker thread will be detached from the main thread and will have its own
 * execution cycle.
 * UNKNOWN: The scheduling policy is unknown.
 */
class AsyncSchedulingPolicy
{
public:
  enum Value : int8_t {
    UNKNOWN = -1,  /// Unknown scheduling policy
    SYNCHRONIZED,  /// Synchronized scheduling policy
    DETACHED,      /// Detached scheduling policy
  };

  AsyncSchedulingPolicy() = default;
  constexpr AsyncSchedulingPolicy(Value value) : value_(value) {}  // NOLINT(runtime/explicit)
  explicit AsyncSchedulingPolicy(const std::string & data_type)
  {
    if (data_type == "synchronized") {
      value_ = SYNCHRONIZED;
    } else if (data_type == "detached") {
      value_ = DETACHED;
    } else {
      value_ = UNKNOWN;
    }
  }

  operator Value() const { return value_; }

  explicit operator bool() const = delete;

  constexpr bool operator==(AsyncSchedulingPolicy other) const { return value_ == other.value_; }
  constexpr bool operator!=(AsyncSchedulingPolicy other) const { return value_ != other.value_; }

  constexpr bool operator==(Value other) const { return value_ == other; }
  constexpr bool operator!=(Value other) const { return value_ != other; }

  std::string to_string() const
  {
    switch (value_) {
      case SYNCHRONIZED:
        return "synchronized";
      case DETACHED:
        return "detached";
      default:
        return "unknown";
    }
  }

  AsyncSchedulingPolicy from_string(const std::string & data_type)
  {
    return AsyncSchedulingPolicy(data_type);
  }

private:
  Value value_ = UNKNOWN;
};

/**
 * @brief The AsyncFunctionHandlerParams struct is used to configure the AsyncFunctionHandler.
 * If the type is SYNCHRONIZED, the async worker thread will be synchronized with the main
 * thread, as the main thread will be triggering the async callback method.
 * If the type is DETACHED, the async worker thread will be detached from the main thread and
 * will have its own execution cycle.
 *
 * @param thread_priority Priority of the async worker thread. Should be between 0 and 99.
 * @param cpu_affinity_cores CPU cores to which the async worker thread should be pinned.
 * If empty, the thread will not be pinned to any CPU core.
 * @param scheduling_policy Scheduling policy for the async worker thread. Can be either
 * SYNCHRONIZED or DETACHED.
 * @param exec_rate Execution rate of the async worker thread in Hz. Only used if the
 * scheduling_policy is DETACHED. Must be a positive integer.
 * @param clock Clock to be used for the async worker thread. Only used if the scheduling_policy
 * is DETACHED.
 * @param logger Logger to be used for the async worker thread. If not set, a default logger will be used.
 * @param trigger_predicate Predicate function to check if the async callback method should be triggered or not.
 * If not set, the async callback method will be triggered every time.
 * @param wait_until_initial_trigger Whether to wait until the initial trigger predicate is true before starting
 * the async callback method. If true, the async callback method will not be called until the trigger predicate
 * returns true for the first time. Very useful when the type is DETACHED.
 * @param print_warnings Whether to print warnings when the async callback method is not triggered due to any reason.
 */
struct AsyncFunctionHandlerParams
{
  /**
   * @brief Validate the parameters.
   * @return true if the parameters are valid, false otherwise.
   * @throws std::runtime_error if the scheduling policy is UNKNOWN.
   */
  bool validate() const
  {
    if (thread_priority < 0 || thread_priority > 99) {
      RCLCPP_ERROR(
        logger, "Invalid thread priority: %d. It should be between 0 and 99.", thread_priority);
      return false;
    }
    if (scheduling_policy == AsyncSchedulingPolicy::DETACHED) {
      if (!clock) {
        RCLCPP_ERROR(logger, "Clock must be set when using DETACHED scheduling policy.");
        return false;
      }
      if (exec_rate == 0u) {
        RCLCPP_ERROR(logger, "Execution rate must be set when using DETACHED scheduling policy.");
        return false;
      }
    }
    if (scheduling_policy == AsyncSchedulingPolicy::UNKNOWN) {
      throw std::runtime_error(
        "AsyncFunctionHandlerParams: scheduling policy is unknown. "
        "Please set it to either 'synchronized' or 'detached'.");
    }
    if (trigger_predicate == nullptr) {
      RCLCPP_ERROR(logger, "The parsed trigger predicate is not valid!");
      return false;
    }
    for (const int & core : cpu_affinity_cores) {
      if (core < 0) {
        RCLCPP_ERROR(logger, "Invalid CPU core id: %d. It should be a non-negative integer.", core);
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Initialize the parameters from a node's parameters.
   * The node should have the following parameters:
   * - thread_priority (int): Priority of the async worker thread. Default is 50.
   * - cpu_affinity (int[]): CPU cores to which the async worker thread should be pinned.
   *   Default is empty, which means the thread will not be pinned to any CPU core.
   * - scheduling_policy (string): Scheduling policy for the async worker thread. Can be either
   *   "synchronized" or "detached". Default is "synchronized".
   * - execution_rate (int): Execution rate of the async worker thread in Hz.
   * - wait_until_initial_trigger (bool): Whether to wait until the initial trigger predicate is true
   *   before starting the async callback method. Default is true.
   * - print_warnings (bool): Whether to print warnings when the async callback method is not triggered
   *   due to any reason. Default is true.
   * @param node The node that is used to get the parameters.
   * @param prefix Parameter prefix to use when accessing node parameters.
   */
  template <typename NodeT>
  void initialize(NodeT & node, const std::string & prefix)
  {
    if (node->has_parameter(prefix + "thread_priority")) {
      thread_priority = static_cast<int>(node->get_parameter(prefix + "thread_priority").as_int());
    }
    if (node->has_parameter(prefix + "cpu_affinity")) {
      const auto cpu_affinity_param =
        node->get_parameter(prefix + "cpu_affinity").as_integer_array();
      for (const auto & core : cpu_affinity_param) {
        cpu_affinity_cores.push_back(static_cast<int>(core));
      }
    }
    if (node->has_parameter(prefix + "scheduling_policy")) {
      scheduling_policy =
        AsyncSchedulingPolicy(node->get_parameter(prefix + "scheduling_policy").as_string());
    }
    if (
      scheduling_policy == AsyncSchedulingPolicy::DETACHED &&
      node->has_parameter(prefix + "execution_rate")) {
      const int execution_rate =
        static_cast<int>(node->get_parameter(prefix + "execution_rate").as_int());
      if (execution_rate <= 0) {
        throw std::runtime_error(
          "AsyncFunctionHandler: execution_rate parameter must be positive.");
      }
      exec_rate = static_cast<unsigned int>(execution_rate);
    }
    if (node->has_parameter(prefix + "wait_until_initial_trigger")) {
      wait_until_initial_trigger =
        node->get_parameter(prefix + "wait_until_initial_trigger").as_bool();
    }
    if (node->has_parameter(prefix + "print_warnings")) {
      print_warnings = node->get_parameter(prefix + "print_warnings").as_bool();
    }
  }

  int thread_priority = 50;
  std::vector<int> cpu_affinity_cores = {};
  AsyncSchedulingPolicy scheduling_policy = AsyncSchedulingPolicy::SYNCHRONIZED;
  unsigned int exec_rate = 0u;
  rclcpp::Clock::SharedPtr clock = nullptr;
  rclcpp::Logger logger = rclcpp::get_logger("AsyncFunctionHandler");
  std::function<bool()> trigger_predicate = []() { return true; };
  bool wait_until_initial_trigger = true;
  bool print_warnings = true;
};

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

  void init(
    std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> callback,
    const AsyncFunctionHandlerParams & params)
  {
    params.validate();
    init(callback, params.trigger_predicate, params.thread_priority);
    params_ = params;
    pause_thread_ = params.wait_until_initial_trigger;
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
        params_.logger, "AsyncFunctionHandler: Exception caught in the async callback thread!");
      std::rethrow_exception(async_exception_ptr_);
    }
    if (params_.scheduling_policy == AsyncSchedulingPolicy::DETACHED) {
      RCLCPP_WARN_ONCE(
        params_.logger,
        "AsyncFunctionHandler is configured with DETACHED scheduling policy. "
        "This means that the async callback may not be synchronized with the main thread. ");
      if (pause_thread_.load(std::memory_order_relaxed)) {
        {
          std::unique_lock<std::mutex> lock(async_mtx_);
          pause_thread_ = false;
          RCLCPP_INFO(params_.logger, "AsyncFunctionHandler: Resuming the async callback thread.");
          async_callback_return_ = T();
          auto const sync_period = std::chrono::nanoseconds(1'000'000'000 / params_.exec_rate);
          previous_time_ = params_.clock->now() - rclcpp::Duration(sync_period);
        }
        async_callback_condition_.notify_one();
      }
      return std::make_pair(true, async_callback_return_.load(std::memory_order_relaxed));
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
  bool wait_for_trigger_cycle_to_finish()
  {
    if (is_running()) {
      std::unique_lock<std::mutex> lock(async_mtx_);
      cycle_end_condition_.wait(lock, [this] { return !trigger_in_progress_; });
      return true;
    }
    return false;
  }

  /// Pauses the execution of the async callback thread
  /**
   * If the async method is running, it will pause the async thread until the next trigger cycle.
   * If the async method is not running, it will return immediately.
   * \note This method is non real-time safe, as it uses a mutex to pause the thread.
   *
   * \returns True if the async callback thread was paused, false otherwise.
   */
  bool pause_execution()
  {
    RCLCPP_INFO_EXPRESSION(
      params_.logger, !pause_thread_, "AsyncFunctionHandler: Pausing the async callback thread.");
    if (params_.scheduling_policy == AsyncSchedulingPolicy::SYNCHRONIZED) {
      pause_thread_ = true;
      return wait_for_trigger_cycle_to_finish();
    } else {
      if (is_running()) {
        pause_thread_.store(true, std::memory_order_relaxed);
        std::unique_lock<std::mutex> lock(async_mtx_);
        return true;
      }
    }
    return pause_thread_.load(std::memory_order_relaxed);
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
  bool is_stopped() const { return stop_async_callback_.load(std::memory_order_relaxed); }

  /// Check if the async callback thread is paused
  /**
   * @return True if the async callback thread is paused, false otherwise
   */
  bool is_paused() const { return pause_thread_.load(std::memory_order_relaxed); }

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

  /// Get the parameters used to configure the AsyncFunctionHandler
  /**
   * @return The parameters used to configure the AsyncFunctionHandler
   */
  const AsyncFunctionHandlerParams & get_params() const { return params_; }

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
        stop_async_callback_.store(true, std::memory_order_relaxed);
        std::unique_lock<std::mutex> lock(async_mtx_);
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
            params_.logger,
            "Could not enable FIFO RT scheduling policy. Consider setting up your user to do FIFO "
            "RT scheduling. See "
            "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
            "for details.");
        }
        if (!params_.cpu_affinity_cores.empty()) {
          const auto affinity_result =
            realtime_tools::set_current_thread_affinity(params_.cpu_affinity_cores);
          RCLCPP_WARN_EXPRESSION(
            params_.logger, !affinity_result.first,
            "Could not set CPU affinity for the async worker thread. Error: %s",
            affinity_result.second.c_str());
          RCLCPP_WARN_EXPRESSION(
            params_.logger, affinity_result.first,
            "Async worker thread is successfully pinned to the requested CPU cores!");
        }
        if (params_.scheduling_policy == AsyncSchedulingPolicy::SYNCHRONIZED) {
          execute_synchronized_callback();
        } else {
          execute_detached_callback();
        }
      });
    }
  }

private:
  void execute_synchronized_callback()
  {
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
  }

  void execute_detached_callback()
  {
    if (!params_.clock) {
      throw std::runtime_error(
        "AsyncFunctionHandler: Clock must be set when using DETACHED scheduling policy.");
    }
    if (params_.exec_rate == 0u) {
      throw std::runtime_error(
        "AsyncFunctionHandler: Execution rate must be set when using DETACHED scheduling policy.");
    }

    auto const period = std::chrono::nanoseconds(1'000'000'000 / params_.exec_rate);

    if (pause_thread_) {
      std::unique_lock<std::mutex> lock(async_mtx_);
      async_callback_condition_.wait(
        lock, [this] { return !pause_thread_ || stop_async_callback_; });
    }
    // for calculating the measured period of the loop
    previous_time_ = params_.clock->now();
    std::this_thread::sleep_for(period);
    std::chrono::steady_clock::time_point next_iteration_time{std::chrono::steady_clock::now()};
    while (!stop_async_callback_.load(std::memory_order_relaxed)) {
      {
        std::unique_lock<std::mutex> lock(async_mtx_);
        async_callback_condition_.wait(
          lock, [this] { return !pause_thread_ || stop_async_callback_; });
        if (!stop_async_callback_) {
          // calculate measured period
          auto const current_time = params_.clock->now();
          auto const measured_period = current_time - previous_time_;
          previous_time_ = current_time;
          current_callback_time_ = current_time;
          current_callback_period_ = measured_period;

          const auto start_time = std::chrono::steady_clock::now();
          try {
            async_callback_return_ = async_function_(current_time, measured_period);
          } catch (...) {
            async_exception_ptr_ = std::current_exception();
          }
          last_execution_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - start_time);

          next_iteration_time += period;
          const auto time_now = std::chrono::steady_clock::now();
          if (next_iteration_time < time_now) {
            const double time_diff =
              std::chrono::duration<double, std::milli>(time_now - next_iteration_time).count();
            const double cm_period = 1.e3 / static_cast<double>(params_.exec_rate);
            const int overrun_count = static_cast<int>(std::ceil(time_diff / cm_period));
            if (params_.print_warnings) {
              RCLCPP_WARN_THROTTLE(
                params_.logger, *params_.clock, 1000,
                "Overrun detected! The async callback missed its desired rate of %d Hz. The loop "
                "took %f ms (missed cycles : %d).",
                params_.exec_rate, time_diff + cm_period, overrun_count + 1);
            }
            next_iteration_time += (overrun_count * period);
          }
          std::this_thread::sleep_until(next_iteration_time);
        }
        trigger_in_progress_ = false;
      }
      cycle_end_condition_.notify_all();
    }
  }

  rclcpp::Time current_callback_time_ = rclcpp::Time(0, 0, RCL_CLOCK_UNINITIALIZED);
  rclcpp::Duration current_callback_period_{0, 0};

  std::function<T(const rclcpp::Time &, const rclcpp::Duration &)> async_function_;
  std::function<bool()> trigger_predicate_ = []() { return true; };

  // Async related variables
  std::thread thread_;
  AsyncFunctionHandlerParams params_;
  rclcpp::Time previous_time_{0, 0, RCL_CLOCK_UNINITIALIZED};
  int thread_priority_ = std::numeric_limits<int>::quiet_NaN();
  std::atomic_bool stop_async_callback_{false};
  std::atomic_bool trigger_in_progress_{false};
  std::atomic_bool pause_thread_{false};
  std::atomic<T> async_callback_return_;
  std::condition_variable async_callback_condition_;
  std::condition_variable cycle_end_condition_;
  std::mutex async_mtx_;
  std::atomic<std::chrono::nanoseconds> last_execution_time_;
  std::atomic<double> periodicity_;
  std::exception_ptr async_exception_ptr_;
};
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__ASYNC_FUNCTION_HANDLER_HPP_
