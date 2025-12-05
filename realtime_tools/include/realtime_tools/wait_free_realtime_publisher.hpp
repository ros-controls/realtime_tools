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

#ifndef REALTIME_TOOLS__WAIT_FREE_REALTIME_PUBLISHER_HPP_
#define REALTIME_TOOLS__WAIT_FREE_REALTIME_PUBLISHER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "rclcpp/publisher.hpp"
#include "realtime_tools/lock_free_queue.hpp"
#include "realtime_tools/realtime_helpers.hpp"
#include "realtime_tools/utils/publisher_interface.hpp"

namespace realtime_tools
{

#if defined(_WIN32) || defined(__APPLE__)
static constexpr bool kRealtimeSupport = false;
#else
static constexpr bool kRealtimeSupport = true;
#endif

template <class MessageT, std::size_t Capacity = 2>
class WaitFreeRealtimePublisher
/**
 * @brief A wait-free realtime-safe publisher using a lock-free queue
 *
 * This class provides a mechanism to publish ROS messages from a realtime context
 * without blocking or using locks. Messages are pushed to a lock-free single-producer
 * single-consumer (SPSC) queue from the realtime thread, and a separate non-realtime
 * thread handles the actual publishing operation.
 *
 * The publisher uses a polling mechanism with configurable sleep duration to check
 * for new messages in the queue, balancing CPU usage with publishing latency.
 *
 * Default settings are derived from benchmarks, and are selected to balance low latency
 * while still having high publish rates. Use benchmarks to find your optimal settings.
 *
 * @tparam MessageT The ROS message type to publish
 * @tparam Capacity The maximum number of messages that can be queued (default: 2)
 *
 * Example usage:
 * @code
 *   auto ros_pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
 *   WaitFreeRealtimePublisher<std_msgs::msg::String> rt_pub(ros_pub);
 *   rt_pub.start();  // Start the publishing thread
 *
 *   // From realtime context:
 *   std_msgs::msg::String msg;
 *   msg.data = "Hello";
 *   rt_pub.push(msg);  // Wait-free operation
 * @endcode
 */
{
public:
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  /**
   * @brief Constructor that wraps a ROS publisher
   *
   * Creates a wait-free realtime publisher by wrapping a standard ROS publisher.
   * The publishing thread is not started automatically; call start() to begin publishing.
   *
   * @param publisher The ROS publisher to wrap for realtime-safe publishing
   * @param sleep_poll_duration The duration to sleep when the queue is empty, in microseconds
   *                            (default: 1 microsecond). Lower values reduce latency but
   *                            increase CPU usage; higher values reduce CPU usage but may
   *                            increase publishing latency.
   */
  explicit WaitFreeRealtimePublisher(
    PublisherSharedPtr publisher,
    std::chrono::microseconds sleep_poll_duration = std::chrono::microseconds(1))
  : WaitFreeRealtimePublisher(
      std::make_shared<utils::ROSPublisherWrapper<MessageT>>(publisher), sleep_poll_duration)
  {
  }

  /**
   * @brief Constructor that accepts a custom publisher interface
   *
   * Creates a wait-free realtime publisher using a custom publisher implementation.
   * Useful for testing or alternative publishing implementations.
   * The publishing thread is not started automatically; call start() to begin publishing.
   *
   * @param publisher A shared pointer to a publisher interface implementation
   * @param sleep_poll_duration The duration to sleep when the queue is empty, in microseconds
   *                            (default: 1 microsecond)
   */
  explicit WaitFreeRealtimePublisher(
    std::shared_ptr<utils::PublisherInterface<MessageT>> publisher,
    std::chrono::microseconds sleep_poll_duration = std::chrono::microseconds(1))
  : publisher_(publisher), sleep_poll_duration_(sleep_poll_duration)
  {
  }

  /**
   * @brief Destructor
   *
   * Automatically stops the publishing thread and waits for it to complete.
   */
  ~WaitFreeRealtimePublisher() { stop(); }

  /**
   * @brief Stop the publishing thread
   *
   * Signals the publishing thread to exit and waits for it to complete.
   * This method is idempotent and can be called multiple times safely.
   * It is automatically called by the destructor.
   */
  void stop()
  {
    is_running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  /**
   * @brief Start the publishing thread with realtime settings (not supported on Windows/macOS)
   *
   * This overload is disabled on platforms that don't support realtime scheduling.
   * Attempting to call this method on unsupported platforms will result in a compile-time error.
   *
   * @tparam Allow Template parameter used for SFINAE to disable this method on unsupported platforms
   * @param thread_priority The realtime thread priority (ignored on unsupported platforms)
   * @param cpu_affinity The CPU affinity mask (ignored on unsupported platforms)
   */
  template <bool Allow = kRealtimeSupport, typename = void, typename = std::enable_if_t<!Allow>>
  void start(int thread_priority, const std::vector<int> & cpu_affinity)
  {
    static_assert(
      kRealtimeSupport,
      "WaitFreeRealtimePublisher::start with realtime settings is not supported on this platform.");
  }

  /**
   * @brief Start the publishing thread with realtime scheduling and CPU affinity
   *
   * Starts the publishing thread with SCHED_FIFO scheduling policy and the specified
   * thread priority and CPU affinity. This method is only available on Linux platforms.
   *
   * @param thread_priority The realtime thread priority for SCHED_FIFO scheduling (1-99)
   * @param cpu_affinity Vector of CPU core IDs to which the thread should be pinned
   * @throws std::runtime_error If the thread fails to start or realtime settings cannot be applied
   */
  template <bool Allow = kRealtimeSupport, typename = std::enable_if_t<Allow>>
  void start(int thread_priority, const std::vector<int> & cpu_affinity)
  {
    auto result = start_(thread_priority, cpu_affinity);

    if (!result.first) {
      throw std::runtime_error(result.second);
    }
  }

  /**
   * @brief Start the publishing thread with default settings
   *
   * Starts the publishing thread without realtime scheduling or CPU affinity.
   * The thread will run with the default scheduling policy of the system.
   *
   * @throws std::runtime_error If the thread fails to start
   */
  void start()
  {
    auto result = start_(-1, {});

    if (!result.first) {
      throw std::runtime_error(
        "Failed to start WaitFreeRealtimePublisher thread: " + result.second);
    }
  }

  /**
   * @brief Push a message to the publishing queue (wait-free operation)
   *
   * This method is wait-free and can be safely called from a realtime context.
   * If the queue is full, the message will be dropped and the method returns false.
   *
   * @param msg The message to publish
   * @return true if the message was successfully queued, false if the queue was full
   */
  bool push(const MessageT & msg) { return message_queue_.push(msg); }

  /**
   * @brief Check if the publishing thread is running
   *
   * @return true if the publishing thread is active, false otherwise
   */
  bool running() const { return is_running_; }

private:
  /**
   * @brief Internal method to start the publishing thread with optional realtime settings
   *
   * This method handles the actual thread creation and configuration. It attempts to
   * apply realtime scheduling and CPU affinity if requested.
   *
   * @param thread_priority The realtime thread priority (-1 for default scheduling, 1-99 for SCHED_FIFO)
   * @param cpu_affinity Vector of CPU core IDs for thread affinity (empty for no affinity)
   * @return A pair containing success status and a descriptive message
   */
  std::pair<bool, std::string> start_(int thread_priority, const std::vector<int> & cpu_affinity)
  {
    if (!thread_.joinable()) {
      is_running_ = true;

      // Check if realtime settings are applied. Note: condition variable complexity is to
      // workaround current API limitation of realtime_helpers.
      // FIXME: is it a problem if these go out of scope?
      std::mutex notify_mtx;
      std::condition_variable notify_cv;
      bool ready = false;
      bool settings_applied = false;
      std::string msg = "Did not apply settings.";

      thread_ = std::thread([this, &notify_mtx, &notify_cv, &settings_applied, &msg, &ready,
                             thread_priority, cpu_affinity]() {
        {
          std::unique_lock<std::mutex> lock(notify_mtx);
          if (thread_priority > 0 && !configure_sched_fifo(thread_priority)) {
            msg = "Failed to set SCHED_FIFO with priority " + std::to_string(thread_priority);
            settings_applied = false;
            notify_cv.notify_one();
            ready = true;
            return;
          } else {
            settings_applied = true;
            msg = "Settings applied successfully.";
            notify_cv.notify_one();
            ready = true;
          }
        }

        // Publishing loop
        publishing_loop();
      });

      if (!cpu_affinity.empty()) {
        auto affinity_result = set_thread_affinity(thread_.native_handle(), cpu_affinity);
        if (!affinity_result.first) {
          stop();
          return affinity_result;
        }
      }

      // Wait for thread scheduling/priorities to be applied, then check for result
      {
        std::unique_lock<std::mutex> lock(notify_mtx);
        notify_cv.wait(lock, [&ready]() { return ready; });

        if (!settings_applied) {
          stop();
          return {false, msg};
        }
      }

      return {true, "Thread started."};
    }

    return {true, "Thread is already running."};
  }

  /**
   * @brief The main publishing loop executed by the background thread
   *
   * This method runs in a separate thread and continuously polls the message queue.
   * When a message is available, it pops the message and publishes it using the
   * configured publisher. When the queue is empty, it sleeps for the configured
   * duration to avoid busy-waiting.
   */
  void publishing_loop()
  {
    while (is_running_) {
      MessageT outgoing;

      if (message_queue_.empty()) {
        // No message to publish, sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(sleep_poll_duration_);
        continue;
      }

      if (!message_queue_.pop(outgoing)) {
        // Failed to pop message, continue to next iteration
        continue;
      }

      // Sends the outgoing message
      publisher_->publish(outgoing);
    }
  }

  /// Lock-free single-producer single-consumer queue for storing messages
  LockFreeSPSCQueue<MessageT, Capacity> message_queue_;

  /// Publisher interface used to publish messages
  std::shared_ptr<utils::PublisherInterface<MessageT>> publisher_;

  /// Atomic flag indicating whether the publishing thread is running
  std::atomic<bool> is_running_{false};

  /// Duration to sleep when the queue is empty (configurable to balance CPU usage vs latency)
  const std::chrono::microseconds sleep_poll_duration_;

  /// Background thread that handles message publishing
  std::thread thread_;
};

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__WAIT_FREE_REALTIME_PUBLISHER_HPP_
