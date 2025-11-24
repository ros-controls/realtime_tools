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
{
public:
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  explicit WaitFreeRealtimePublisher(
    PublisherSharedPtr publisher,
    std::chrono::microseconds sleep_poll_duration = std::chrono::microseconds(1))
  : WaitFreeRealtimePublisher(
      std::make_shared<utils::ROSPublisherWrapper<MessageT>>(publisher), sleep_poll_duration)
  {
  }

  explicit WaitFreeRealtimePublisher(
    std::shared_ptr<utils::PublisherInterface<MessageT>> publisher,
    std::chrono::microseconds sleep_poll_duration = std::chrono::microseconds(1))
  : publisher_(publisher), sleep_poll_duration_(sleep_poll_duration)
  {
  }

  ~WaitFreeRealtimePublisher() { stop(); }

  void stop()
  {
    is_running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  template <bool Allow = kRealtimeSupport, typename = void, typename = std::enable_if_t<!Allow>>
  void start(int thread_priority, const std::vector<int> & cpu_affinity)
  {
    static_assert(
      kRealtimeSupport,
      "WaitFreeRealtimePublisher::start with realtime settings is not supported on this platform.");
  }

  template <bool Allow = kRealtimeSupport, typename = std::enable_if_t<Allow>>
  void start(int thread_priority, const std::vector<int> & cpu_affinity)
  {
    auto result = start_(thread_priority, cpu_affinity);

    if (!result.first) {
      throw std::runtime_error(result.second);
    }
  }

  void start()
  {
    auto result = start_(-1, {});

    if (!result.first) {
      throw std::runtime_error(
        "Failed to start WaitFreeRealtimePublisher thread: " + result.second);
    }
  }

  bool push(const MessageT & msg) { return message_queue_.push(msg); }

  bool running() const { return is_running_; }

private:
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
        publishingLoop();
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

  void publishingLoop()
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

  LockFreeSPSCQueue<MessageT, Capacity> message_queue_;
  std::shared_ptr<utils::PublisherInterface<MessageT>> publisher_;
  std::atomic<bool> is_running_{false};
  const std::chrono::microseconds sleep_poll_duration_;

  std::thread thread_;
};

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__WAIT_FREE_REALTIME_PUBLISHER_HPP_
