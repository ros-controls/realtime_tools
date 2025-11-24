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
#include <thread>

#include "rclcpp/publisher.hpp"
#include "realtime_tools/lock_free_queue.hpp"
#include "realtime_tools/utils/publisher_interface.hpp"

namespace realtime_tools
{

template <class MessageT, std::size_t Capacity = 1>
class WaitFreeRealtimePublisher
{
public:
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  explicit WaitFreeRealtimePublisher(PublisherSharedPtr publisher)
  : WaitFreeRealtimePublisher(std::make_shared<utils::ROSPublisherWrapper<MessageT>>(publisher))
  {
  }

  explicit WaitFreeRealtimePublisher(std::shared_ptr<utils::PublisherInterface<MessageT>> publisher)
  : publisher_(publisher), is_running_(true)
  {
    thread_ = std::thread(&WaitFreeRealtimePublisher::publishingLoop, this);

    while (!thread_.joinable()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  ~WaitFreeRealtimePublisher() { stop(); }

  void stop()
  {
    is_running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void start()
  {
    if (!thread_.joinable()) {
      is_running_ = true;
      thread_ = std::thread(&WaitFreeRealtimePublisher::publishingLoop, this);
    }
  }

  bool push(const MessageT & msg) { return message_queue_.push(msg); }

  bool running() const { return is_running_; }

private:
  void publishingLoop()
  {
    while (is_running_) {
      MessageT outgoing;

      if (message_queue_.empty()) {
        // No message to publish, sleep briefly to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::microseconds(10));
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

  std::thread thread_;
  std::atomic<bool> is_running_;
};

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__WAIT_FREE_REALTIME_PUBLISHER_HPP_
