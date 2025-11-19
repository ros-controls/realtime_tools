// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * Publishing ROS messages is difficult, as the publish function is
 * not realtime safe.  This class provides the proper locking so that
 * you can call publish in realtime and a separate (non-realtime)
 * thread will ensure that the message gets published over ROS.
 *
 * Author: Stuart Glaser
 */
#ifndef REALTIME_TOOLS__REALTIME_PUBLISHER_HPP_
#define REALTIME_TOOLS__REALTIME_PUBLISHER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/publisher.hpp"

namespace realtime_tools
{
template <class MessageT>
class RealtimePublisher
{
public:
  /// Provide various typedefs to resemble the rclcpp::Publisher type
  using PublisherType = rclcpp::Publisher<MessageT>;
  using PublisherSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;

  using PublishedType = typename rclcpp::TypeAdapter<MessageT>::custom_type;
  using ROSMessageType = typename rclcpp::TypeAdapter<MessageT>::ros_message_type;

  RCLCPP_SMART_PTR_DEFINITIONS(RealtimePublisher<MessageT>)

  /**
   * \brief Constructor for the realtime publisher
   *
   * Starts a dedicated thread for message publishing.
   * The publishing thread runs the publishingLoop() function to handle message
   * delivery in a non-realtime context.
   *
   * \param publisher the ROS publisher to wrap
   */
  explicit RealtimePublisher(PublisherSharedPtr publisher)
  : publisher_(publisher), is_running_(false), keep_running_(true), turn_(State::LOOP_NOT_STARTED)
  {
    thread_ = std::thread(&RealtimePublisher::publishingLoop, this);

    // Wait for the thread to be ready before proceeding
    // This is important to ensure that the thread is properly initialized and ready to handle
    // messages before any other operations are performed on the RealtimePublisher instance.
    while (!thread_.joinable() ||
           turn_.load(std::memory_order_acquire) == State::LOOP_NOT_STARTED) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  /// Destructor
  ~RealtimePublisher()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("realtime_tools"), "Waiting for publishing thread to stop....");
    stop();
    while (is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("realtime_tools"), "Publishing thread stopped, joining thread....");
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  /**
   * \brief Stop the realtime publisher
   *
   * Signals the publishing thread to exit by setting keep_running_ to false
   * and notifying the condition variable. This allows the publishing loop
   * to break out of its wait state and exit cleanly.
   */
  void stop()
  {
    {
      std::unique_lock<std::mutex> lock(msg_mutex_);
      keep_running_ = false;
    }
    updated_cond_.notify_one();  // So the publishing loop can exit
  }

  /**
   * \brief Check if the realtime publisher is in a state to publish messages
   * \return true if the publisher is in a state to publish messages
  */
  bool can_publish() const
  {
    std::unique_lock<std::mutex> lock(msg_mutex_, std::try_to_lock);
    return can_publish(lock);
  }

  /**
   * \brief Try to publish the given message
   *
   * This method attempts to publish the given message if the publisher is in a state to do so.
   * It uses a try_lock to avoid blocking if the mutex is already held by another thread.
   *
   * \param [in] msg The message to publish
   * \return true if the message was successfully published, false otherwise
   */
  bool try_publish(const MessageT & msg)
  {
    std::unique_lock<std::mutex> lock(msg_mutex_, std::try_to_lock);
    if (can_publish(lock)) {
      {
        std::unique_lock<std::mutex> scoped_lock(std::move(lock));
        msg_ = msg;
        turn_.store(State::NON_REALTIME, std::memory_order_release);
      }
      updated_cond_.notify_one();  // Notify the publishing thread
      return true;
    }
    return false;
  }

  /**
   * \brief Get the thread object for the publishing thread.
   *
   * This can be used to set thread properties.
   */
  std::thread & get_thread() { return thread_; }

  /**
   * \brief Get the thread object for the publishing thread.
   *
   * This can be used to set thread properties.
   */
  const std::thread & get_thread() const { return thread_; }

  /**
   * \brief Get the mutex protecting the stored message.
   */
  std::mutex & get_mutex() { return msg_mutex_; }

  /**
   * \brief Get the mutex protecting the stored message.
   */
  const std::mutex & get_mutex() const { return msg_mutex_; }

private:
  /**
   * \brief Check if the realtime publisher is in a state to publish messages
   * \param lock A unique_lock that is already acquired on the msg_mutex_
   * \return true if the publisher is in a state to publish messages
  */
  bool can_publish(std::unique_lock<std::mutex> & lock) const
  {
    return turn_.load(std::memory_order_acquire) == State::REALTIME && lock.owns_lock();
  }

  // non-copyable
  RealtimePublisher(const RealtimePublisher &) = delete;
  RealtimePublisher & operator=(const RealtimePublisher &) = delete;

  bool is_running() const { return is_running_; }

  /**
   * \brief Publishing loop (runs in separate thread)
   *
   * This is the main loop for the non-realtime publishing thread. It:
   * 1. Waits for new messages (State::NON_REALTIME)
   * 2. Copies the message data
   * 3. Publishes the message through the ROS publisher
   * 4. Returns to State::REALTIME to allow realtime updates
   *
   * The loop continues until keep_running_ is set to false.
   */
  void publishingLoop()
  {
    is_running_ = true;

    while (keep_running_) {
      MessageT outgoing;

      {
        turn_.store(State::REALTIME, std::memory_order_release);
        // Locks msg_ and copies it to outgoing
        std::unique_lock<std::mutex> lock_(msg_mutex_);
        updated_cond_.wait(lock_, [&] { return turn_ == State::NON_REALTIME || !keep_running_; });
        outgoing = msg_;
      }

      // Sends the outgoing message
      if (keep_running_) {
        publisher_->publish(outgoing);
      }
    }
    is_running_ = false;
  }

  PublisherSharedPtr publisher_;
  std::atomic<bool> is_running_;
  std::atomic<bool> keep_running_;

  std::thread thread_;

  MessageT msg_;

  mutable std::mutex msg_mutex_;  // Protects msg_
  std::condition_variable updated_cond_;

  enum class State : int { REALTIME, NON_REALTIME, LOOP_NOT_STARTED };
  std::atomic<State> turn_;  // Who's turn is it to use msg_?
};

template <class MessageT>
using RealtimePublisherSharedPtr = std::shared_ptr<RealtimePublisher<MessageT>>;

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__REALTIME_PUBLISHER_HPP_
