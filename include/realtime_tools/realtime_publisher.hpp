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

  /// The msg_ variable contains the data that will get published on the ROS topic.
  MessageT msg_;

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

  [[deprecated(
    "Use constructor with rclcpp::Publisher<T>::SharedPtr instead - this class does not make sense "
    "without a real publisher")]]
  RealtimePublisher()
  : is_running_(false), keep_running_(false), turn_(State::LOOP_NOT_STARTED)
  {
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
  * \brief Try to acquire the data lock for non-realtime message publishing
  *
  * It first checks if the current state allows non-realtime message publishing (turn_ == REALTIME)
  * and then attempts to lock
  *
  * \return true if the lock was successfully acquired, false otherwise
  */
  bool trylock()
  {
    if (turn_.load(std::memory_order_acquire) == State::REALTIME && msg_mutex_.try_lock()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * \brief Try to get the data lock from realtime and publish the given message
   *
   * Tries to gain unique access to msg_ variable. If this succeeds
   * update the msg_ variable and call unlockAndPublish
   *
   * \param [in] msg The message to publish
   * \return false in case no lock for the realtime variable is acquired. This implies the message will not be published.
   */
  bool tryPublish(const MessageT & msg)
  {
    if (!trylock()) {
      return false;
    }

    msg_ = msg;
    unlockAndPublish();
    return true;
  }

  /**
   * \brief Unlock the msg_ variable for the non-realtime thread to start publishing
   *
   * After a successful trylock and after the data is written to the mgs_
   * variable, the lock has to be released for the message to get
   * published on the specified topic.
   */
  void unlockAndPublish()
  {
    turn_.store(State::NON_REALTIME, std::memory_order_release);
    unlock();
  }

  /**
   * \brief Acquire the data lock
   *
   * This blocking call acquires exclusive access to the msg_ variable.
   * Use trylock() for non-blocking attempts to acquire the lock.
   */
  void lock() { msg_mutex_.lock(); }

  /**
   * \brief Unlocks the data without publishing anything
   *
   */
  void unlock()
  {
    msg_mutex_.unlock();
    updated_cond_.notify_one();
  }

private:
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

  std::mutex msg_mutex_;  // Protects msg_
  std::condition_variable updated_cond_;

  enum class State : int { REALTIME, NON_REALTIME, LOOP_NOT_STARTED };
  std::atomic<State> turn_;  // Who's turn is it to use msg_?
};

template <class MessageT>
using RealtimePublisherSharedPtr = std::shared_ptr<RealtimePublisher<MessageT>>;

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__REALTIME_PUBLISHER_HPP_
