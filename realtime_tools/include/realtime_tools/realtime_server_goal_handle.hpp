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

/// \author Stuart Glaser

#ifndef REALTIME_TOOLS__REALTIME_SERVER_GOAL_HANDLE_HPP_
#define REALTIME_TOOLS__REALTIME_SERVER_GOAL_HANDLE_HPP_

#include <atomic>
#include <memory>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

// prio_inherit_mutex uses pthread APIs not available on Windows
#ifndef _WIN32
#include "realtime_tools/mutex.hpp"
#else
#include <mutex>
#endif

namespace realtime_tools
{
#ifndef _WIN32
using rt_server_goal_handle_mutex = prio_inherit_mutex;
#else
using rt_server_goal_handle_mutex = std::mutex;
#endif
template <class Action>
class RealtimeServerGoalHandle
{
private:
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using ResultSharedPtr = typename Action::Result::SharedPtr;
  using FeedbackSharedPtr = typename Action::Feedback::SharedPtr;

  std::atomic<bool> req_abort_;
  std::atomic<bool> req_cancel_;
  std::atomic<bool> req_succeed_;
  std::atomic<bool> req_execute_;

  rt_server_goal_handle_mutex mutex_;
  ResultSharedPtr req_result_;
  FeedbackSharedPtr req_feedback_;
  rclcpp::Logger logger_;

public:
  std::shared_ptr<GoalHandle> gh_;
  ResultSharedPtr preallocated_result_;      // Preallocated so it can be used in realtime
  FeedbackSharedPtr preallocated_feedback_;  // Preallocated so it can be used in realtime

  explicit RealtimeServerGoalHandle(
    std::shared_ptr<GoalHandle> & gh, const ResultSharedPtr & preallocated_result = nullptr,
    const FeedbackSharedPtr & preallocated_feedback = nullptr)
  : RealtimeServerGoalHandle(
      gh, preallocated_result, preallocated_feedback, rclcpp::get_logger("realtime_tools"))
  {
  }

  RealtimeServerGoalHandle(
    std::shared_ptr<GoalHandle> & gh, const ResultSharedPtr & preallocated_result,
    const FeedbackSharedPtr & preallocated_feedback, rclcpp::Logger logger)
  : req_abort_(false),
    req_cancel_(false),
    req_succeed_(false),
    req_execute_(false),
    logger_(logger),
    gh_(gh),
    preallocated_result_(preallocated_result),
    preallocated_feedback_(preallocated_feedback)
  {
    if (!preallocated_result_) {
      preallocated_result_.reset(new typename Action::Result);
    }
    if (!preallocated_feedback_) {
      preallocated_feedback_.reset(new typename Action::Feedback);
    }
  }

  void setAborted(ResultSharedPtr result = nullptr)
  {
    if (
      req_execute_.load(std::memory_order_acquire) &&
      !req_succeed_.load(std::memory_order_acquire) &&
      !req_abort_.load(std::memory_order_acquire) && !req_cancel_.load(std::memory_order_acquire)) {
      std::lock_guard<rt_server_goal_handle_mutex> guard(mutex_);

      req_result_ = result;
      req_abort_.store(true, std::memory_order_release);
    }
  }

  void setCanceled(ResultSharedPtr result = nullptr)
  {
    if (
      req_execute_.load(std::memory_order_acquire) &&
      !req_succeed_.load(std::memory_order_acquire) &&
      !req_abort_.load(std::memory_order_acquire) && !req_cancel_.load(std::memory_order_acquire)) {
      std::lock_guard<rt_server_goal_handle_mutex> guard(mutex_);

      req_result_ = result;
      req_cancel_.store(true, std::memory_order_release);
    }
  }

  void setSucceeded(ResultSharedPtr result = nullptr)
  {
    if (
      req_execute_.load(std::memory_order_acquire) &&
      !req_succeed_.load(std::memory_order_acquire) &&
      !req_abort_.load(std::memory_order_acquire) && !req_cancel_.load(std::memory_order_acquire)) {
      std::lock_guard<rt_server_goal_handle_mutex> guard(mutex_);

      req_result_ = result;
      req_succeed_.store(true, std::memory_order_release);
    }
  }

  /**
   * @brief Set feedback to be published by runNonRealtime().
   *
   * This method uses a non-blocking lock acquisition with try_to_lock() and is safe
   * to call from real-time context without causing thread blocking or priority inversion.
   * If the lock cannot be acquired (because runNonRealtime() is holding it), the update
   * is silently skipped to avoid blocking the real-time thread.
   *
   * The feedback pointer is stored and the non-RT thread will publish it on the next
   * runNonRealtime() call, provided the goal handle is in executing state.
   *
   * @param feedback Shared pointer to feedback message. Can be nullptr to clear pending
   *        feedback. If a valid pointer is provided, the caller must ensure the feedback
   *        object remains valid until runNonRealtime() processes it. Using a
   *        preallocated feedback message (stored in preallocated_feedback_) is
   *        recommended to avoid dynamic allocations in the real-time thread.
   *
   * @return true if the lock was acquired and feedback was successfully set,
   *         false if the lock could not be acquired (feedback update was dropped).
   *
   * @note If the mutex lock cannot be acquired, the feedback update is dropped without
   *       notification. This is intentional to preserve real-time guarantees. Check the
   *       return value if you need to know whether the update succeeded.
   *
   * @note Feedback is only published if the goal is currently executing. If feedback
   *       is set after the goal transitions out of executing state, it will be discarded.
   *
   * @see runNonRealtime() for the counterpart that publishes the feedback.
   * @see preallocated_feedback_ for recommended pre-allocated feedback usage.
   */
  bool setFeedback(FeedbackSharedPtr feedback = nullptr)
  {
    std::unique_lock<rt_server_goal_handle_mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock()) {
      req_feedback_ = feedback;
      return true;
    }
    return false;
  }

  void execute()
  {
    if (
      !req_succeed_.load(std::memory_order_acquire) &&
      !req_abort_.load(std::memory_order_acquire) && !req_cancel_.load(std::memory_order_acquire)) {
      std::lock_guard<rt_server_goal_handle_mutex> guard(mutex_);
      req_execute_.store(true, std::memory_order_release);
    }
  }

  bool valid() { return nullptr != gh_.get(); }

  void runNonRealtime()
  {
    if (!valid()) {
      return;
    }

    std::lock_guard<rt_server_goal_handle_mutex> guard(mutex_);

    try {
      if (
        req_execute_.load(std::memory_order_acquire) && !gh_->is_executing() && gh_->is_active() &&
        !gh_->is_canceling()) {
        gh_->execute();
      }
      if (req_abort_.load(std::memory_order_acquire) && gh_->is_executing()) {
        gh_->abort(req_result_);
        req_abort_.store(false, std::memory_order_release);
      }
      if (req_cancel_.load(std::memory_order_acquire) && gh_->is_active()) {
        gh_->canceled(req_result_);
        req_cancel_.store(false, std::memory_order_release);
      }
      if (req_succeed_.load(std::memory_order_acquire) && !gh_->is_canceling()) {
        gh_->succeed(req_result_);
        req_succeed_.store(false, std::memory_order_release);
      }
      if (req_feedback_ && gh_->is_executing()) {
        gh_->publish_feedback(req_feedback_);
      }
    } catch (const rclcpp::exceptions::RCLErrorBase & e) {
      // Likely invalid state transition
      RCLCPP_WARN(logger_, "%s", e.formatted_message.c_str());
    }
  }
};

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__REALTIME_SERVER_GOAL_HANDLE_HPP_
