///////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Stuart Glaser

#ifndef REALTIME_TOOLS__REALTIME_SERVER_GOAL_HANDLE_H
#define REALTIME_TOOLS__REALTIME_SERVER_GOAL_HANDLE_H

// Standard
#include <memory>

#include <rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/server_goal_handle.hpp>

namespace realtime_tools
{

template <class Action>
class RealtimeServerGoalHandle
{
private:

  typedef rclcpp_action::ServerGoalHandle<Action> GoalHandle;

  uint8_t state_;

  bool req_abort_;
  bool req_cancel_;
  bool req_succeed_;
  bool req_execute_;
  typename Action::Result::SharedPtr req_result_;
  typename Action::Feedback::SharedPtr req_feedback_;
  rclcpp::Logger logger_;

public:
  std::shared_ptr<GoalHandle> gh_;
  typename Action::Result::SharedPtr preallocated_result_;  // Preallocated so it can be used in realtime
  typename Action::Feedback::SharedPtr preallocated_feedback_;  // Preallocated so it can be used in realtime

  RealtimeServerGoalHandle(
    std::shared_ptr<GoalHandle> &gh,
    const typename Action::Result::SharedPtr &preallocated_result = nullptr,
    const typename Action::Feedback::SharedPtr &preallocated_feedback = nullptr)
    : RealtimeServerGoalHandle(gh, preallocated_result, preallocated_feedback, rclcpp::get_logger("realtime_tools"))
  {
  }

  RealtimeServerGoalHandle(std::shared_ptr<GoalHandle> &gh, const typename Action::Result::SharedPtr &preallocated_result, const typename Action::Feedback::SharedPtr &preallocated_feedback, rclcpp::Logger logger)
    : req_abort_(false),
      req_cancel_(false),
      req_succeed_(false),
      req_execute_(false),
      gh_(gh),
      preallocated_result_(preallocated_result),
      preallocated_feedback_(preallocated_feedback),
      logger_(logger)
  {
    if (!preallocated_result_)
      preallocated_result_.reset(new typename Action::Result);
    if (!preallocated_feedback_)
      preallocated_feedback_.reset(new typename Action::Feedback);
  }

  void setAborted(typename Action::Result::SharedPtr result = nullptr)
  {
    if (req_execute_ && !req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_abort_ = true;
    }
  }

  void setCanceled(typename Action::Result::SharedPtr result = nullptr)
  {
    if (req_execute_ && !req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_cancel_ = true;
    }
  }

  void setSucceeded(typename Action::Result::SharedPtr result = nullptr)
  {
    if (req_execute_ && !req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_succeed_ = true;
    }
  }

  void setFeedback(typename Action::Feedback::SharedPtr feedback = nullptr)
  {
    req_feedback_ = feedback;
  }

  void execute()
  {
    if (!req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_execute_ = true;
    }
  }

  bool valid()
  {
    return nullptr != gh_.get();
  }

  void runNonRealtime()
  {
    if (valid())
    {
      try
      {
        if (req_execute_ && !gh_->is_executing() && gh_->is_active() && !gh_->is_canceling())
        {
          gh_->execute();
        }
        if (req_abort_ && gh_->is_executing())
        {
          gh_->abort(req_result_);
        }
        if (req_cancel_ && gh_->is_active())
        {
          gh_->canceled(req_result_);
        }
        if (req_succeed_ && !gh_->is_canceling())
        {
          gh_->succeed(req_result_);
        }
        if (req_feedback_ && gh_->is_executing())
        {
          gh_->publish_feedback(req_feedback_);
        }
      }
      catch (const rclcpp::exceptions::RCLErrorBase & e)
      {
        // Likely invalid state transition
        RCLCPP_WARN(logger_, e.formatted_message);
      }
    }
  }
};

} // namespace

#endif // header guard
