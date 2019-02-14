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
#include <inttypes.h>
#include <memory>

// Actionlib
#include <actionlib/server/action_server.h>

namespace realtime_tools
{

template <class Action>
class RealtimeServerGoalHandle
{
private:
  ACTION_DEFINITION(Action);

  typedef actionlib::ServerGoalHandle<Action> GoalHandle;
  typedef boost::shared_ptr<Result> ResultPtr;
  typedef boost::shared_ptr<Feedback> FeedbackPtr;

  uint8_t state_;

  bool req_abort_;
  bool req_cancel_;
  bool req_succeed_;
  ResultConstPtr req_result_;
  FeedbackConstPtr req_feedback_;

public:
  GoalHandle gh_;
  ResultPtr preallocated_result_;  // Preallocated so it can be used in realtime
  FeedbackPtr preallocated_feedback_;  // Preallocated so it can be used in realtime

  RealtimeServerGoalHandle(GoalHandle &gh, const ResultPtr &preallocated_result = ResultPtr((Result*)NULL), const FeedbackPtr &preallocated_feedback = FeedbackPtr((Feedback*)NULL))
    : req_abort_(false),
      req_cancel_(false),
      req_succeed_(false),
      gh_(gh),
      preallocated_result_(preallocated_result),
      preallocated_feedback_(preallocated_feedback)
  {
    if (!preallocated_result_)
      preallocated_result_.reset(new Result);
    if (!preallocated_feedback_)
      preallocated_feedback_.reset(new Feedback);
  }

  void setAborted(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_abort_ = true;
    }
  }

  void setCanceled(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_cancel_ = true;
    }
  }

  void setSucceeded(ResultConstPtr result = ResultConstPtr((Result*)NULL))
  {
    if (!req_succeed_ && !req_abort_ && !req_cancel_)
    {
      req_result_ = result;
      req_succeed_ = true;
    }
  }

  void setFeedback(FeedbackConstPtr feedback = FeedbackConstPtr((Feedback*)NULL))
  {
    req_feedback_ = feedback;
  }

  bool valid()
  {
    return gh_.getGoal() != NULL;
  }

  void runNonRealtime(const ros::TimerEvent &te)
  {
    using namespace actionlib_msgs;
    if (valid())
    {
      actionlib_msgs::GoalStatus gs = gh_.getGoalStatus();
      if (req_abort_ && (gs.status == GoalStatus::ACTIVE ||
                         gs.status == GoalStatus::PREEMPTING))
      {
        if (req_result_)
          gh_.setAborted(*req_result_);
        else
          gh_.setAborted();
      }
      else if (req_cancel_ && gs.status == GoalStatus::PREEMPTING)
      {
        if (req_result_)
          gh_.setCanceled(*req_result_);
        else
          gh_.setCanceled();
      }
      else if (req_succeed_ && (gs.status == GoalStatus::ACTIVE ||
                                gs.status == GoalStatus::PREEMPTING))
      {
        if (req_result_)
          gh_.setSucceeded(*req_result_);
        else
          gh_.setSucceeded();
      }
      if (req_feedback_ && gs.status == GoalStatus::ACTIVE)
      {
        gh_.publishFeedback(*req_feedback_);
      }
    }
  }
};

} // namespace

#endif // header guard
