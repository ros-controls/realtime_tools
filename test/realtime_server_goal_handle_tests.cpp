/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gmock/gmock.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/TwoIntsAction.h>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

using realtime_tools::RealtimeServerGoalHandle;
using TwoIntsActionClient = actionlib::SimpleActionClient<actionlib::TwoIntsAction>;
using TwoIntsActionServer = actionlib::ActionServer<actionlib::TwoIntsAction>;

const size_t ATTEMPTS = 10;
const std::chrono::milliseconds DELAY(250);

struct ActionCallback
{
  using GoalHandle = TwoIntsActionServer::GoalHandle;
  bool have_handle_ = false;
  GoalHandle handle_;
  std::mutex mtx_;

  void goal_callback(GoalHandle handle)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    handle_ = handle;
    handle_.setAccepted();
    have_handle_ = true;
  }

  void cancel_callback(GoalHandle handle)
  {
  }

  bool wait_for_handle()
  {
    for (size_t i = 0; i < ATTEMPTS; ++i)
    {
      ros::spinOnce();
      std::this_thread::sleep_for(DELAY);
      std::unique_lock<std::mutex> lock(mtx_);
      if (have_handle_)
      {
        break;
      }
    }
    std::unique_lock<std::mutex> lock(mtx_);
    return have_handle_;
  }
};

struct FeedbackCallback
{
  bool have_feedback_ = false;
  std::mutex mtx_;

  using FeedbackConstPtr = TwoIntsActionServer::FeedbackConstPtr;
  void feedback_callback(const FeedbackConstPtr &)
  {
    std::unique_lock<std::mutex> lock(mtx_);
    have_feedback_ = true;
  }

  bool wait_for_feedback()
  {
    for (size_t i = 0; i < ATTEMPTS; ++i)
    {
      ros::spinOnce();
      std::this_thread::sleep_for(DELAY);
      std::unique_lock<std::mutex> lock(mtx_);
      if (have_feedback_)
      {
        break;
      }
    }
    std::unique_lock<std::mutex> lock(mtx_);
    return have_feedback_;
  }
};

std::shared_ptr<TwoIntsActionClient>
send_goal(
  const std::string & server_name,
  FeedbackCallback * cb = nullptr)
{
  auto ac = std::make_shared<TwoIntsActionClient>(server_name, true);

  for (size_t i = 0; i < ATTEMPTS && !ac->isServerConnected(); ++i)
  {
    ros::spinOnce();
    std::this_thread::sleep_for(DELAY);
  }
  if (!ac->isServerConnected())
  {
    ac.reset();
  } else {
    actionlib::TwoIntsGoal goal;
    goal.a = 2;
    goal.b = 3;
    ac->sendGoal(
      goal,
      TwoIntsActionClient::SimpleDoneCallback(),
      TwoIntsActionClient::SimpleActiveCallback(),
      std::bind(&FeedbackCallback::feedback_callback, cb, std::placeholders::_1));
  }
  return ac;
}

bool wait_for_result(std::shared_ptr<TwoIntsActionClient> ac)
{
  for (int i = 0; i < ATTEMPTS; ++i)
  {
    ros::spinOnce();
    if (ac->getState().isDone())
    {
      return true;
    }
    std::this_thread::sleep_for(DELAY);
  }
  return false;
}

std::shared_ptr<TwoIntsActionServer>
make_server(const std::string & server_name, ActionCallback & callbacks)
{
  ros::NodeHandle nh;
  auto as = std::make_shared<TwoIntsActionServer>(
      nh, server_name,
      std::bind(&ActionCallback::goal_callback, &callbacks, std::placeholders::_1),
      std::bind(&ActionCallback::cancel_callback, &callbacks, std::placeholders::_1), false);
  as->start();
  return as;
}

TEST(RealtimeServerGoalHandle, set_aborted)
{
  ActionCallback callbacks;
  const std::string server_name("set_aborted");
  auto as = make_server(server_name, callbacks);

  auto client = send_goal(server_name);
  ASSERT_NE(nullptr, client.get());

  ASSERT_TRUE(callbacks.wait_for_handle());
  RealtimeServerGoalHandle<actionlib::TwoIntsAction> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());

  {
    actionlib::TwoIntsResult::Ptr result(new actionlib::TwoIntsResult);
    result->sum = 42;
    rt_handle.setAborted(result);
    rt_handle.runNonRealtime(ros::TimerEvent());
  }

  ASSERT_TRUE(wait_for_result(client));
  auto state = client->getState();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::ABORTED) << state.toString();
  auto result = client->getResult();
  ASSERT_NE(nullptr, result.get());
  EXPECT_EQ(42, result->sum);
}

TEST(RealtimeServerGoalHandle, set_canceled)
{
  ActionCallback callbacks;
  const std::string server_name("set_canceled");
  auto as = make_server(server_name, callbacks);

  auto client = send_goal(server_name);
  ASSERT_NE(nullptr, client.get());

  ASSERT_TRUE(callbacks.wait_for_handle());
  RealtimeServerGoalHandle<actionlib::TwoIntsAction> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());

  // Cancel and wait for server to learn about that
  client->cancelGoal();
  for (size_t i = 0; i < ATTEMPTS; ++i)
  {
    actionlib_msgs::GoalStatus gs = callbacks.handle_.getGoalStatus();
    if (gs.status == actionlib_msgs::GoalStatus::PREEMPTING) {
      break;
    }
    ros::spinOnce();
    std::this_thread::sleep_for(DELAY);
  }
  ASSERT_EQ(callbacks.handle_.getGoalStatus().status, actionlib_msgs::GoalStatus::PREEMPTING);

  {
    actionlib::TwoIntsResult::Ptr result(new actionlib::TwoIntsResult);
    result->sum = 1234;
    rt_handle.setCanceled(result);
    rt_handle.runNonRealtime(ros::TimerEvent());
  }

  ASSERT_TRUE(wait_for_result(client));
  auto state = client->getState();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::PREEMPTED) << state.toString();
  auto result = client->getResult();
  ASSERT_NE(nullptr, result.get());
  EXPECT_EQ(1234, result->sum);
}

TEST(RealtimeServerGoalHandle, set_succeeded)
{
  ActionCallback callbacks;
  const std::string server_name("set_succeeded");
  auto as = make_server(server_name, callbacks);

  auto client = send_goal(server_name);
  ASSERT_NE(nullptr, client.get());

  ASSERT_TRUE(callbacks.wait_for_handle());
  RealtimeServerGoalHandle<actionlib::TwoIntsAction> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());

  {
    actionlib::TwoIntsResult::Ptr result(new actionlib::TwoIntsResult);
    result->sum = 321;
    rt_handle.setSucceeded(result);
    rt_handle.runNonRealtime(ros::TimerEvent());
  }

  ASSERT_TRUE(wait_for_result(client));
  auto state = client->getState();
  EXPECT_EQ(state, actionlib::SimpleClientGoalState::SUCCEEDED) << state.toString();
  auto result = client->getResult();
  ASSERT_NE(nullptr, result.get());
  EXPECT_EQ(321, result->sum);
}

TEST(RealtimeServerGoalHandle, send_feedback)
{
  ActionCallback callbacks;
  FeedbackCallback client_callbacks;
  const std::string server_name("send_feedback");
  auto as = make_server(server_name, callbacks);

  auto client = send_goal(server_name, &client_callbacks);
  ASSERT_NE(nullptr, client.get());

  ASSERT_TRUE(callbacks.wait_for_handle());
  RealtimeServerGoalHandle<actionlib::TwoIntsAction> rt_handle(callbacks.handle_);
  ASSERT_TRUE(rt_handle.valid());

  {
    actionlib::TwoIntsFeedback::Ptr fb(new actionlib::TwoIntsFeedback());
    rt_handle.setFeedback(fb);
    rt_handle.runNonRealtime(ros::TimerEvent());
  }

  EXPECT_TRUE(client_callbacks.wait_for_feedback());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "realtime_server_goal_handle_tests");
  return RUN_ALL_TESTS();
}
