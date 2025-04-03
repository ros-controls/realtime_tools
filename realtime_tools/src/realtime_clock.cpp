// Copyright (c) 2013, hiDOF, Inc.
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
//    * Neither the name of the hiDOF, Inc. nor the names of its
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
 * Author: Wim Meeussen
 */

#include "realtime_tools/realtime_clock.hpp"

#include <chrono>

#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"

namespace realtime_tools
{
RealtimeClock::RealtimeClock() : RealtimeClock(nullptr, rclcpp::get_logger("realtime_tools")) {}

RealtimeClock::RealtimeClock(rclcpp::Clock::SharedPtr clock)
: RealtimeClock(clock, rclcpp::get_logger("realtime_tools"))
{
}

RealtimeClock::RealtimeClock(rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger)
: clock_(clock), logger_(logger), running_(true), thread_(std::thread(&RealtimeClock::loop, this))
{
}

RealtimeClock::~RealtimeClock()
{
  if (thread_.joinable()) {
    running_ = false;
    thread_.join();
  }
}

rclcpp::Time RealtimeClock::now(const rclcpp::Time & realtime_time)
{
  // Default constructed or given invalid clock, so return zero
  if (!clock_) {
    return rclcpp::Time();
  }

  std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
  if (guard.owns_lock()) {
    // update time offset when we have a new system time measurement in the last cycle
    if (lock_misses_ == 0 && system_time_ != rclcpp::Time()) {
      // get additional offset caused by period of realtime loop
      rclcpp::Duration period_offset(0, 0u);
      if (last_realtime_time_ != rclcpp::Time()) {
        period_offset = (realtime_time - last_realtime_time_) * 0.5;
      }

      if (!initialized_) {
        clock_offset_ = system_time_ + period_offset - realtime_time;
        initialized_ = true;
      } else {
        clock_offset_ =
          clock_offset_ * 0.9999 + (system_time_ + period_offset - realtime_time) * 0.0001;
      }
    }
    system_time_ = rclcpp::Time();
    lock_misses_ = 0;
  } else {
    lock_misses_++;
  }

  last_realtime_time_ = realtime_time;

  // return time
  return realtime_time + clock_offset_;
}

void RealtimeClock::loop()
{
  rclcpp::Rate r(750);
  while (running_) {
#ifdef NON_POLLING
    std::lock_guard<std::mutex> guard(mutex_);
#else
    std::unique_lock<std::mutex> guard(mutex_, std::defer_lock);
    while (!guard.try_lock()) {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
#endif

    // store system time
    system_time_ = clock_->now();

    // warning, using non-locked 'lock_misses_', but it's just for debugging
    if (lock_misses_ > 100) {
      static rclcpp::Time last_warn_time = system_time_;
      if ((system_time_ - last_warn_time).seconds() > 1.0) {
        RCLCPP_WARN(logger_, "Time estimator has trouble transferring data between non-RT and RT");
      }
    }

    // release lock
    guard.unlock();
    r.sleep();
  }
}
}  // namespace realtime_tools
