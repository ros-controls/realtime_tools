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

#ifndef REALTIME_TOOLS__REALTIME_CLOCK_H_
#define REALTIME_TOOLS__REALTIME_CLOCK_H_

#include <mutex>
#include <thread>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"

namespace realtime_tools
{
class RealtimeClock
{
public:
  /**
   * Default constructor creates an instance that always returns zero time.
   */
  RealtimeClock();

  /**
   * Create a realtime-safe wrapper around a clock object.
   */
  explicit RealtimeClock(rclcpp::Clock::SharedPtr clock);

  /**
   * Create a realtime-safe wrapper around a clock object with a specified logger.
   */
  RealtimeClock(rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger);

  ~RealtimeClock();

  /**
   * Get the current time from the clock.
   * \deprecated use now() instead.
   */
  [[deprecated]] rclcpp::Time getSystemTime(const rclcpp::Time & realtime_time = rclcpp::Time());

  /**
   * Get the current time from the clock.
   * \return current time, or
   * \return zero if RealtimeClock was not given a valid clock object or time is uninitialized.
   */
  rclcpp::Time now(const rclcpp::Time & realtime_time = rclcpp::Time());

private:
  void loop();

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  unsigned int lock_misses_ = 0;
  rclcpp::Time system_time_;
  rclcpp::Duration clock_offset_{0, 0u};

  rclcpp::Time last_realtime_time_;
  bool running_ = false;
  bool initialized_ = false;
  std::mutex mutex_;
  std::thread thread_;
};  // class

}  // namespace realtime_tools
#endif  // REALTIME_TOOLS__REALTIME_CLOCK_H_
