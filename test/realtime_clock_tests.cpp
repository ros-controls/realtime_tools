// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the Open Source Robotics Foundation, Inc. nor the names of its
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

#include <gmock/gmock.h>

#include <chrono>
#include <thread>

#include "rclcpp/utilities.hpp"
#include "realtime_tools/realtime_clock.hpp"

using realtime_tools::RealtimeClock;

TEST(RealtimeClock, get_system_time)
{
  // initialize the global context
  rclcpp::init(0, nullptr);
  const int ATTEMPTS = 10;
  const std::chrono::milliseconds DELAY(1);

  rclcpp::Clock::SharedPtr clock(new rclcpp::Clock());
  {
    RealtimeClock rt_clock(clock);
    // Wait for time to be available
    rclcpp::Time last_rt_time;
    for (int i = 0; i < ATTEMPTS && rclcpp::Time() == last_rt_time; ++i) {
      std::this_thread::sleep_for(DELAY);
      last_rt_time = rt_clock.now(rclcpp::Time());
    }
    ASSERT_NE(rclcpp::Time(), last_rt_time);

    // This test assumes system time will not jump backwards during it
    EXPECT_GT(rt_clock.now(last_rt_time), last_rt_time);
  }
  rclcpp::shutdown();
}
