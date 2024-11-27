// Copyright (c) 2024, Lennart Nachtigall
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

// Author: Lennart Nachtigall

#include <gmock/gmock.h>
#include <thread>

#include <realtime_tools/realtime_helpers.hpp>

TEST(thread_priority, get_core_count)
{
  const auto count = realtime_tools::get_number_of_available_processors();

  EXPECT_EQ(count, std::thread::hardware_concurrency());
}

TEST(thread_priority, set_thread_affinity_valid)
{
  // create a basic thread for the test
  std::thread t([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
  // We should always have at least one core
  EXPECT_TRUE(realtime_tools::set_thread_affinity(t.native_handle(), 0).first);
  t.join();
}

TEST(thread_priority, set_thread_affinity_invalid_too_many_cores)
{
  // create a basic thread for the test
  std::thread t([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
  const int count = static_cast<int>(realtime_tools::get_number_of_available_processors());
  // We should always have at least one core
  EXPECT_FALSE(realtime_tools::set_thread_affinity(t.native_handle(), count + 10).first);
  EXPECT_FALSE(realtime_tools::set_thread_affinity(t.native_handle(), {0, 1, count + 2}).first);
  EXPECT_FALSE(realtime_tools::set_thread_affinity(t.native_handle(), {0, -1, 2}).first);
  EXPECT_FALSE(realtime_tools::set_thread_affinity(t.native_handle(), {0, 3, -1}).first);
  EXPECT_FALSE(
    realtime_tools::set_thread_affinity(t.native_handle(), std::vector<int>({-1})).first);
  t.join();
}

TEST(thread_priority, set_current_thread_affinity_invalid_too_many_cores)
{
  const int count = static_cast<int>(realtime_tools::get_number_of_available_processors());
  // We should always have at least one core
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity(count + 10).first);
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity({count + 10}).first);
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity({0, count + 10}).first);
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity({0, -1}).first);
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity(std::vector<int>({-1})).first);
}

TEST(thread_priority, set_thread_affinity_valid_reset)
{
  // create a basic thread for the test
  std::thread t([]() { std::this_thread::sleep_for(std::chrono::milliseconds(100)); });
  // Reset core affinity
  EXPECT_TRUE(realtime_tools::set_thread_affinity(t.native_handle(), -1).first);
  EXPECT_TRUE(realtime_tools::set_thread_affinity(t.native_handle(), {}).first);
  t.join();
}

TEST(thread_priority, set_current_thread_affinity_valid_reset)
{
  // Reset core affinity
  EXPECT_TRUE(realtime_tools::set_current_thread_affinity(-1).first);
  EXPECT_TRUE(realtime_tools::set_current_thread_affinity({}).first);
}

TEST(thread_priority, set_current_thread_affinity_valid)
{
  // We should always have at least one core
  EXPECT_TRUE(realtime_tools::set_current_thread_affinity(0).first);
  const int count = static_cast<int>(realtime_tools::get_number_of_available_processors());
  EXPECT_TRUE(realtime_tools::set_current_thread_affinity({0, count - 1}).first);
}
