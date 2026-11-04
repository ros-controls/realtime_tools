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
#include <string>
#include <thread>

#ifdef __linux__
#include <sys/capability.h>
#include <sys/resource.h>
#endif

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
  EXPECT_FALSE(realtime_tools::set_current_thread_affinity(std::vector<int>({count + 10})).first);
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

#ifdef __linux__
namespace
{
bool has_effective_capability(cap_value_t capability)
{
  cap_t caps = cap_get_proc();
  if (caps == nullptr) {
    return false;
  }
  cap_flag_value_t value = CAP_CLEAR;
  const bool ok = cap_get_flag(caps, capability, CAP_EFFECTIVE, &value) == 0;
  cap_free(caps);
  return ok && value == CAP_SET;
}

int count_occurrences(const std::string & text, const std::string & pattern)
{
  int count = 0;
  for (auto pos = text.find(pattern); pos != std::string::npos;
       pos = text.find(pattern, pos + pattern.size())) {
    ++count;
  }
  return count;
}

/// Lowers the soft limit of a resource for the scope of the test and restores it afterwards
class ScopedSoftLimit
{
public:
  ScopedSoftLimit(int resource, rlim_t soft_limit) : resource_(resource)
  {
    getrlimit(resource_, &original_);
    rlimit lowered = original_;
    lowered.rlim_cur = soft_limit;
    setrlimit(resource_, &lowered);
  }
  ~ScopedSoftLimit() { setrlimit(resource_, &original_); }

private:
  int resource_;
  rlimit original_{};
};
}  // namespace

TEST(realtime_environment, reports_every_check_once)
{
  const auto [ok, report] = realtime_tools::check_realtime_environment(0);
  for (const auto & check :
       {"Realtime kernel", "RT priority", "Memory locking", "CPU governor", "Isolated CPUs"}) {
    EXPECT_EQ(count_occurrences(report, check), 1) << check << " missing in:\n" << report;
  }
  // The overall result is false exactly when at least one check warns
  EXPECT_EQ(ok, report.find("[WARN]") == std::string::npos) << report;
}

TEST(realtime_environment, priority_zero_is_always_allowed)
{
  const auto report = realtime_tools::check_realtime_environment(0).second;
  EXPECT_THAT(report, ::testing::HasSubstr("[OK] RT priority")) << report;
}

TEST(realtime_environment, warns_when_rtprio_limit_too_low)
{
  const ScopedSoftLimit limit(RLIMIT_RTPRIO, 0);
  const auto [ok, report] = realtime_tools::check_realtime_environment(50);
  if (has_effective_capability(CAP_SYS_NICE)) {
    // CAP_SYS_NICE bypasses RLIMIT_RTPRIO
    EXPECT_THAT(report, ::testing::HasSubstr("[OK] RT priority")) << report;
  } else {
    EXPECT_FALSE(ok);
    EXPECT_THAT(report, ::testing::HasSubstr("[WARN] RT priority")) << report;
    EXPECT_THAT(report, ::testing::HasSubstr("RLIMIT_RTPRIO")) << report;
  }
}

TEST(realtime_environment, warns_when_memlock_limit_too_low)
{
  const ScopedSoftLimit limit(RLIMIT_MEMLOCK, 0);
  const auto [ok, report] = realtime_tools::check_realtime_environment(0);
  if (has_effective_capability(CAP_IPC_LOCK)) {
    // CAP_IPC_LOCK bypasses RLIMIT_MEMLOCK
    EXPECT_THAT(report, ::testing::HasSubstr("[OK] Memory locking")) << report;
  } else {
    EXPECT_FALSE(ok);
    EXPECT_THAT(report, ::testing::HasSubstr("[WARN] Memory locking")) << report;
    EXPECT_THAT(report, ::testing::HasSubstr("RLIMIT_MEMLOCK")) << report;
  }
}

TEST(realtime_environment, memlock_ok_when_unlimited_or_capable)
{
  rlimit memlock{};
  ASSERT_EQ(getrlimit(RLIMIT_MEMLOCK, &memlock), 0);
  const auto report = realtime_tools::check_realtime_environment(0).second;
  const bool expect_ok =
    memlock.rlim_cur == RLIM_INFINITY || has_effective_capability(CAP_IPC_LOCK);
  EXPECT_EQ(report.find("[OK] Memory locking") != std::string::npos, expect_ok) << report;
}
#else
TEST(realtime_environment, unsupported_platform)
{
  EXPECT_FALSE(realtime_tools::check_realtime_environment(0).first);
}
#endif
