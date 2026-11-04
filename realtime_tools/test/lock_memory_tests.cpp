// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// lock_memory() with prefaulting changes process-wide malloc settings, so these tests live in their
// own executable to not influence other tests.

#include <gmock/gmock.h>

#include <malloc.h>
#include <sys/resource.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>

#include "realtime_tools/realtime_helpers.hpp"

namespace
{
constexpr std::size_t kStackSize = 64 * 1024;
constexpr std::size_t kHeapSize = 8 * 1024 * 1024;

// mlockall(MCL_FUTURE) stays active for the whole process once it succeeded, and with a finite
// RLIMIT_MEMLOCK any later allocation beyond the limit fails. Lowering the soft limit to 0 makes
// locking fail on every host (unless the process has CAP_IPC_LOCK, which ignores the limit), so the
// tests check the prefault behavior independently of the host's limits.
class LockMemory : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ASSERT_EQ(getrlimit(RLIMIT_MEMLOCK, &original_memlock_), 0);
    rlimit no_memlock = original_memlock_;
    no_memlock.rlim_cur = 0;
    ASSERT_EQ(setrlimit(RLIMIT_MEMLOCK, &no_memlock), 0);
  }
  void TearDown() override { setrlimit(RLIMIT_MEMLOCK, &original_memlock_); }

private:
  rlimit original_memlock_{};
};

long minor_page_faults()  // NOLINT(runtime/int): rusage uses long
{
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_minflt;
}
}  // namespace

TEST_F(LockMemory, rejects_stack_prefault_larger_than_stack_limit)
{
  rlimit stack{};
  ASSERT_EQ(getrlimit(RLIMIT_STACK, &stack), 0);
  if (stack.rlim_cur == RLIM_INFINITY) {
    GTEST_SKIP() << "Unlimited stack, nothing to reject";
  }
  const auto [ok, message] = realtime_tools::lock_memory(stack.rlim_cur, 0);
  EXPECT_FALSE(ok);
  EXPECT_THAT(message, ::testing::HasSubstr("RLIMIT_STACK")) << message;
}

TEST_F(LockMemory, prefault_reports_same_lock_result_as_plain_lock_memory)
{
  const auto plain = realtime_tools::lock_memory();
  const auto [ok, message] = realtime_tools::lock_memory(kStackSize, kHeapSize);
  EXPECT_EQ(ok, plain.first) << message;
  EXPECT_THAT(message, ::testing::HasSubstr(plain.second)) << message;
}

#if defined(__GLIBC__) && (__GLIBC__ > 2 || (__GLIBC__ == 2 && __GLIBC_MINOR__ >= 33))
TEST_F(LockMemory, large_allocations_stay_on_the_prefaulted_heap)
{
  realtime_tools::lock_memory(kStackSize, kHeapSize);

  // Without M_MMAP_MAX = 0 a 1 MiB allocation is served by mmap (counted in hblks) and returned to
  // the OS on free, so it would page fault again on every allocation.
  const auto mmapped_before = mallinfo2().hblks;
  void * block = std::malloc(1024 * 1024);
  ASSERT_NE(block, nullptr);
  EXPECT_EQ(mallinfo2().hblks, mmapped_before);
  std::free(block);
}
#endif

TEST_F(LockMemory, allocations_within_prefaulted_heap_do_not_page_fault)
{
  realtime_tools::lock_memory(kStackSize, kHeapSize);

  constexpr std::size_t kSize = 1024 * 1024;
  const long faults_before = minor_page_faults();  // NOLINT(runtime/int)
  auto * block = static_cast<char *>(std::malloc(kSize));
  ASSERT_NE(block, nullptr);
  std::memset(block, 1, kSize);
  const long faults = minor_page_faults() - faults_before;  // NOLINT(runtime/int)
  std::free(block);

  // Touching 1 MiB of fresh memory costs kSize / page_size (256 with 4 KiB pages) faults. Allow a
  // few for unrelated allocations of the test framework.
  const auto pages = static_cast<long>(kSize / sysconf(_SC_PAGESIZE));  // NOLINT(runtime/int)
  EXPECT_LT(faults, pages / 8) << faults << " minor page faults while touching " << pages
                               << " pages";
}
