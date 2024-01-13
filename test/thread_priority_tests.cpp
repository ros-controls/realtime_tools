#include <gmock/gmock.h>
#include <realtime_tools/realtime_helpers.hpp>
#include <thread>

TEST(thread_priority, get_core_count)
{
  const auto count = realtime_tools::get_core_count();

  EXPECT_EQ(count, std::thread::hardware_concurrency());
}

TEST(thread_priority, set_preferred_core_valid)
{
  //We should always have at least one core
  EXPECT_TRUE(realtime_tools::set_preferred_core(0));
}

TEST(thread_priority, set_preferred_core_invalid_too_many_cores)
{
  const auto count = realtime_tools::get_core_count();
  //We should always have at least one core
  EXPECT_FALSE(realtime_tools::set_preferred_core(count + 10));
}

TEST(thread_priority, set_preferred_core_valid_reset)
{
  //Reset core affinity
  EXPECT_TRUE(realtime_tools::set_preferred_core(-1));
}
