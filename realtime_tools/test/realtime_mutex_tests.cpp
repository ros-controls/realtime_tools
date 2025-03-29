// Copyright 2024 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#include <gmock/gmock.h>

#include <mutex>
#include <thread>

#include <realtime_tools/mutex.hpp>

TEST(PriorityInheritanceMutexTests, lock_unlock)
{
  // The mutex is locked and unlocked correctly
  realtime_tools::prio_inherit_mutex mutex;
  mutex.lock();
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded)
{
  // The mutex is locked and unlocked correctly in a multithreaded environment
  realtime_tools::prio_inherit_mutex mutex;
  std::thread t1([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.unlock();
  });
  std::thread t2([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.unlock();
  });
  t1.join();
  t2.join();
}

TEST(PriorityInheritanceMutexTests, recursive_lock_lock_unlock_multithreaded)
{
  // The mutex is locked and unlocked correctly in a multithreaded environment
  realtime_tools::prio_inherit_recursive_mutex mutex;
  std::thread t1([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.unlock();
  });
  std::thread t2([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex.unlock();
  });
  t1.join();
  t2.join();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded_multiple_mutexes)
{
  // The mutex is locked and unlocked correctly in a multithreaded environment with multiple mutexes
  realtime_tools::prio_inherit_mutex mutex1;
  realtime_tools::prio_inherit_mutex mutex2;
  std::thread t1([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  std::thread t2([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  t1.join();
  t2.join();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded_multiple_mutexes_different_types)
{
  // The mutex is locked and unlocked correctly in a multithreaded environment with multiple mutexes
  realtime_tools::prio_inherit_mutex mutex1;
  realtime_tools::prio_inherit_recursive_mutex mutex2;
  std::thread t1([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  std::thread t2([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  t1.join();
  t2.join();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_recursive_mutex)
{
  // Test to check that the mutex is recursive
  realtime_tools::prio_inherit_recursive_mutex mutex;
  mutex.lock();
  mutex.lock();
  mutex.unlock();
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded_recursive_mutex_multiple_mutexes)
{
  realtime_tools::prio_inherit_recursive_mutex mutex1;
  realtime_tools::prio_inherit_recursive_mutex mutex2;
  std::thread t1([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  std::thread t2([&mutex1, &mutex2]() {
    mutex1.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex1.unlock();
    mutex2.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    mutex2.unlock();
  });
  t1.join();
  t2.join();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded_mutex_one_thread_dies)
{
  realtime_tools::prio_inherit_mutex mutex;
  std::thread t1([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // simulating no unlock. Due to the robustness of the mutex, the mutex should be consistent
  });
  std::thread t2([&mutex]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    mutex.lock();
    mutex.unlock();
  });
  t1.join();
  t2.join();
  mutex.lock();
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, lock_unlock_multithreaded_recursive_mutex_one_thread_dies)
{
  realtime_tools::prio_inherit_recursive_mutex mutex;
  std::thread t1([&mutex]() {
    mutex.lock();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // simulating no unlock. Due to the robustness of the mutex, the mutex should be consistent
  });
  std::thread t2([&mutex]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    mutex.lock();
    mutex.unlock();
  });
  t1.join();
  t2.join();
  mutex.lock();
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, lock_guard_mutex)
{
  realtime_tools::prio_inherit_mutex mutex;
  {
    std::lock_guard<realtime_tools::prio_inherit_mutex> lock(mutex);
  }

  realtime_tools::prio_inherit_recursive_mutex recursive_mutex;
  {
    std::lock_guard<realtime_tools::prio_inherit_recursive_mutex> lock(recursive_mutex);
  }
}

TEST(PriorityInheritanceMutexTests, unique_lock_mutex)
{
  realtime_tools::prio_inherit_mutex mutex;
  {
    std::unique_lock<realtime_tools::prio_inherit_mutex> lock(mutex);
  }

  realtime_tools::prio_inherit_recursive_mutex recursive_mutex;
  {
    std::unique_lock<realtime_tools::prio_inherit_recursive_mutex> lock(recursive_mutex);
  }
}

TEST(PriorityInheritanceMutexTests, try_lock_mutex)
{
  {
    realtime_tools::prio_inherit_mutex mutex;
    ASSERT_TRUE(mutex.try_lock());
    ASSERT_THROW(mutex.try_lock(), std::system_error)
      << "Mutex is already locked in the same thread";
    std::thread t([&mutex]() {
      ASSERT_FALSE(mutex.try_lock())
        << "try_lock should pass when checking from a different thread";
    });
    t.join();
    mutex.unlock();
    ASSERT_TRUE(mutex.try_lock());
    mutex.unlock();
  }

  {
    realtime_tools::prio_inherit_recursive_mutex recursive_mutex;
    ASSERT_TRUE(recursive_mutex.try_lock());
    ASSERT_TRUE(recursive_mutex.try_lock());
    ASSERT_TRUE(recursive_mutex.try_lock());
    recursive_mutex.unlock();
    recursive_mutex.unlock();
    recursive_mutex.unlock();
  }
}

TEST(PriorityInheritanceMutexTests, standard_lock_test)
{
  realtime_tools::prio_inherit_mutex mutex1;
  realtime_tools::prio_inherit_mutex mutex2;
  {
    std::lock(mutex1, mutex2);
    // do work
    mutex1.unlock();
    mutex2.unlock();
  }
  {
    std::scoped_lock lock(mutex1, mutex2);
  }
  ASSERT_TRUE(mutex1.try_lock());
  ASSERT_TRUE(mutex2.try_lock());
  mutex1.unlock();
  mutex2.unlock();
}

TEST(PriorityInheritanceMutexTests, native_handle_mutex)
{
  {
    realtime_tools::prio_inherit_mutex mutex;
    auto native_handle = mutex.native_handle();
    ASSERT_NE(native_handle, nullptr);
  }

  {
    realtime_tools::prio_inherit_recursive_mutex recursive_mutex;
    auto native_handle = recursive_mutex.native_handle();
    ASSERT_NE(native_handle, nullptr);
  }
}

TEST(PriorityInheritanceMutexTests, test_mutex_lock_functionality)
{
  // Trying to lock again should throw an exception
  realtime_tools::prio_inherit_mutex mutex;
  mutex.lock();
  ASSERT_THROW(mutex.lock(), std::system_error);
  mutex.unlock();
  ASSERT_NO_THROW(mutex.lock());
  ASSERT_THROW(mutex.try_lock(), std::system_error);
  mutex.unlock();
  ASSERT_NO_THROW(mutex.try_lock());
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, test_lock_constructors)
{
  realtime_tools::prio_inherit_mutex mutex;
  {
    std::unique_lock<realtime_tools::prio_inherit_mutex> lock(mutex, std::defer_lock);
    ASSERT_FALSE(lock.owns_lock());
    lock.lock();
    ASSERT_TRUE(lock.owns_lock());
    lock.unlock();
  }
  {
    std::unique_lock<realtime_tools::prio_inherit_mutex> lock(mutex, std::try_to_lock);
    ASSERT_TRUE(lock.owns_lock());
  }
}

TEST(PriorityInheritanceMutexTests, test_deadlock_detection)
{
  realtime_tools::prio_inherit_mutex mutex;
  mutex.lock();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_THROW(mutex.try_lock(), std::system_error);
  ASSERT_THROW(mutex.lock(), std::system_error);
  // In a different thread, try to lock the mutex should not throw an exception
  std::thread t([&mutex]() { ASSERT_FALSE(mutex.try_lock()); });
  t.join();
  mutex.unlock();
}

TEST(PriorityInheritanceMutexTests, test_mutex_reflection)
{
  static_assert(
    std::is_same<
      realtime_tools::prio_inherit_mutex::type,
      realtime_tools::detail::error_mutex_type_t>::value == true);
  static_assert(
    std::is_same<
      realtime_tools::prio_inherit_recursive_mutex::type,
      realtime_tools::detail::recursive_mutex_type_t>::value == true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
