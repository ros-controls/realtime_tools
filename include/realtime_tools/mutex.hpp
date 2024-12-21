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

#ifndef REALTIME_TOOLS__MUTEX_HPP_
#define REALTIME_TOOLS__MUTEX_HPP_

#ifdef _WIN32
#error "The mutex.hpp header is not supported on Windows platforms"
#endif

#include <pthread.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

/**
 * @brief A pthread mutex wrapper that provides a mutex with the priority inheritance
 * protocol and a priority ceiling of 99.
 * The mutex is also error checked and robust.
 * This mutex is intended to be used in real-time contexts.
 * @note This mutex is not recursive.
 */
namespace realtime_tools
{
namespace detail
{
struct error_mutex_type_t
{
  static constexpr int value = PTHREAD_MUTEX_ERRORCHECK;
};

struct recursive_mutex_type_t
{
  static constexpr int value = PTHREAD_MUTEX_RECURSIVE;
};

struct stalled_robustness_t
{
  static constexpr int value = PTHREAD_MUTEX_STALLED;
};

struct robust_robustness_t
{
  static constexpr int value = PTHREAD_MUTEX_ROBUST;
};
/**
 * @brief A class template that provides a pthread mutex with the priority inheritance protocol
 *
 * @tparam MutexType The type of the mutex. It can be one of the following: PTHREAD_MUTEX_NORMAL, PTHREAD_MUTEX_RECURSIVE, PTHREAD_MUTEX_ERRORCHECK, PTHREAD_MUTEX_DEFAULT
 * @tparam MutexRobustness The robustness of the mutex. It can be one of the following: PTHREAD_MUTEX_STALLED, PTHREAD_MUTEX_ROBUST
 */
template <typename MutexType, typename MutexRobustness>
class mutex
{
public:
  using native_handle_type = pthread_mutex_t *;
  using type = MutexType;
  using robustness = MutexRobustness;

  mutex()
  {
    pthread_mutexattr_t attr;

    const auto attr_destroy = [](pthread_mutexattr_t * mutex_attr) {
      // Destroy the mutex attributes
      const auto res_destroy = pthread_mutexattr_destroy(mutex_attr);
      if (res_destroy != 0) {
        throw std::system_error(
          res_destroy, std::generic_category(), "Failed to destroy mutex attribute");
      }
    };
    using attr_cleanup_t = std::unique_ptr<pthread_mutexattr_t, decltype(attr_destroy)>;
    auto attr_cleanup = attr_cleanup_t(&attr, attr_destroy);

    // Initialize the mutex attributes
    const auto res_attr = pthread_mutexattr_init(&attr);
    if (res_attr != 0) {
      throw std::system_error(
        res_attr, std::system_category(), "Failed to initialize mutex attribute");
    }

    // Set the mutex type to MutexType
    const auto res_type = pthread_mutexattr_settype(&attr, MutexType::value);

    if (res_type != 0) {
      throw std::system_error(res_type, std::system_category(), "Failed to set mutex type");
    }

    // Set the mutex attribute to use the protocol PTHREAD_PRIO_INHERIT
    const auto res_protocol = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    if (res_protocol != 0) {
      throw std::system_error(res_protocol, std::system_category(), "Failed to set mutex protocol");
    }

    // Set the mutex attribute robustness to MutexRobustness
    const auto res_robust = pthread_mutexattr_setrobust(&attr, MutexRobustness::value);
    if (res_robust != 0) {
      throw std::system_error(res_robust, std::system_category(), "Failed to set mutex robustness");
    }

    // Initialize the mutex with the attributes
    const auto res_init = pthread_mutex_init(&mutex_, &attr);
    if (res_init != 0) {
      throw std::system_error(res_init, std::system_category(), "Failed to initialize mutex");
    }
  }

  ~mutex()
  {
    const auto res = pthread_mutex_destroy(&mutex_);
    if (res != 0) {
      std::cerr << "Failed to destroy mutex : " << std::strerror(res) << std::endl;
    }
  }

  mutex(const mutex &) = delete;

  mutex & operator=(const mutex &) = delete;

  native_handle_type native_handle() noexcept { return &mutex_; }

  void lock()
  {
    const auto res = pthread_mutex_lock(&mutex_);
    if (res == 0) {
      return;
    }
    if (res == EOWNERDEAD) {
      const auto res_consistent = pthread_mutex_consistent(&mutex_);
      if (res_consistent != 0) {
        throw std::runtime_error(
          std::string("Failed to make mutex consistent : ") + std::strerror(res_consistent));
      }
      std::cerr << "Mutex owner died, but the mutex is consistent now. This shouldn't happen!"
                << std::endl;
    } else if (res == EDEADLK) {
      throw std::system_error(res, std::system_category(), "Deadlock detected");
    } else {
      throw std::runtime_error(std::string("Failed to lock mutex : ") + std::strerror(res));
    }
  }

  void unlock() noexcept
  {
    // As per the requirements of BasicLockable concept, unlock should not throw
    const auto res = pthread_mutex_unlock(&mutex_);
    if (res != 0) {
      std::cerr << "Failed to unlock mutex : " << std::strerror(res) << std::endl;
    }
  }

  bool try_lock()
  {
    const auto res = pthread_mutex_trylock(&mutex_);
    if (res == 0) {
      return true;
    }
    if (res == EBUSY) {
      return false;
    } else if (res == EOWNERDEAD) {
      const auto res_consistent = pthread_mutex_consistent(&mutex_);
      if (res_consistent != 0) {
        throw std::runtime_error(
          std::string("Failed to make mutex consistent : ") + std::strerror(res_consistent));
      }
      std::cerr << "Mutex owner died, but the mutex is consistent now. This shouldn't happen!"
                << std::endl;
    } else if (res == EDEADLK) {
      throw std::system_error(res, std::system_category(), "Deadlock detected");
    } else {
      throw std::runtime_error(std::string("Failed to try lock mutex : ") + std::strerror(res));
    }
    return true;
  }

private:
  pthread_mutex_t mutex_;
};
}  // namespace detail
using prio_inherit_mutex = detail::mutex<detail::error_mutex_type_t, detail::robust_robustness_t>;
using prio_inherit_recursive_mutex =
  detail::mutex<detail::recursive_mutex_type_t, detail::robust_robustness_t>;
}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__MUTEX_HPP_
