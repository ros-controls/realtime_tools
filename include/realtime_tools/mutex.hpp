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

#include <errno.h>
#include <pthread.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace realtime_tools
{
class mutex
{
public:
  mutex()
  {
    pthread_mutexattr_t attr;

    // Initialize the mutex attributes
    const auto res_attr = pthread_mutexattr_init(&attr);
    if (res_attr != 0) {
      throw std::runtime_error(
        std::string("Failed to initialize mutex attribute : ") + std::strerror(res_attr));
    }

    // Set the mutex type to PTHREAD_MUTEX_ERRORCHECK
    const auto res_type = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
    if (res_type != 0) {
      throw std::runtime_error(
        std::string("Failed to set mutex type : ") + std::strerror(res_type));
    }

    // Set the mutex attribute to use the protocol PTHREAD_PRIO_INHERIT
    const auto res_protocol = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    if (res_protocol != 0) {
      throw std::runtime_error(
        std::string("Failed to set mutex protocol : ") + std::strerror(res_protocol));
    }

    // Set the mutex attribute to have a priority ceiling of 99
    const auto res_ceiling = pthread_mutexattr_setprioceiling(&attr, 99);
    if (res_ceiling != 0) {
      throw std::runtime_error(
        std::string("Failed to set mutex prio ceiling : ") + std::strerror(res_ceiling));
    }

    // Set the mutex attribute robustness to PTHREAD_MUTEX_ROBUST
    const auto res_robust = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    if (res_robust != 0) {
      throw std::runtime_error(
        std::string("Failed to set mutex robustness : ") + std::strerror(res_robust));
    }

    // Initialize the mutex with the attributes
    const auto res_init = pthread_mutex_init(&mutex_, &attr);
    if (res_init != 0) {
      throw std::runtime_error(
        std::string("Failed to initialize mutex : ") + std::strerror(res_init));
    }

    // Destroy the mutex attributes
    const auto res_destroy = pthread_mutexattr_destroy(&attr);
    if (res_destroy != 0) {
      throw std::runtime_error(
        std::string("Failed to destroy mutex attribute : ") + std::strerror(res_destroy));
    }
  }

  ~mutex()
  {
    const auto res = pthread_mutex_destroy(&mutex_);
    if (res != 0) {
      std::cerr << "Failed to destroy mutex : " << std::strerror(res) << std::endl;
    }
  }

  const pthread_mutex_t * get_mutex() const { return &mutex_; }

  pthread_mutex_t * get_mutex() { return &mutex_; }

  void lock()
  {
    const auto res = pthread_mutex_lock(&mutex_);
    if (res != 0) {
      if (res == EOWNERDEAD) {
        const auto res_consistent = pthread_mutex_consistent(&mutex_);
        if (res_consistent != 0) {
          throw std::runtime_error(
            std::string("Failed to make mutex consistent : ") + std::strerror(res_consistent));
        }
      } else {
        throw std::runtime_error(std::string("Failed to lock mutex : ") + std::strerror(res));
      }
    }
  }

  void unlock()
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
    if (res != 0) {
      if (res == EBUSY) {
        return false;
      } else if (res == EOWNERDEAD) {
        const auto res_consistent = pthread_mutex_consistent(&mutex_);
        if (res_consistent != 0) {
          throw std::runtime_error(
            std::string("Failed to make mutex consistent : ") + std::strerror(res_consistent));
        }
      } else {
        throw std::runtime_error(std::string("Failed to try lock mutex : ") + std::strerror(res));
      }
    }
    return true;
  }

private:
  pthread_mutex_t mutex_;
};

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__MUTEX_HPP_
