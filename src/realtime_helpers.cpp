// Copyright (c) 2022, PickNik, Inc.
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
//    * Neither the name of the PickNik Inc. nor the names of its
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

#include "realtime_tools/realtime_helpers.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <sched.h>
#include <sys/capability.h>
#include <sys/mman.h>
#include <sys/utsname.h>

#include <unistd.h>
#endif

#include <cstring>
#include <fstream>
#include <iostream>

namespace realtime_tools
{
bool has_realtime_kernel()
{
#ifdef _WIN32
  std::cerr << "Realtime kernel detection is not supported on Windows." << std::endl;
  return false;
#else
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }
  if (!has_realtime) {
    struct utsname kernel_info;
    if (uname(&kernel_info) == -1) {
      std::cerr << "Error: Could not get kernel information : " << std::strerror(errno)
                << std::endl;
      return false;
    }
    const std::string kernel_version(kernel_info.version);
    return kernel_version.find("PREEMPT_RT") != std::string::npos;
  }
  return has_realtime;
#endif
}

bool configure_sched_fifo(int priority)
{
#ifdef _WIN32
  HANDLE thread = GetCurrentThread();
  return SetThreadPriority(thread, priority);
#else
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  return !sched_setscheduler(0, SCHED_FIFO, &schedp);
#endif
}

bool lock_memory(std::string & message)
{
  const auto lock_result = lock_memory();
  message = lock_result.second;
  return lock_result.first;
}

std::pair<bool, std::string> lock_memory()
{
#ifdef _WIN32
  return {false, "Memory locking is not supported on Windows."};
#else
  auto is_capable = [](cap_value_t v) -> bool {
    bool rc = false;
    cap_t caps;
    if ((caps = cap_get_proc()) == NULL) {
      return false;
    }

    if (cap_set_flag(caps, CAP_EFFECTIVE, 1, &v, CAP_SET) == -1) {
      rc = false;
    } else {
      rc = (cap_set_proc(caps) == 0);
    }
    cap_free(caps);
    return rc;
  };

  std::string message;
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    if (!is_capable(CAP_IPC_LOCK)) {
      message = "No proper privileges to lock the memory!";
    } else if (errno == ENOMEM) {
      message =
        "The caller had a nonzero RLIMIT_MEMLOCK soft resource limit, but tried to lock more "
        "memory than the limit permitted. Try running the application with privileges!";
    } else if (errno == EPERM) {
      message =
        "The caller is not privileged, but needs privilege to perform the requested operation.";
    } else if (errno == EINVAL) {
      message =
        "The result of the addition start+len was less than start (e.g., the addition may have "
        "resulted in an overflow).";
    } else if (errno == EAGAIN) {
      message = "Some or all of the specified address range could not be locked.";
    } else {
      message = "Unknown error occurred!";
    }
    return {false, message};
  } else {
    message = "Memory locked successfully!";
    return {true, message};
  }
#endif
}

std::pair<bool, std::string> set_thread_affinity(
  NATIVE_THREAD_HANDLE thread, const std::vector<int> & cores)
{
  std::string message;
#ifdef _WIN32
  message = "Thread affinity is not supported on Windows.";
  return std::make_pair(false, message);
#else
  auto set_affinity_result_message = [](int result, std::string & msg) -> bool {
    if (result == 0) {
      msg = "Thread affinity set successfully!";
      return true;
    }
    switch (errno) {
      case EFAULT:
        msg = "Call of sched_setaffinity with invalid cpuset!";
        break;
      case EINVAL:
        msg = "Call of sched_setaffinity with an invalid cpu core!";
        break;
      case ESRCH:
        msg = "Call of sched_setaffinity with a thread id/process id that is invalid or not found!";
        break;
      case EPERM:
        msg = "Call of sched_setaffinity with insufficient privileges!";
        break;
      default:
        msg = "Unknown error code: " + std::string(strerror(errno));
    }
    return false;
  };
  // Allow attaching the thread/process to a certain cpu core
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  // Obtain available processors
  const auto number_of_cores = get_number_of_available_processors();

  bool valid_cpu_set = true;
  // Reset affinity by setting it to all cores
  if (cores.empty()) {
    for (auto i = 0; i < number_of_cores; i++) {
      CPU_SET(i, &cpuset);
    }
  } else {
    for (const auto core : cores) {
      if (core < 0 || core >= number_of_cores) {
        valid_cpu_set = false;
        break;
      }
      CPU_SET(core, &cpuset);
    }
  }

  if (valid_cpu_set) {
    // And actually tell the schedular to set the affinity of the thread of respective pid
    const auto result = set_affinity_result_message(
      pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset), message);
    return std::make_pair(result, message);
  }
  // create a string from the core numbers
  std::string core_numbers;
  for (const auto core : cores) {
    core_numbers += std::to_string(core) + " ";
  }
  // Invalid core number passed
  message = "Invalid core numbers : ['" + core_numbers + "'] passed! The system has " +
            std::to_string(number_of_cores) +
            " cores. Parsed core number should be between 0 and " +
            std::to_string(number_of_cores - 1);
  return std::make_pair(false, message);
#endif
}

std::pair<bool, std::string> set_thread_affinity(NATIVE_THREAD_HANDLE thread, int core)
{
  const std::vector<int> affinity_cores = core < 0 ? std::vector<int>() : std::vector<int>{core};
  return set_thread_affinity(thread, affinity_cores);
}

std::pair<bool, std::string> set_thread_affinity(std::thread & thread, int core)
{
  if (!thread.joinable()) {
    return std::make_pair(
      false, "Unable to set the thread affinity, as the thread is not joinable!");
  }
  return set_thread_affinity(thread.native_handle(), core);
}

std::pair<bool, std::string> set_current_thread_affinity(int core)
{
#ifdef _WIN32
  return set_thread_affinity(GetCurrentThread(), core);
#else
  return set_thread_affinity(pthread_self(), core);
#endif
}

std::pair<bool, std::string> set_current_thread_affinity(const std::vector<int> & cores)
{
#ifdef _WIN32
  return set_thread_affinity(GetCurrentThread(), cores);
#else
  return set_thread_affinity(pthread_self(), cores);
#endif
}

int64_t get_number_of_available_processors()
{
#ifdef _WIN32
  SYSTEM_INFO sysinfo;
  GetSystemInfo(&sysinfo);
  return static_cast<int64_t>(sysinfo.dwNumberOfProcessors);
#else
  return sysconf(_SC_NPROCESSORS_ONLN);
#endif
}

}  // namespace realtime_tools
