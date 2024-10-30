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

#include "realtime_tools/thread_priority.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <sched.h>
#endif

#include <sys/capability.h>
#include <sys/mman.h>
#endif

#include <cstring>
#include <fstream>

namespace realtime_tools
{
bool has_realtime_kernel()
{
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open()) {
    realtime_file >> has_realtime;
  }
  return has_realtime;
}

#ifdef _WIN32
bool configure_sched_fifo(int priority)
{
  HANDLE thread = GetCurrentThread();
  return SetThreadPriority(thread, priority);
}
#else
bool configure_sched_fifo(int priority)
{
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  return !sched_setscheduler(0, SCHED_FIFO, &schedp);
}

bool is_capable(cap_value_t v)
{
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
}

bool lock_memory(std::string & message)
{
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
    return false;
  } else {
    message = "Memory locked successfully!";
    return true;
  }
}
#endif
}  // namespace realtime_tools
