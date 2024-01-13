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

#include <sched.h>

#include <unistd.h>
#include <cstring>
#include <fstream>
#include <iostream>

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

bool configure_sched_fifo(int priority)
{
  struct sched_param schedp;
  memset(&schedp, 0, sizeof(schedp));
  schedp.sched_priority = priority;
  return !sched_setscheduler(0, SCHED_FIFO, &schedp);
}

static void print_error(const int errc)
{
  switch (errc) {
    case 0:
      return;
    case EFAULT:
      std::cerr << "Call of sched_setaffinity with invalid cpuset" << std::endl;
      return;
    case EINVAL:
      std::cerr << "Call of sched_setaffinity with an invalid cpu core" << std::endl;
      return;
    case ESRCH:
      std::cerr << "Call of sched_setaffinity with and invalid thread id/process id" << std::endl;
      return;
    default:
      std::cerr << "Error code: " << errc << ": " << std::string(strerror(errc)) << std::endl;
  }
}

bool set_preferred_core(const int core)
{
  //Allow attaching the thread/process to a certain cpu core
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  //Obtain amount of cores/
  const auto number_of_cores = get_core_count();

  //Reset affinity by setting it to all cores
  if (core < 0) {
    for (int i{0}; i < number_of_cores; i++) {
      CPU_SET(i, &cpuset);
    }
    //And actually tell the schedular to set the affinitiy of the currently calling thread
    const auto result = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    print_error(result);
    return result == 0;
  }

  if (core < number_of_cores) {
    //Set the passed core to the cpu set
    CPU_SET(core, &cpuset);
    //And actually tell the schedular to set the affinitiy of the currently calling thread
    const auto result = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    print_error(result);
    return result == 0;
  }
  //Invalid core number passed
  return false;
}

int get_core_count() { return sysconf(_SC_NPROCESSORS_ONLN); }

}  // namespace realtime_tools
