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

#ifndef REALTIME_TOOLS__REALTIME_HELPERS_HPP_
#define REALTIME_TOOLS__REALTIME_HELPERS_HPP_

#include <string>
#include <thread>
#include <utility>

namespace realtime_tools
{
/**
 * Detect if realtime kernel is present.
 * \returns true if realtime kernel is detected
 */
bool has_realtime_kernel();

/**
 * Configure SCHED_FIFO thread priority for the thread that calls this function
 * \param[in] priority the priority of this thread from 0-99
 * \returns true if configuring scheduler succeeded
 */
bool configure_sched_fifo(int priority);

/**
 * Locks the memory pages of the calling thread to prevent page faults.
 * By calling this method, the programs locks all pages mapped into the address
 * space of the calling process and future mappings. This means that the kernel
 * will not swap out the pages to disk i.e., the pages are guaranteed to stay in
 * RAM until later unlocked - which is important for realtime applications.
 * \param[out] message a message describing the result of the operation
 * \returns true if memory locking succeeded, false otherwise
*/
bool lock_memory(std::string & message);

/**
 * Configure the caller thread affinity - Tell the scheduler to prefer a certain
 * core for the given thread handle.
 * \note The threads created by the calling thread will inherit the affinity.
 * \param[in] thread the thread handle of the thread
 * \param[in] core the cpu number of the core. If a negative number is passed,
 * the affinity is reset to the default.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_thread_affinity(pthread_t thread, int core);

/**
 * Configure the caller thread affinity - Tell the scheduler to prefer a certain
 * core for the given thread.
 * \note The threads created by the calling thread will inherit the affinity.
 * \param[in] thread the reference of the thread
 * \param[in] core the cpu number of the core. If a negative number is passed,
 * the affinity is reset to the default.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_thread_affinity(std::thread & thread, int core);

/**
 * Configure the current thread affinity - Tell the scheduler to prefer a certain
 * core for the current thread.
 * \note The threads created by the calling thread will inherit the affinity.
 * \param[in] core the cpu number of the core. If a negative number is passed,
 * the affinity is reset to the default.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_current_thread_affinity(int core);

/**
 * Method to get the amount of available cpu cores
 * \ref https://man7.org/linux/man-pages/man3/sysconf.3.html
 * \ref https://stackoverflow.com/a/150971
 * \returns The number of processors currently online (available)
*/
int64_t get_number_of_available_processors();

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__REALTIME_HELPERS_HPP_
