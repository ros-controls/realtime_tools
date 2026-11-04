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

#include <cstddef>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
// Here we directly use void* instead of including windows.h
// and using HANDLE macro to avoid polluting all the downstream
// compilation units that include the public header realtime_helpers.hpp
// with problematic macros like MIN, MAX or ERROR
using NATIVE_THREAD_HANDLE = void *;
#else
using NATIVE_THREAD_HANDLE = pthread_t;
#endif

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
 * Configure SCHED_RR thread priority for the thread that calls this function.
 * SCHED_RR threads of the same priority are time-sliced (round-robin)
 * Useful on PREEMPT_RT kernels with multiple RT-priority threads.
 * \param[in] priority scheduling priority (platform-dependent; on Linux typically 1-99)
 * \returns true if configuring scheduler succeeded
 */
bool configure_sched_rr(int priority);

/**
 * Locks the memory pages of the calling thread to prevent page faults.
 * By calling this method, the programs locks all pages mapped into the address
 * space of the calling process and future mappings. This means that the kernel
 * will not swap out the pages to disk i.e., the pages are guaranteed to stay in
 * RAM until later unlocked - which is important for realtime applications.
 * \param[out] message a message describing the result of the operation
 * \returns  a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> lock_memory();

/**
 * Lock the memory like lock_memory() and additionally make sure the realtime loop does not page
 * fault on its first stack usage and heap allocations.
 *
 * Steps, in this order:
 * 1. With glibc, disable returning memory to the OS (mallopt(M_TRIM_THRESHOLD, -1)) and serving
 *    large allocations with mmap (mallopt(M_MMAP_MAX, 0)), so freed memory stays in the process
 *    heap and is reused by later allocations. This affects the whole process.
 * 2. Lock the memory with lock_memory().
 * 3. Allocate \p prefault_heap_size bytes, touch every page and free them again, so that later
 *    allocations up to that size are served from already mapped memory.
 * 4. Touch \p prefault_stack_size bytes of the calling thread's stack. Only the calling thread is
 *    prefaulted, so call it from the realtime thread (or again from each realtime thread).
 *
 * Steps 1, 3 and 4 are done even if locking fails, as they still avoid most page faults.
 * \note Once locked, all memory the process maps later on counts against RLIMIT_MEMLOCK (unless the
 * process has CAP_IPC_LOCK), so with a finite limit the prefaulted heap must fit into it as well,
 * otherwise the prefault allocation, and any later one beyond the limit, fails.
 * lock_memory() without arguments keeps its previous behavior and does none of this.
 * \param[in] prefault_stack_size bytes of the calling thread's stack to prefault, 0 to skip. Must
 * be at most half of RLIMIT_STACK to leave room for the stack already in use.
 * \param[in] prefault_heap_size bytes of heap to prefault, 0 to skip.
 * \returns a pair of a boolean that is true if the memory was locked and all prefaulting steps
 * succeeded, and a message describing the result. Returns false without changing anything if
 * \p prefault_stack_size is too large. On Windows and macOS returns false (not supported).
 */
std::pair<bool, std::string> lock_memory(
  std::size_t prefault_stack_size, std::size_t prefault_heap_size);

/**
 * Configure the caller thread affinity - Tell the scheduler to prefer a certain
 * set of cores for the given thread handle.
 * \note The threads created by the calling thread will inherit the affinity.
 * \param[in] thread the thread handle of the thread
 * \param[in] core the cpu numbers of the core. If an empty vector is passed,
 * the affinity is reset to the default.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_thread_affinity(
  NATIVE_THREAD_HANDLE thread, const std::vector<int> & cores);

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
std::pair<bool, std::string> set_thread_affinity(NATIVE_THREAD_HANDLE thread, int core);

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
 * Configure the current thread affinity - Tell the scheduler to prefer a certain
 * set of cores for the current thread.
 * \note The threads created by the calling thread will inherit the affinity.
 * \param[in] core the cpu numbers of the core. If an empty vector is passed,
 * the affinity is reset to the default.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_current_thread_affinity(const std::vector<int> & cores);

/**
 * Configure the current thread name.
 * \param[in] name The name for the thread.
 * On Linux, it will be automatically truncated to 15 characters.
 * \returns a pair of a boolean indicating whether the operation succeeded or not
 * and a message describing the result of the operation
*/
std::pair<bool, std::string> set_current_thread_name(const std::string & name);

/**
 * Method to get the amount of available cpu cores
 * \ref https://man7.org/linux/man-pages/man3/sysconf.3.html
 * \ref https://stackoverflow.com/a/150971
 * \returns The number of processors currently online (available)
*/
int64_t get_number_of_available_processors();

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__REALTIME_HELPERS_HPP_
