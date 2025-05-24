// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2024, Lennart Nachtigall
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
//    * Neither the name of the Willow Garage, Inc. nor the names of its
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

// Author: Stuart Glaser
// Author: Lennart Nachtigall

#ifndef REALTIME_TOOLS__REALTIME_BOX_HPP_
#define REALTIME_TOOLS__REALTIME_BOX_HPP_

#include "realtime_tools/realtime_thread_safe_box.hpp"

// Deprecation notice
#pragma message( \
  "'RealtimeBox' is deprecated. Please update your code to use 'realtime_thread_safe_box.hpp' header, and class name 'RealtimeThreadSafeBox' instead.")  //NOLINT

namespace realtime_tools
{

// Provide backward-compatibility for the old RealtimeBox class
template <class T, typename mutex_type = std::mutex>
using RealtimeBoxBase = RealtimeThreadSafeBox<T, mutex_type>;

template <typename T>
using RealtimeBoxStandard = RealtimeBoxBase<T, std::mutex>;

template <typename T>
using RealtimeBoxRecursive = RealtimeBoxBase<T, std::recursive_mutex>;

template <typename T>
using RealtimeBox = RealtimeBoxStandard<T>;

// Only kept for compatibility reasons
template <typename T, typename mutex_type = std::mutex>
using RealtimeBoxBestEffort [[deprecated("Use RealtimeBox instead")]] =
  RealtimeBoxBase<T, mutex_type>;

}  // namespace realtime_tools

#endif  // REALTIME_TOOLS__REALTIME_BOX_HPP_
