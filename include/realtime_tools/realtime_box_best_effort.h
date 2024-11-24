// Copyright 2024 ros2_control Development Team
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

#ifndef REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_H_
#define REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_H_

#include "realtime_tools/realtime_box.h"

// Deprecation notice
#ifdef _WIN32
#pragma message( \
  "This header include is deprecated. Please update your code to use 'realtime_box.h' header.")  //NOLINT
#else
#warning \
  "This header include is deprecated. Please update your code to use 'realtime_box.h' header." //NOLINT
#endif

#endif  // REALTIME_TOOLS__REALTIME_BOX_BEST_EFFORT_H_
