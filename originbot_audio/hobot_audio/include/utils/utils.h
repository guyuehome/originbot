// Copyright (c) 2022，Horizon Robotics.
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

#ifndef __USB_CAMERA_UTILS_H__
#define __USB_CAMERA_UTILS_H__

#include <time.h>

/*##################### TRACE & DEBUG #######################*/
#define TRACE_ON 0
// #undef TRCACE_ON  // uncomment it to enable function trace
#define trace_in() \
  if (TRACE_ON) printf("##function %s in\n", __func__);

#define trace_out() \
  if (TRACE_ON) printf("##function %s succeed\n", __func__);

#endif /* __USB_CAMERA_UTILS_H__ */
