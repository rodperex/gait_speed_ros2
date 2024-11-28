// Copyright 2024 Rodrigo Pérez-Rodríguez
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


#ifndef GAITSPEED_STATES_HPP_
#define GAITSPEED_STATES_HPP_

#include "gait_speed_ros2/orchestrators/gait_speed_states.hpp"

namespace gait_speed
{

enum class State : int {
    INIT = 0,
    FIND = 1,
    EXPLAIN = 2,
    PREPARE = 3,
    MEASURE = 4,
    STOP = 5,
    ERROR = 6,
};


} // namespace gait_speed

#endif  // GAITSPEED_STATES_HPP_
