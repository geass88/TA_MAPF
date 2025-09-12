/**
 * @file Conflict.cpp
 * @author Tommaso Adamo (tommaso.adamo@unisalento.it)
 * @brief
 *
 * @copyright Copyright (c) 2023 - University of Salento
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Conflict.hpp"
#include "commons.hpp"

namespace unisalento
{

    Conflict::Conflict()
    {
        type = time1 = time2 = agent1 = agent2 = location1 = location2 = -1;
    }

    Conflict::Conflict(int t, int a1, int a2, int l) : time1(t), time2(t), type(VERTEX_CONFLICT), agent1(a1),
                                                       agent2(a2), location1(l), location2(l) {}

    Conflict::Conflict(int t1, int t2, int a1, int a2, int l1, int l2) : time1(t1), time2(t2), type(EDGE_CONFLICT),
                                                                         agent1(a1), agent2(a2), location1(l1),
                                                                         location2(l2) {}

    Conflict::operator std::string() const
    {
        return "(" + std::to_string(time1) + ", " + std::to_string(time2) + ", " + std::to_string(agent1) + ", " +
               std::to_string(agent2) + ", " + std::to_string(location1) + ", " + std::to_string(location2) + ")";
    }

    bool Conflict::absent() const
    {
        return type == -1;
    }
}
