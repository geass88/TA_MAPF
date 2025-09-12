/**
 * @file Agent.cpp
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

#include "Agent.hpp"

namespace unisalento
{
    Agent::Agent(std::string n, const int s, const int g) : name(std::move(n)), start(s), goal(g) {}

    Agent::operator std::string() const
    {
        return "(" + name + ", " + std::to_string(start) + ", " + std::to_string(goal) + ")";
    }

    int Agent::getStart() const
    {
        return start;
    }

    int Agent::getGoal() const
    {
        return goal;
    }

    void Agent::setStart(const int s)
    {
        this->start = s;
    }

    void Agent::setGoal(const int g)
    {
        this->goal = g;
    }
}
