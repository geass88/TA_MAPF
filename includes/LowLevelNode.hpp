/**
 * @file LowLevelNode.hpp
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

#pragma once

#include "State.hpp"

namespace unisalento
{
    /**
     * @brief Modella un sotto-problema nell'algoritmo di ricerca di basso livello (A*).
     *
     */
    class LowLevelNode
    {
        friend class AStar;

    private:
        State state;
        int fScore;
        int gScore;
        int conflicts{0};

    public:
        LowLevelNode();

        LowLevelNode(const State &s, int f, int g, int c = 0);

        bool operator<(const LowLevelNode &rhs) const;
    };
} // namespace unisalento
