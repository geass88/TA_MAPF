/**
 * @file AStar.hpp
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

#include "Environment.hpp"

namespace unisalento
{
    class Environment;

    /**
     * @brief Algoritmo di ricerca A* con stati e/o transizioni proibite.
     *
     */
    class AStar
    {
    private:
        Environment *const env;

        /**
         * @brief Ricostruisce un percorso usando la mappa dei predecessori.
         *
         * @param predMap
         * @param current
         * @return std::list<State>
         */
        static std::list<State> reconstructPath(const mymap<State, State, Hash> &predMap, const State &current);

    public:
        explicit AStar(Environment *const _env = nullptr);

        /**
         * @brief Low level search
         *
         * @param agentId
         */
        std::list<State> search(int agentId, const Solution &curSol) const;
    };
}
