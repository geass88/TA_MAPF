/**
 * @file Agent.hpp
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

#include <string>

namespace unisalento
{
    /**
     * @brief Definisce il concetto di Agente.
     *
     */
    class Agent
    {
        friend class Environment;
        // friend class AStar;

    private:
        std::string name;
        /// @brief posizione iniziale
        int start;
        /// @brief  posizione finale
        int goal;

    public:
        Agent(std::string n, const int s, const int g);

        explicit operator std::string() const;

        int getStart() const;

        int getGoal() const;

        void setStart(const int start);

        void setGoal(const int goal);
    };
}
