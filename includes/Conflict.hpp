/**
 * @file Conflict.hpp
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
     * @brief Modella il concetto di conflitto tra due agenti nell'utilizzo di una risorsa (vertice o arco) simultaneamente.
     *
     */
    class Conflict
    {
        friend class Environment;

    private:
        int time1;
        int time2;
        char type;
        int agent1;
        int agent2;
        int location1;
        int location2;

    public:
        Conflict();

        Conflict(int t, int a1, int a2, int l);

        Conflict(int t1, int t2, int a1, int a2, int l1, int l2);

        explicit operator std::string() const;

        bool absent() const;
    };
}
