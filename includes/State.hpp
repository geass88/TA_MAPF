/**
 * @file State.hpp
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

#include "Hashable.hpp"
#include <string>

namespace unisalento
{
    /**
     * @brief Modella lo stato di un agente (posizione ed istante di tempo).
     *
     */
    class State : public Hashable
    {
        friend class Environment;

    protected:
        int time;
        int location;

    public:
        State();

        State(int t, int l);

        int getTime() const;

        int getLocation() const;

        bool operator==(const State &rhs) const;

        virtual std::size_t hash() const override;

        bool sameLocation(const State &rhs) const;

        operator std::string() const;

        friend std::ostream &operator<<(std::ostream &out, const State &rhs);
    };
}
