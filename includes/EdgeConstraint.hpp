/**
 * @file EdgeConstraint.hpp
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
     * @brief Modella il concetto di vincolo di non attraversabilità di un arco.
     *
     */
    class EdgeConstraint : public Hashable
    {
    private:
        int time;
        int location1;
        int location2;

    public:
        EdgeConstraint();

        EdgeConstraint(int t, int l1, int l2);

        bool operator==(const EdgeConstraint &rhs) const;

        virtual std::size_t hash() const override;

        /*bool sameLocation(const EdgeConstraint &rhs) const
        {
            return location1 == rhs.location1 && location2 == rhs.location2;
        }*/
        explicit operator std::string() const;
    };
}
