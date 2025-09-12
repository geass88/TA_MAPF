/**
 * @file VertexConstraint.hpp
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

namespace unisalento {
    /**
     * @brief Modella il concetto di vincolo di non attraversabilità di un vertice.
     *
     */
    class VertexConstraint : public Hashable {
        friend class Environment;

    private:
        int time;
        int location;

    public:
        VertexConstraint();

        VertexConstraint(int t, int l);

        bool operator==(const VertexConstraint &rhs) const;

        virtual std::size_t hash() const override;

        bool sameLocation(const VertexConstraint &rhs) const;

        explicit operator std::string() const;
    };
}
