/**
 * @file VertexConstraint.cpp
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

#include "VertexConstraint.hpp"
#include <boost/functional/hash.hpp>

namespace unisalento
{
    VertexConstraint::VertexConstraint() : time(-1), location(-1) {}

    VertexConstraint::VertexConstraint(int t, int l) : time(t), location(l) {}

    bool VertexConstraint::operator==(const VertexConstraint &rhs) const
    {
        return time == rhs.time && location == rhs.location;
    }

    std::size_t VertexConstraint::hash() const
    {
        // return std::hash<std::string>{}(static_cast<std::string>(*this));
        size_t seed{0};
        boost::hash_combine(seed, time);
        boost::hash_combine(seed, location);
        return seed;
    }

    bool VertexConstraint::sameLocation(const VertexConstraint &rhs) const
    {
        return location == rhs.location;
    }

    VertexConstraint::operator std::string() const
    {
        return "(" + std::to_string(time) + ", " + std::to_string(location) + ")";
    }
}
