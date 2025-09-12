/**
 * @file EdgeConstraint.cpp
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

#include "EdgeConstraint.hpp"
// #include "commons.hpp"
#include <boost/functional/hash.hpp>

namespace unisalento
{
    EdgeConstraint::EdgeConstraint() : time(-1), location1(-1), location2(-1) {}

    EdgeConstraint::EdgeConstraint(int t, int l1, int l2) : time(t), location1(l1), location2(l2) {}

    bool EdgeConstraint::operator==(const EdgeConstraint &rhs) const
    {
        return time == rhs.time && location1 == rhs.location1 && location2 == rhs.location2;
    }

    std::size_t EdgeConstraint::hash() const
    {
        // return std::hash<std::string>{}(static_cast<std::string>(*this)); // std::string(*this)
        size_t seed{0};
        boost::hash_combine(seed, time);
        boost::hash_combine(seed, location1);
        boost::hash_combine(seed, location2);
        return seed;
    }

    /*bool sameLocation(const EdgeConstraint &rhs) const
    {
        return location1 == rhs.location1 && location2 == rhs.location2;
    }*/
    EdgeConstraint::operator std::string() const
    {
        return "(" + std::to_string(time) + ", " + std::to_string(location1) + ", " + std::to_string(location2) + ")";
    }
}
