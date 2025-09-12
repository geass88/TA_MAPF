/**
 * @file Constraints.cpp
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

#include "Constraints.hpp"
#include <numeric>

namespace unisalento
{

    Constraints::Constraints() {}

    bool Constraints::empty() const
    {
        return vertexConstraints.empty() && edgeConstraints.empty();
    }

    Constraints &Constraints::operator+=(const Constraints &rhs)
    {
        vertexConstraints.insert(rhs.vertexConstraints.cbegin(), rhs.vertexConstraints.cend());
        edgeConstraints.insert(rhs.edgeConstraints.cbegin(), rhs.edgeConstraints.cend());
        return *this;
    }

    bool Constraints::overlap(const Constraints &other) const
    {
        for (const auto &vc : vertexConstraints)
        {
            if (other.vertexConstraints.count(vc) > 0)
            {
                return true;
            }
        }
        for (const auto &ec : edgeConstraints)
        {
            if (other.edgeConstraints.count(ec) > 0)
            {
                return true;
            }
        }
        return false;
    }

    Constraints::operator std::string() const
    {
        auto commaFold{
            [](std::string a, auto b)
            {
                return std::move(a) + "," + std::string(b);
            }};

        auto firstV{vertexConstraints.cbegin()};
        std::string vc{vertexConstraints.empty() ? "" : std::accumulate(std::next(firstV), vertexConstraints.cend(), std::string(*firstV), commaFold)};
        auto firstE{edgeConstraints.cbegin()};
        std::string ec{edgeConstraints.empty() ? "" : std::accumulate(std::next(firstE), edgeConstraints.cend(), std::string(*firstE), commaFold)};
        return "VC: [ " + vc + " ]\nEC: [ " + ec + " ]";
    }

}
