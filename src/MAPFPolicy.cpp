/**
 * @file MAPFPolicy.cpp
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

#include "MAPFPolicy.hpp"

namespace unisalento {
    MAPFPolicy::MAPFPolicy(Environment *const _env) : env(_env) {}

    void MAPFPolicy::printPlan(std::ostream &out, const Solution &solution) {
        for (const auto &path: solution) {
            printCollection(out, path);
        }
    }

    void MAPFPolicy::exportPlan(std::ostream &out, const Solution &solution) const {
        // const Dim2D dimension
        out << "cost: " << env->computeSolutionCost(solution) << std::endl;
        out << "schedule:" << std::endl;
        int i{0};
        auto ptr{this->env->getLayoutGraph()};
        for (const auto &path: solution) {
            out << "  agent" << i++ << ":" << std::endl;
            // printCollection(out, path);
            for (const auto &s: path) {
                Dim2D xy{ptr->getXY(s.getLocation())};
                out << "  - t: " << s.getTime() << "\n    x: " << xy.first << "\n    y: " << xy.second << std::endl;
            }
        }
    }

} // namespace unisalento
