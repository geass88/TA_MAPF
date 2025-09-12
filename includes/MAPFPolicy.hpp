/**
 * @file MAPFPolicy.hpp
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
    /**
     * @brief Definisce l'interfaccia di base per un algoritmo di MAPF.
     *
     */
    class MAPFPolicy
    {
    protected:
        Environment *const env;

    public:
        explicit MAPFPolicy(Environment *const _env = nullptr);

        virtual Solution search() = 0;

        static void printPlan(std::ostream &out, const Solution &solution);

        void exportPlan(std::ostream &out, const Solution &solution) const;

        virtual void stop() { std::exit(EXIT_FAILURE); }

        virtual ~MAPFPolicy() {}
    };
} // namespace unisalento
