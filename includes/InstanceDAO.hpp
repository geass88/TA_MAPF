/**
 * @file InstanceDAO.hpp
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
#include "GridGraph.hpp"

namespace unisalento
{
    Solution readSolution(std::string_view solFile, std::shared_ptr<GridGraph> graph);

#if false
    Environment readMap(std::string_view problemInstance);
#endif

    Environment
    readInstance(const Param &params, std::string_view problemInstance, std::string_view preprocessing = "");

    /**
     * @brief Write an instance for "Real-Time Schedule Adjustments for Conflict-Free Vehicle Routing"
     *
     * @param problemInstance
     * @param preprocessing
     */
    void
    writeRec2Instance(std::string_view folder, std::string_view name, const Environment &env, const Solution &solution);

    void runServer(const Environment &env, const Solution &solution);

} // namespace unisalento
