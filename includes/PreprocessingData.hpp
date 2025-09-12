/**
 * @file PreprocessingData.hpp
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

#include "commons.hpp"
#include "BoostDirectedGraph.hpp"

namespace unisalento {
    /**
     * @brief Classe per la memorizzazione dei quickest path/cost in un grafo di layout.
     *
     */
    class PreprocessingData {
    private:
        int numVertices{0};
        std::vector<int> quickestCosts;
        std::vector<int> quickestPaths;

    public:
        PreprocessingData(std::string_view prepFile, const int n);

        PreprocessingData(std::shared_ptr<MyDirectedGraph> graph);

        friend std::ostream &operator<<(std::ostream &out, const PreprocessingData &rhs);

        std::list<int> quickestPath(const int from, const int to) const;

        int quickestCost(const int from, const int to) const;

        const std::vector<int> &getQuickestPaths() const;

        const std::vector<int> &getQuickestCosts() const;

        int getNumVertices() const;
    };
} // namespace unisalento
