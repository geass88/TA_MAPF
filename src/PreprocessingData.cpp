/**
 * @file PreprocessingData.cpp
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

#include "PreprocessingData.hpp"
#include <filesystem>

namespace unisalento
{
    PreprocessingData::PreprocessingData(std::string_view prepFile, const int n) : numVertices(n)
    {
        quickestCosts.resize(numVertices * numVertices);
        quickestPaths.resize(numVertices * numVertices);
        std::ifstream fin(prepFile.data());
        for (int i{0}; i < numVertices * numVertices; i++)
            fin >> quickestCosts[i];
        for (int i{0}; i < numVertices * numVertices; i++)
            fin >> quickestPaths[i];
        fin.close();
    }

    PreprocessingData::PreprocessingData(std::shared_ptr<MyDirectedGraph> graph)
    {
        this->numVertices = graph->getNumVertices();
        auto pair{graph->parallelAllPairsShortestPath()};
        this->quickestCosts = std::move(pair.first);
        this->quickestPaths = std::move(pair.second);
    }

    std::ostream &operator<<(std::ostream &out, const PreprocessingData &rhs)
    {
        for (int v : rhs.quickestCosts)
            out << v << " ";
        out << std::endl;
        for (int v : rhs.quickestPaths)
            out << v << " ";
        out << std::endl;
        return out;
    }

    std::list<int> PreprocessingData::quickestPath(const int from, const int to) const
    {
        std::list<int> path;
        // const int start{agents[agentId].start};
        // const int goal{agents.at(assignment.at(agentId)).goal};
        int c{to};
        while (c != -1)
        {
            path.push_front(c);
            c = quickestPaths[from * numVertices + c];
        }
        return path;
    }

    int PreprocessingData::quickestCost(const int from, const int to) const
    {
        // const int goal{agents.at(assignment.at(agentId2)).goal};
        // return quickestCosts[agents[agentId1].start * graph->getNumVertices() + goal];
        return quickestCosts[from * numVertices + to];
    }

    const std::vector<int> &PreprocessingData::getQuickestPaths() const
    {
        return this->quickestPaths;
    }

    const std::vector<int> &PreprocessingData::getQuickestCosts() const
    {
        return this->quickestCosts;
    }

    int PreprocessingData::getNumVertices() const
    {
        return this->numVertices;
    }

} // namespace unisalento
