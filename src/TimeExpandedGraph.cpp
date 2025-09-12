/**
 * @file TimeExpandedGraph.cpp
 * @author Tommaso Adamo (tommaso.adamo@unisalento.it)
 * @brief
 *
 * @copyright Copyright (c) 2024 - University of Salento
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

#include "TimeExpandedGraph.hpp"

namespace unisalento
{
    TimeExpandedGraph::TimeExpandedGraph(const std::shared_ptr<MyDirectedGraph> g)
        : AbstractTimeExpandedGraph(g)
    {
    }

    std::vector<int> TimeExpandedGraph::outNeighbors(int u) const
    {
        std::vector<int> n;
        n.reserve(5);
        const int t{u / numLocations + 1};
        const int i{u % numLocations};
        if (exists(i, t))
        {
            n.emplace_back(getVertex(i, t));
        }
        auto candidates{layoutGraph->getOutNeighbors(i)};
        for (auto ei{candidates.first}; ei != candidates.second; ei++)
        {
            const int j{layoutGraph->getTarget(ei)};
            if (exists(j, t))
            {
                n.emplace_back(getVertex(j, t));
            }
        }
        return n;
    }

    std::vector<int> TimeExpandedGraph::inNeighbors(int u) const
    {
        std::vector<int> n;
        n.reserve(5);
        const int t{u / numLocations - 1};
        const int j{u % numLocations};
        if (exists(j, t))
        {
            n.emplace_back(getVertex(j, t));
        }
        auto candidates{layoutGraph->getInNeighbors(j)};
        for (auto ei{candidates.first}; ei != candidates.second; ei++)
        {
            const int i{layoutGraph->getSource(ei)};
            if (exists(i, t))
            {
                n.emplace_back(getVertex(i, t));
            }
        }
        return n;
    }

    std::pair<mymap<int, int>::iterator, bool> TimeExpandedGraph::insert(int i, int t)
    {
        const auto &status{timeExpandedNodes.at(t).emplace(i, t * numLocations + i)};
        if (status.second)
        { // ho inserito un nuovo nodo tempo espanso che non esisteva, quindi devo aggiornare i borderVertices
            updateBorder(i, t);
            // TODO: aggiornare i conflicting arcs
            // adjustArcs(v);
            if (t > horizon)
            {
                horizon = t;
            }
        }
        // else, il nodo tempo espanso esiste già
        return status;
    }

    void TimeExpandedGraph::timeExpand()
    {
        myset<int> newBorderVertices;
        for (int u : borderVertices)
        {
            const int t{u / numLocations + 1};
            const int i{u % numLocations};
            const int v{t * numLocations + i};
            const auto &status{timeExpandedNodes.at(t).emplace(i, v)};
            if (status.second) // ho inserito un nuovo nodo tempo espanso che non esisteva
            {
                if (!exists(i, t +
                                   1)) // se nella rete tempo-espansa non esiste la sua versione all'istante t+1, indicalo come nuovo borderVertex
                {
                    newBorderVertices.emplace(v);
                }
                // adjustArcs(v);
                if (t > horizon)
                {
                    horizon = t;
                }
            }
            else // il nodo tempo espanso esiste già... quindi u non era border!
            {
                throw "Border vertex error";
            }
        }
        borderVertices = std::move(newBorderVertices);
    }

    /*void TimeExpandedGraph::timeExpand()
    {
        myset<int> oldBorderVertices{borderVertices};
        for (int u : oldBorderVertices)
        {
            const int t{u / numLocations + 1};
            const int i{u % numLocations};
            const int v{t * numLocations + i};
            const auto &status{timeExpandedNodes.at(t).emplace(i, v)};
            if (status.second) // ho inserito un nuovo nodo tempo espanso che non esisteva
            {
                updateBorder(i, t);
                // adjustArcs(v);
                if (t > makespan)
                {
                    makespan = t;
                }
            }
            else // il nodo tempo espanso esiste già... quindi u non era border!
            {
                throw "Border vertex error";
            }
        }
    }*/
}
