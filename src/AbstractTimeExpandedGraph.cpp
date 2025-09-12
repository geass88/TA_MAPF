/**
 * @file AbstractTimeExpandedGraph.cpp
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

#include "AbstractTimeExpandedGraph.hpp"

namespace unisalento
{
    void AbstractTimeExpandedGraph::updateBorder(int i, int t)
    {
        const int u{(t - 1) * numLocations + i};
        const int v{t * numLocations + i};
        borderVertices.erase(u); // elimina u dalla lista dei borderVertices (se presente)
        if (!exists(i, t +
                           1)) // se nella rete tempo-espansa non esiste la sua versione all'istante t+1, indicalo come nuovo borderVertex
        {
            borderVertices.emplace(v);
        }
    }

    AbstractTimeExpandedGraph::AbstractTimeExpandedGraph(const std::shared_ptr<MyDirectedGraph> g)
        : layoutGraph(g), timeExpandedNodes(CF_HORIZON_UB), numLocations(g->getNumVertices())
    {
    }

    int AbstractTimeExpandedGraph::getTime(int u) const
    {
        return u / numLocations;
    }

    int AbstractTimeExpandedGraph::getLocation(int u) const
    {
        return u % numLocations;
    }

    Dim2D AbstractTimeExpandedGraph::getPair(int u) const
    {
        return {u % numLocations, u / numLocations};
    }

    int AbstractTimeExpandedGraph::getVertex(int i, int t) const
    {
        return t * numLocations + i;
    }

    bool AbstractTimeExpandedGraph::exists(int i, int t) const
    {
        if (t < 0)
        {
            return false;
        }
        const auto &V{timeExpandedNodes.at(t)};
        return V.find(i) != V.cend();
    }

    bool AbstractTimeExpandedGraph::exists(int u) const
    {
        auto [i, t]{getPair(u)};
        return exists(i, t);
    }

    void AbstractTimeExpandedGraph::forEachVertex(std::function<void(int)> functor) const
    {
        for (int t{0}; t <= horizon; t++) // per ogni istante nell'orizzonte temporale corrente
        {
            for (auto [i, v] : timeExpandedNodes.at(t))
            {
                const int u{getVertex(i, t)}; // vertice tempo-espanso (i, t)
                functor(u);
            }
        }
    }

    void AbstractTimeExpandedGraph::forEachArc(int goal, std::function<void(int, int)> functor) const
    {
        for (int t{0}; t <= horizon; t++) // per ogni istante nell'orizzonte temporale corrente
        {
            for (auto [i, s] : timeExpandedNodes.at(t))
            {
                const int u{getVertex(i, t)}; // vertice tempo-espanso (i, t)
                for (int v : outNeighbors(u))
                {
                    functor(u, v);
                }
                if (goal == i) // se il vertice i è il goal, richiama il functor anche per l'arco dummy
                {
                    const int v{getVertex(i, CF_HORIZON_UB)};
                    functor(u, v);
                }
            }
        }
    }

    void AbstractTimeExpandedGraph::dot(std::ostream &os, const Solution &sol) const
    {
        os << "digraph G1 {" << std::endl;
        os << "fontsize=10;" << std::endl;
        os << "margin=0;" << std::endl;
        mymap<int, int> stateToAgent;
        int h{0};
        for (const auto &path : sol)
        {
            for (const auto &s : path)
            {
                stateToAgent.emplace(getVertex(s.getLocation(), s.getTime()), h);
            }
            ++h;
        }
        this->forEachVertex([&](int u)
                            {
            os << "v" << u << "[shape=circle, label=\"" << getLocation(u) << ", " << getTime(u) << "\"";
            auto it{stateToAgent.find(u)};
            if (it != stateToAgent.cend()) {
                os << ", color=\"" << COLORS[it->second] << "\", style=\"filled,rounded\"";
            }
            os << "]" << std::endl; });
        this->forEachArc(-1, [&](int u, int v)
                         { os << "v" << u << " -> v" << v << " [color=black]" << std::endl; });
        os << "}" << std::endl;
    }

    std::ostream &operator<<(std::ostream &os, const AbstractTimeExpandedGraph &rhs)
    {
        for (int t{0}; t <= rhs.horizon; t++)
        {
            os << t << ": ";
            for (auto [i, v] : rhs.timeExpandedNodes.at(t))
            {
                os << i << " ";
            }
            os << std::endl;
        }
        return os;
    }
}
