/**
 * @file BoostDirectedGraph.hpp
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

#include <boost/graph/adjacency_list.hpp>
// #include "commons.hpp"

namespace unisalento
{
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, boost::no_property, boost::property<boost::edge_weight_t, int>> GraphBase;

    // typedef boost::adjacency_list<> GraphBase;
    /**
     * @brief Modella il grafo di layout su cui si muovono gli agenti.
     *
     */
    class BoostDirectedGraph : public GraphBase
    {
    public:
        explicit BoostDirectedGraph(const int N);

        int getNumVertices() const;

        int getNumArcs() const;

        const BoostDirectedGraph &addEdge(const int u, const int v, const int weight);

        int getWeight(const int u, const int v) const;

        int getWeight(boost::graph_traits<BoostDirectedGraph>::out_edge_iterator it) const;

        int getOutDegree(const int u) const;

        const std::vector<int> getOutNeighborsVector(const int u) const;

        const std::vector<int> getInNeighborsVector(const int u) const;

        std::pair<boost::graph_traits<BoostDirectedGraph>::out_edge_iterator, boost::graph_traits<BoostDirectedGraph>::out_edge_iterator>
        getOutNeighbors(const int u) const;

        std::pair<boost::graph_traits<BoostDirectedGraph>::in_edge_iterator, boost::graph_traits<BoostDirectedGraph>::in_edge_iterator>
        getInNeighbors(const int u) const;

        int getTarget(boost::graph_traits<BoostDirectedGraph>::out_edge_iterator it) const;

        int getSource(boost::graph_traits<BoostDirectedGraph>::in_edge_iterator it) const;

        std::pair<std::vector<int>, std::vector<int>> parallelAllPairsShortestPath() const;

        std::pair<std::vector<int>, std::vector<int>> allPairsShortestPath() const;

        std::pair<std::vector<int>, std::vector<int>> floydWarshall() const;

        virtual std::pair<int, int> getXY(const int u) const;

        friend std::ostream &operator<<(std::ostream &os, const BoostDirectedGraph &rhs);
    };
}
