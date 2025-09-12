/**
 * @file BoostDirectedGraph.cpp
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

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "BoostDirectedGraph.hpp"
#include "commons.hpp"
#include <thread>

namespace unisalento
{
    BoostDirectedGraph::BoostDirectedGraph(const int N) : GraphBase(N) {}

    int BoostDirectedGraph::getNumVertices() const
    {
        return boost::num_vertices(*this);
    }

    int BoostDirectedGraph::getNumArcs() const
    {
        return boost::num_edges(*this);
    }

    const BoostDirectedGraph &BoostDirectedGraph::addEdge(const int u, const int v, const int weight)
    {
        boost::add_edge(u, v, weight, *this);
        return *this;
    }

    int BoostDirectedGraph::getWeight(const int u, const int v) const
    {
        return boost::get(boost::edge_weight_t(), *this, boost::edge(u, v, *this).first);
    }

    int BoostDirectedGraph::getWeight(boost::graph_traits<BoostDirectedGraph>::out_edge_iterator it) const
    {
        return boost::get(boost::edge_weight_t(), *this, *it);
    }

    int BoostDirectedGraph::getOutDegree(const int u) const
    {
        return boost::out_degree(u, *this);
    }

    const std::vector<int> BoostDirectedGraph::getOutNeighborsVector(const int u) const
    {
        std::vector<int> outgoing;
        boost::graph_traits<BoostDirectedGraph>::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::out_edges(u, *this); ei != ei_end; ++ei)
        {
            outgoing.emplace_back(boost::target(*ei, *this));
        }
        return outgoing;
    }

    const std::vector<int> BoostDirectedGraph::getInNeighborsVector(const int u) const
    {
        std::vector<int> incoming;
        boost::graph_traits<BoostDirectedGraph>::in_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = boost::in_edges(u, *this); ei != ei_end; ++ei)
        {
            incoming.emplace_back(boost::source(*ei, *this));
        }
        return incoming;
    }

    std::pair<boost::graph_traits<BoostDirectedGraph>::out_edge_iterator, boost::graph_traits<BoostDirectedGraph>::out_edge_iterator>
    BoostDirectedGraph::getOutNeighbors(const int u) const
    {
        return boost::out_edges(u, *this);
    }

    std::pair<boost::graph_traits<BoostDirectedGraph>::in_edge_iterator, boost::graph_traits<BoostDirectedGraph>::in_edge_iterator>
    BoostDirectedGraph::getInNeighbors(const int u) const
    {
        return boost::in_edges(u, *this);
    }

    int BoostDirectedGraph::getTarget(boost::graph_traits<BoostDirectedGraph>::out_edge_iterator it) const
    {
        return boost::target(*it, *this);
    }

    int BoostDirectedGraph::getSource(boost::graph_traits<BoostDirectedGraph>::in_edge_iterator it) const
    {
        return boost::source(*it, *this);
    }

    void helper(int tid, const BoostDirectedGraph &graph, std::vector<int> &dist, std::vector<int> &pred)
    {
        const int n{graph.getNumVertices()};
        for (int i{tid}; i < n; i += NUM_THREADS)
        {
            std::vector<int> p(n);
            std::vector<int> d(n);
            int s = boost::vertex(i, graph);
            boost::dijkstra_shortest_paths(graph, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));
            for (int j{0}; j < n; j++)
            {
                // if (d[j] == INT_MAX)
                if (p[j] == j)
                {
                    assert((j == s && d[s] == 0) || d[j] == INT_MAX);
                    dist[i * n + j] = INF;
                    pred[i * n + j] = -1;
                }
                else
                {
                    dist[i * n + j] = d[j];
                    pred[i * n + j] = p[j];
                }
            }
            dist[i * n + i] = 0;
            pred[i * n + i] = -1;
        }
    }

    std::pair<std::vector<int>, std::vector<int>> BoostDirectedGraph::parallelAllPairsShortestPath() const
    {
        const int n{this->getNumVertices()};
        std::vector<int> dist(n * n, INF);
        std::vector<int> pred(n * n, -1);
        std::vector<std::thread> threads;
        // create threads
        for (int i{0}; i < NUM_THREADS; i++)
        {
            threads.emplace_back(helper, i, std::cref(*this), std::ref(dist), std::ref(pred));
        }
        // wait for them to complete
        for (auto &th : threads)
            th.join();
        return {dist, pred};
    }

    std::pair<std::vector<int>, std::vector<int>> BoostDirectedGraph::allPairsShortestPath() const
    {
        const int n{this->getNumVertices()};
        std::vector<int> dist(n * n, INF);
        std::vector<int> pred(n * n, -1);
        for (int i{0}; i < n; i++)
        {
            std::vector<int> p(n);
            std::vector<int> d(n);
            int s = boost::vertex(i, *this);
            boost::dijkstra_shortest_paths(*this, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));
            for (int j{0}; j < n; j++)
            {
                // if (d[j] == INT_MAX)
                if (p[j] == j)
                {
                    assert((j == s && d[s] == 0) || d[j] == INT_MAX);
                    dist[i * n + j] = INF;
                    pred[i * n + j] = -1;
                }
                else
                {
                    dist[i * n + j] = d[j];
                    pred[i * n + j] = p[j];
                }
            }
            dist[i * n + i] = 0;
            pred[i * n + i] = -1;
        }
        return {dist, pred};
    }

    std::pair<std::vector<int>, std::vector<int>> BoostDirectedGraph::floydWarshall() const
    {
        const int n{this->getNumVertices()};
        std::vector<int> dist(n * n, INF);
        std::vector<int> pred(n * n, -1);
        for (int i{0}; i < n; ++i)
        {
            auto neighbors{getOutNeighbors(i)};
            for (auto ei{neighbors.first}; ei != neighbors.second; ei++)
            {
                int j{static_cast<int>(boost::target(*ei, *this))};
                dist[i * n + j] = this->getWeight(ei);
                pred[i * n + j] = i;
            }
        }

        for (int i{0}; i < n; ++i)
        {
            dist[i * n + i] = 0;
            // pred[i * n + i] = i;
        }
        for (int k{0}; k < n; ++k)
            for (int i{0}; i < n; ++i)
                for (int j{0}; j < n; ++j)
                {
                    int v1{dist[i * n + k]};
                    int v2{dist[k * n + j]};
                    int &v3{dist[i * n + j]};
                    int sum{v1 + v2};
                    if (v1 != INF && v2 != INF && v3 > sum)
                    {
                        v3 = sum;
                        pred[i * n + j] = pred[k * n + j];
                    }
                }
        return {dist, pred};
    }

    std::pair<int, int> BoostDirectedGraph::getXY(const int u) const
    {
        return {u, u};
    }

    std::ostream &operator<<(std::ostream &os, const BoostDirectedGraph &rhs)
    {
        for (int i{0}; i < rhs.getNumVertices(); i++)
        {
            printCollection(os, rhs.getOutNeighborsVector(i));
        }
        return os;
    }
}
