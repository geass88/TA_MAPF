/**
 * @file GridGraph.cpp
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

#include "GridGraph.hpp"
#include "commons.hpp"

namespace unisalento
{
    void GridGraph::set(const int i, const int j, const bool value)
    {
        layout.at((i + 1) * (dimension.second + 2) + j + 1) = value;
    }

    GridGraph::GridGraph(const Dim2D &d, const myset<int> &obstacles) : MyDirectedGraph(
                                                                            d.first * d.second - obstacles.size())
    {
        this->dimension = d;
        int m{d.first};
        int n{d.second};
        this->layout.resize((m + 2) * (n + 2), true);
        for (int j{-1}; j <= n; ++j)
        {
            set(-1, j, false);
            set(m, j, false);
        }
        for (int i{-1}; i <= m; ++i)
        {
            set(i, -1, false);
            set(i, n, false);
        }
        for (int o : obstacles)
        {
            set(o / n, o % n, false);
        }

        this->loc2id.resize(m * n);
        this->id2loc.resize(m * n - obstacles.size());
        int id{0};
        for (int i{0}; i < m; ++i)
        {
            for (int j{0}; j < n; j++)
            {
                if (this->at(i, j))
                {
                    loc2id[i * n + j] = id;
                    id2loc[id] = i * n + j;
                    id++;
                }
                else
                {
                    loc2id[i * n + j] = -1;
                }
            }
        }

        for (int i{0}; i < m; ++i)
        {
            for (int j{0}; j < n; j++)
            {
                if (!this->at(i, j))
                    continue;
                const Dim2D neighbors[]{
                    //{0,0}, // same node: self arc
                    {-1, 0}, // left
                    {1, 0},  // right
                    {0, 1},  // up
                    {0, -1}  // down
                };

                for (const auto &p : neighbors)
                {
                    int p_i{i + p.first};
                    int p_j{j + p.second};
                    if (this->at(p_i, p_j))
                    {
                        // boost::add_edge(i * n + j, p_i * n + p_j, *this);
                        // this->addEdge(i * n + j, p_i * n + p_j, STEP_COST);
                        this->addEdge(loc2id[i * n + j], loc2id[p_i * n + p_j], STEP_COST);
                    }
                }
            }
        }
    }

#if 0
    GridGraph::GridGraph(const Dim2D &d, const myset<int> &obstacles) : MyDirectedGraph(d.first * d.second)
    {
        this->dimension = d;
        int m{d.first};
        int n{d.second};
        this->layout.resize((m + 2) * (n + 2), true);
        for (int j{-1}; j <= n; ++j) {
            set(-1, j, false);
            set(m, j, false);
        }
        for (int i{-1}; i <= m; ++i) {
            set(i, -1, false);
            set(i, n, false);
        }
        for (int o : obstacles) {
            set(o / n, o % n, false);
        }

        for (int i{0}; i < m; ++i) {
            for (int j{0}; j < n; j++) {
                if (!this->at(i, j))
                    continue;
                Dim2D neighbors[] {
                    //{0,0}, // same node: self arc
                    {-1, 0}, // left
                    {1, 0},  // right
                    {0, 1},  // up
                    {0, -1}  // down
                };

                for (const auto &p : neighbors) {
                    int p_i{i + p.first};
                    int p_j{j + p.second};
                    if (this->at(p_i, p_j)) {
                        // boost::add_edge(i * n + j, p_i * n + p_j, *this);
                        this->addEdge(i * n + j, p_i * n + p_j, STEP_COST);
                    }
                }
            }
        }
    }
#endif

    /*virtual const std::vector<AdjacentNode> GridGraph::getOutNeighbors(const int u) const
    {
        int i{u / dimension.second};
        int j{u % dimension.second};
        Dim2D neighbors[]{
            //{0,0}, // same node: self arc
            {-1, 0}, // left
            {1, 0},  // right
            {0, 1},  // up
            {0, -1}  // down
        };
        std::vector<AdjacentNode> v;
        v.reserve(4);
        for (const auto &p : neighbors)
        {
            int p_i{i + p.first};
            int p_j{j + p.second};
            if (this->at(p_i, p_j))
                v.emplace_back(p_i * dimension.second + p_j, 1);
        }
        // std::sort(v.begin(), v.end());
        // assert(std::equal(v.cbegin(), v.cend(), outgoing[u].begin()));

        return v;
    }*/

    bool GridGraph::at(const int i, const int j) const
    {
        return layout.at((i + 1) * (dimension.second + 2) + j + 1);
    }

    /*
    int row(int id) const
    {
        return id / (dimension.second + 2) - 1;
    }
    int rrow(int id) const
    {
        return id / dimension.second;
    }
    int column(int id) const
    {
        return id % (dimension.second + 2) - 1;
    }
    int ccolumn(int id) const
    {
        return id % dimension.second;
    }*/

    int GridGraph::L1norm(const int u, const int v) const
    {
        // return abs(u / dimension.second - v / dimension.second) + abs(u % dimension.second - v % dimension.second);
        auto xyU{getXY(u)};
        auto xyV{getXY(v)};
        return abs(xyU.first - xyV.first) + abs(xyU.second - xyV.second);
    }

    Dim2D GridGraph::getDimension() const
    {
        return dimension;
    }

    Dim2D GridGraph::getXY(const int u) const
    {
        int loc{id2loc.at(u)};
        return {loc / dimension.second, loc % dimension.second};
    }

    int GridGraph::getId(Dim2D xy) const
    {
        // return xy.first * dimension.second + xy.second;
        return loc2id.at(xy.first * dimension.second + xy.second);
    }

    int GridGraph::getId(const int x, const int y) const
    {
        // return x * dimension.second + y
        return loc2id.at(x * dimension.second + y);
    }

    int GridGraph::getId(const int xy) const
    {
        // return x * dimension.second + y
        return loc2id.at(xy); // getId(xy / dimension.second, xy % dimension.second);
    }

    int GridGraph::getRows() const
    {
        return dimension.first;
    }

    int GridGraph::getColumns() const
    {
        return dimension.second;
    }

    int GridGraph::countBifurcatedVertices() const
    {
        int counter{0};
        for (int i{0}; i < getNumVertices(); i++)
        {
            if (this->getOutDegree(i) >= 3) // boost::out_degree(i, *this)
                counter++;
        }
        return counter;
    }

    void GridGraph::dot(std::ostream &os) const
    {
        os << "graph G1 {" << std::endl;
        os << "fontsize=10;" << std::endl;
        os << "margin=0;" << std::endl;
        os << "layout=fdp;" << std::endl;

        for (auto it{boost::vertices(*this)}; it.first != it.second; ++it.first)
        {
            auto u{*(it.first)};
            auto [x, y]{getXY(u)};
            os << "v" << u << "[shape=circle, label=\"" << u << "\", pos=\"" << x << "," << y << "!\"]" << std::endl;
        }
        for (auto it{boost::edges(*this)}; it.first != it.second; ++it.first)
        {
            auto e{*(it.first)};
            if (e.m_source < e.m_target)
            {
                continue;
            }
            os << "v" << e.m_source << " -- v" << e.m_target << " [color=black]" << std::endl;
        }
        os << "}" << std::endl;
    }

}
