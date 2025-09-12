/**
 * @file AbstractTimeExpandedGraph.hpp
 * @author Tommaso Adamo (tommaso.adamo@unisalento.it)
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

#pragma once

#include "Environment.hpp"

namespace unisalento
{
    constexpr int CF_HORIZON_UB{800}; // UB on time
    /**
     * @brief Definisce l'interfaccia principale del grafo tempo-espanso.
     *
     */
    class AbstractTimeExpandedGraph
    {
        friend class MCNF;

        friend class MCNF2;

    protected:
        const std::shared_ptr<MyDirectedGraph> layoutGraph;
        std::vector<mymap<int, int>> timeExpandedNodes; // time-expanded network id = (r, t) = t * m + r  // numInstants x numVertices matrix
        // std::vector<std::set<int>> nodeExpandedTimes; // // dato un nodo i restituisce tutti gli istanti ordinati in cui è presente una replica temporale
        myset<int> borderVertices;
        int horizon{0};
        const int numLocations;

        void updateBorder(int i, int t);

        // virtual void adjustArcs(int u) = 0;

    public:
        AbstractTimeExpandedGraph(const std::shared_ptr<MyDirectedGraph> g);

        int getTime(int u) const;

        int getLocation(int u) const;

        Dim2D getPair(int u) const;

        int getVertex(int i, int t) const;

        bool exists(int i, int t) const;

        bool exists(int u) const;

        /**
         * @brief Ritorna gli out-neighbors di un vertice tempo-espanso
         *
         * @param u
         * @return std::vector<int>
         */
        virtual std::vector<int> outNeighbors(int u) const = 0;

        /**
         * @brief Ritorna gli in-neighbors di un vertice tempo-espanso
         *
         * @param u
         * @return std::vector<int>
         */
        virtual std::vector<int> inNeighbors(int u) const = 0;

        virtual std::pair<mymap<int, int>::iterator, bool> insert(int i, int t) = 0;

        virtual void timeExpand() = 0;

        // virtual void timeExpand2() = 0;

        void forEachVertex(std::function<void(int)>) const;

        void forEachArc(int goal, std::function<void(int, int)>) const;

        void dot(std::ostream &os, const Solution &sol = {}) const;

        friend std::ostream &operator<<(std::ostream &os, const AbstractTimeExpandedGraph &rhs);
    };
} // namespace unisalento
